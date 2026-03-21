#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <cmath>
#include <numeric>

class TrajectoryPlanner : public rclcpp::Node
{
public:
  TrajectoryPlanner() : Node("trajectory_planner")
  {
    // Parameters
    this->declare_parameter("waypoints_file", "");
    this->declare_parameter("waypoint_radius", 0.3);
    this->declare_parameter("hover_time", 2.0);
    this->declare_parameter("loop_trajectory", false);
    
    waypoint_radius_ = this->get_parameter("waypoint_radius").as_double();
    hover_time_ = this->get_parameter("hover_time").as_double();
    loop_trajectory_ = this->get_parameter("loop_trajectory").as_bool();
    std::string waypoints_file = this->get_parameter("waypoints_file").as_string();

    // QoS for PX4
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.transient_local();

    // Publishers
    pub_setpoint_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/control/position_setpoint", 10);
      
    pub_arm_command_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_position_controller/commands", 10);

    // Subscriber
    sub_position_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos,
      std::bind(&TrajectoryPlanner::positionCallback, this, std::placeholders::_1));

    // Timer for Control Loop (50 Hz for smooth arm movement)
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),  
      std::bind(&TrajectoryPlanner::controlLoop, this));

    // Load drone waypoints
    if (waypoints_file.empty()) {
      loadDefaultWaypoints();
    } else {
      loadWaypointsFromFile(waypoints_file);
    }

    RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
    RCLCPP_INFO(get_logger(), "  Trajectory & Arm Swing Tester Started");
    RCLCPP_INFO(get_logger(), "  Arm: Continuous Sine Wave (starts > 1.0m altitude)");
    RCLCPP_INFO(get_logger(), "  Metrics: Drone Position Tracking Only");
    RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
  }

private:
  struct Waypoint {
    double x, y, z;
    std::string name;
  };

  // --- Metrics ---
  double sum_sq_error_pos_ = 0.0;
  double max_error_pos_ = 0.0;
  int sample_count_ = 0;
  bool metrics_started_ = false;

  void loadDefaultWaypoints()
  {
    // Drone Flight Path Only
    waypoints_ = {
      {0.0, 0.0, -5.0, "Takeoff"},
      {5.0, 0.0, -5.0, "Point A"},
      {5.0, 5.0, -5.0, "Point B"},
      {0.0, 5.0, -5.0, "Point C"},
      {0.0, 0.0, -5.0, "Return Home"}
    };
  }

  void loadWaypointsFromFile(const std::string& filename)
  {
    try {
      YAML::Node config = YAML::LoadFile(filename);
      if (!config["waypoints"]) {
        loadDefaultWaypoints();
        return;
      }
      waypoints_.clear();
      for (const auto& wp : config["waypoints"]) {
        Waypoint w;
        w.x = wp["x"].as<double>();
        w.y = wp["y"].as<double>();
        w.z = wp["z"].as<double>();
        w.name = wp["name"] ? wp["name"].as<std::string>() : "";
        waypoints_.push_back(w);
      }
    } catch (const std::exception& e) {
      loadDefaultWaypoints();
    }
  }

  void positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    current_position_[0] = msg->x;
    current_position_[1] = msg->y;
    current_position_[2] = msg->z;
    position_received_ = true;

    // --- Safety Check for Arm ---
    // Only start swinging arm if we are 1m above ground to avoid hitting floor
    if (!arm_active_ && std::abs(msg->z) > 1.0) {
        arm_active_ = true;
        arm_start_time_ = this->now();
        RCLCPP_INFO(get_logger(), "⚠ Altitude safe. Starting Arm Swing!");
        
        // Also start recording metrics now
        metrics_started_ = true;
    }

    // --- Waypoint Logic ---
    if (current_waypoint_idx_ < waypoints_.size()) {
      const auto& wp = waypoints_[current_waypoint_idx_];
      double distance = calculateDistance(current_position_[0], current_position_[1], 
                                         current_position_[2], wp.x, wp.y, wp.z);
      
      // Check for completion
      if (distance < waypoint_radius_ && !hovering_) {
        RCLCPP_INFO(get_logger(), "✓ Reached [%s] (Error: %.2fm)", wp.name.c_str(), distance);
        hovering_ = true;
        hover_start_time_ = this->now();
      }

      if (hovering_ && (this->now() - hover_start_time_).seconds() >= hover_time_) {
        hovering_ = false;
        current_waypoint_idx_++;
        if (current_waypoint_idx_ >= waypoints_.size()) {
          if (loop_trajectory_) {
            current_waypoint_idx_ = 0;
          } else {
            trajectory_complete_ = true;
            reportPerformance();
          }
        }
      }
    }
  }

  void controlLoop()
  {
    if (waypoints_.empty() || trajectory_complete_ || !position_received_) return;

    // 1. Get Current Drone Target
    const auto& current_wp = waypoints_[current_waypoint_idx_];

    // 2. Publish Drone Setpoint
    auto msg = geometry_msgs::msg::PointStamped();
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.point.x = current_wp.x;
    msg.point.y = current_wp.y;
    msg.point.z = current_wp.z;
    pub_setpoint_->publish(msg);

    // 3. Calculate Performance Metrics (Drone Position Only)
    if (metrics_started_ && !trajectory_complete_) {
        double err = calculateDistance(current_position_[0], current_position_[1], current_position_[2],
                                      current_wp.x, current_wp.y, current_wp.z);
        
        sum_sq_error_pos_ += err * err;
        if (err > max_error_pos_) max_error_pos_ = err;
        sample_count_++;
    }

    // 4. Update Arm Trajectory (Continuous Sine Wave)
    updateArmTrajectory();
  }

  void updateArmTrajectory()
  {
    std::vector<double> joints = {-1.57, 3.14, 0.0}; // Default safe pose

    if (arm_active_) {
        double t = (this->now() - arm_start_time_).seconds();
        
        // --- Oscillation Logic ---
        // Center: -1.57 (Straight Down)
        // Amplitude: 1.0 rad (~57 degrees) -> Swing from -0.57 to -2.57
        // Frequency: 0.5 Hz (2 seconds per cycle)
        double amplitude = 1.0; 
        double frequency = 0.5; 
        
        // J1: Sine Wave
        joints[0] = -1.57 + amplitude * std::sin(2.0 * M_PI * frequency * t);
        
        // J2: 0.0 (Fully Extended) - Maximizes disturbance torque
        joints[1] = 0.0; 
        
        // J3: 0.0 (Unused)
        joints[2] = 0.0;
    }

    std_msgs::msg::Float64MultiArray arm_msg;
    arm_msg.data = joints;
    pub_arm_command_->publish(arm_msg);
  }

  void reportPerformance()
  {
    double rmse = (sample_count_ > 0) ? std::sqrt(sum_sq_error_pos_ / (double)sample_count_) : 0.0;
    
    RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
    RCLCPP_INFO(get_logger(), "  PERFORMANCE REPORT (Drone Tracking)");
    RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
    RCLCPP_INFO(get_logger(), "  Total Samples: %d", sample_count_);
    RCLCPP_INFO(get_logger(), "  Max Pos Error: %.4f m", max_error_pos_);
    RCLCPP_INFO(get_logger(), "  RMSE Position: %.4f m", rmse);
    RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
  }

  double calculateDistance(double x1, double y1, double z1, 
                          double x2, double y2, double z2)
  {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) + std::pow(z2 - z1, 2));
  }

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_setpoint_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_arm_command_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_position_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Waypoint> waypoints_;
  size_t current_waypoint_idx_ = 0;
  double current_position_[3] = {0.0, 0.0, 0.0};
  
  bool position_received_ = false;
  bool hovering_ = false;
  bool trajectory_complete_ = false;
  bool arm_active_ = false;

  rclcpp::Time hover_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time arm_start_time_{0, 0, RCL_ROS_TIME};
  
  double waypoint_radius_, hover_time_;
  bool loop_trajectory_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPlanner>());
  rclcpp::shutdown();
  return 0;
}