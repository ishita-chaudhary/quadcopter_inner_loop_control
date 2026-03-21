#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <cmath>

class TrajectoryPlanner : public rclcpp::Node
{
public:
  TrajectoryPlanner() : Node("trajectory_planner")
  {
    // Parameters
    this->declare_parameter("waypoints_file", "");
    this->declare_parameter("waypoint_radius", 2.2);  // Acceptance radius in meters
    this->declare_parameter("hover_time", 2.0);       // Time to hover at each waypoint
    this->declare_parameter("loop_trajectory", false);
    
    waypoint_radius_ = this->get_parameter("waypoint_radius").as_double();
    hover_time_ = this->get_parameter("hover_time").as_double();
    loop_trajectory_ = this->get_parameter("loop_trajectory").as_bool();
    
    std::string waypoints_file = this->get_parameter("waypoints_file").as_string();

    // QoS for PX4
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.transient_local();

    // Publisher
    pub_setpoint_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/control/position_setpoint", 10);

    // Subscriber
    sub_position_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos,
      std::bind(&TrajectoryPlanner::positionCallback, this, std::placeholders::_1));

    // Timer for setpoint publishing
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),  // 10 Hz
      std::bind(&TrajectoryPlanner::publishSetpoint, this));

    // Load waypoints
    if (waypoints_file.empty()) {
      loadDefaultWaypoints();
    } else {
      loadWaypointsFromFile(waypoints_file);
    }

    printWaypoints();
    
    RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
    RCLCPP_INFO(get_logger(), "  Trajectory Planner Started");
    RCLCPP_INFO(get_logger(), "  Waypoints: %zu", waypoints_.size());
    RCLCPP_INFO(get_logger(), "  Acceptance radius: %.2f m", waypoint_radius_);
    RCLCPP_INFO(get_logger(), "  Loop: %s", loop_trajectory_ ? "Yes" : "No");
    RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
  }

private:
  struct Waypoint {
    double x, y, z;
    std::string name;
  };

  void loadDefaultWaypoints()
  {
    RCLCPP_INFO(get_logger(), "Loading default trajectory...");
    
    // Example: Square pattern at 5m altitude
    waypoints_ = {
      {0.0, 0.0, -5.0, "Takeoff"},
      {5.0, 0.0, -5.0, "Point 1"},
      {5.0, 5.0, -5.0, "Point 2"},
      {0.0, 5.0, -5.0, "Point 3"},
      {0.0, 0.0, -5.0, "Return Home"}
    };
  }

  void loadWaypointsFromFile(const std::string& filename)
  {
    try {
      RCLCPP_INFO(get_logger(), "Loading waypoints from: %s", filename.c_str());
      YAML::Node config = YAML::LoadFile(filename);
      
      if (!config["waypoints"]) {
        RCLCPP_ERROR(get_logger(), "No 'waypoints' key in YAML file!");
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

      if (waypoints_.empty()) {
        RCLCPP_ERROR(get_logger(), "No waypoints loaded!");
        loadDefaultWaypoints();
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to load waypoints: %s", e.what());
      loadDefaultWaypoints();
    }
  }

  void printWaypoints()
  {
    RCLCPP_INFO(get_logger(), "Trajectory waypoints:");
    for (size_t i = 0; i < waypoints_.size(); i++) {
      const auto& wp = waypoints_[i];
      RCLCPP_INFO(get_logger(), "  [%zu] %-15s → (%.2f, %.2f, %.2f)", 
                  i, wp.name.c_str(), wp.x, wp.y, wp.z);
    }
  }

  void positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    current_position_[0] = msg->x;
    current_position_[1] = msg->y;
    current_position_[2] = msg->z;
    position_received_ = true;

    // Check if reached current waypoint
    if (current_waypoint_idx_ < waypoints_.size()) {
      const auto& wp = waypoints_[current_waypoint_idx_];
      double distance = calculateDistance(current_position_[0], current_position_[1], 
                                         current_position_[2], wp.x, wp.y, wp.z);
      
      if (distance < waypoint_radius_ && !hovering_) {
        // Reached waypoint
        RCLCPP_INFO(get_logger(), "✓ Reached waypoint [%zu]: %s (dist: %.2fm)", 
                    current_waypoint_idx_, wp.name.c_str(), distance);
        
        hovering_ = true;
        hover_start_time_ = this->now();
      }

      // Check if hover time completed
      if (hovering_ && (this->now() - hover_start_time_).seconds() >= hover_time_) {
        advanceToNextWaypoint();
      }
    }
  }

  void advanceToNextWaypoint()
  {
    hovering_ = false;
    current_waypoint_idx_++;

    if (current_waypoint_idx_ >= waypoints_.size()) {
      if (loop_trajectory_) {
        RCLCPP_INFO(get_logger(), "↻ Looping back to start");
        current_waypoint_idx_ = 0;
      } else {
        RCLCPP_INFO(get_logger(), "✓ Trajectory complete!");
        trajectory_complete_ = true;
      }
    } else {
      const auto& next_wp = waypoints_[current_waypoint_idx_];
      RCLCPP_INFO(get_logger(), "→ Next waypoint [%zu]: %s", 
                  current_waypoint_idx_, next_wp.name.c_str());
    }
  }

  void publishSetpoint()
  {
    if (waypoints_.empty() || trajectory_complete_) {
      return;
    }

    const auto& current_wp = waypoints_[current_waypoint_idx_];
    
    auto msg = geometry_msgs::msg::PointStamped();
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.point.x = current_wp.x;
    msg.point.y = current_wp.y;
    msg.point.z = current_wp.z;
    
    pub_setpoint_->publish(msg);

    // Debug logging
    if (position_received_ && (log_counter_++ % 50 == 0)) {  // Every 5 seconds
      double dist = calculateDistance(current_position_[0], current_position_[1], 
                                      current_position_[2], 
                                      current_wp.x, current_wp.y, current_wp.z);
      RCLCPP_INFO(get_logger(), 
                  "[%zu/%zu] %s → Dist: %.2fm | Pos: [%.2f, %.2f, %.2f]",
                  current_waypoint_idx_ + 1, waypoints_.size(),
                  current_wp.name.c_str(), dist,
                  current_position_[0], current_position_[1], current_position_[2]);
    }
  }

  double calculateDistance(double x1, double y1, double z1, 
                          double x2, double y2, double z2)
  {
    return std::sqrt(std::pow(x2 - x1, 2) + 
                    std::pow(y2 - y1, 2) + 
                    std::pow(z2 - z1, 2));
  }

  // Publishers & Subscribers
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_setpoint_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_position_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Waypoints
  std::vector<Waypoint> waypoints_;
  size_t current_waypoint_idx_ = 0;
  
  // State
  double current_position_[3] = {0.0, 0.0, 0.0};
  bool position_received_ = false;
  bool hovering_ = false;
  bool trajectory_complete_ = false;
  
  rclcpp::Time hover_start_time_{0, 0, RCL_ROS_TIME};
  
  // Parameters
  double waypoint_radius_;
  double hover_time_;
  bool loop_trajectory_;
  
  int log_counter_ = 0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPlanner>());
  rclcpp::shutdown();
  return 0;
}








// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/point_stamped.hpp>
// #include <px4_msgs/msg/vehicle_local_position.hpp>
// #include <yaml-cpp/yaml.h>
// #include <vector>
// #include <cmath>

// class TrajectoryPlanner : public rclcpp::Node
// {
// public:
//   TrajectoryPlanner() : Node("trajectory_planner")
//   {
//     // 1. Declare and get parameters
//     this->declare_parameter("waypoints_file", "");
//     this->declare_parameter("waypoint_radius", 0.3);
//     this->declare_parameter("hover_time", 2.0);
//     this->declare_parameter("loop_trajectory", false);
//     this->declare_parameter("hold_at_setpoint", false);
//     this->declare_parameter("hold_x", 0.0);
//     this->declare_parameter("hold_y", 0.0);
//     this->declare_parameter("hold_z", -5.0);
    
//     updateParameters();

//     // 2. QoS for PX4
//     auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
//     qos.best_effort();
//     qos.transient_local();

//     // 3. Publishers & Subscribers
//     pub_setpoint_ = create_publisher<geometry_msgs::msg::PointStamped>("/control/position_setpoint", 10);
    
//     sub_position_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
//       "/fmu/out/vehicle_local_position_v1", qos,
//       std::bind(&TrajectoryPlanner::positionCallback, this, std::placeholders::_1));

//     // 4. Timer (10Hz)
//     timer_ = create_wall_timer(
//       std::chrono::milliseconds(100),
//       std::bind(&TrajectoryPlanner::publishSetpoint, this));

//     // 5. Load Trajectory
//     std::string waypoints_file = this->get_parameter("waypoints_file").as_string();
//     if (waypoints_file.empty()) {
//       loadDefaultWaypoints();
//     } else {
//       loadWaypointsFromFile(waypoints_file);
//     }

//     RCLCPP_INFO(get_logger(), "Planner Started. Hold Mode: %s", hold_at_setpoint_ ? "ON" : "OFF");
//   }

// private:
//   struct Waypoint {
//     double x, y, z;
//     std::string name;
//   };

//   // --- UPDATED: HELPER FUNCTIONS WITH FULL LOGIC ---

//   void updateParameters() {
//     waypoint_radius_ = this->get_parameter("waypoint_radius").as_double();
//     hover_time_ = this->get_parameter("hover_time").as_double();
//     loop_trajectory_ = this->get_parameter("loop_trajectory").as_bool();
//     hold_at_setpoint_ = this->get_parameter("hold_at_setpoint").as_bool();
//     hold_pos_[0] = this->get_parameter("hold_x").as_double();
//     hold_pos_[1] = this->get_parameter("hold_y").as_double();
//     hold_pos_[2] = this->get_parameter("hold_z").as_double();
//   }

//   void loadDefaultWaypoints()
//   {
//     RCLCPP_INFO(get_logger(), "Loading default trajectory...");
//     waypoints_ = {
//       {0.0, 0.0, -5.0, "Takeoff"},
//       {5.0, 0.0, -5.0, "Point 1"},
//       {5.0, 5.0, -5.0, "Point 2"},
//       {0.0, 5.0, -5.0, "Point 3"},
//       {0.0, 0.0, -5.0, "Return Home"}
//     };
//   }

//   void loadWaypointsFromFile(const std::string& filename)
//   {
//     try {
//       RCLCPP_INFO(get_logger(), "Loading waypoints from: %s", filename.c_str());
//       YAML::Node config = YAML::LoadFile(filename);
      
//       if (!config["waypoints"]) {
//         RCLCPP_ERROR(get_logger(), "No 'waypoints' key in YAML!");
//         loadDefaultWaypoints();
//         return;
//       }

//       waypoints_.clear();
//       for (const auto& wp : config["waypoints"]) {
//         Waypoint w;
//         w.x = wp["x"].as<double>();
//         w.y = wp["y"].as<double>();
//         w.z = wp["z"].as<double>();
//         w.name = wp["name"] ? wp["name"].as<std::string>() : "WP";
//         waypoints_.push_back(w);
//       }
//     } catch (const std::exception& e) {
//       RCLCPP_ERROR(get_logger(), "File Error: %s. Using defaults.", e.what());
//       loadDefaultWaypoints();
//     }
//   }

//   void positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
//   {
//     current_position_[0] = msg->x;
//     current_position_[1] = msg->y;
//     current_position_[2] = msg->z;
//     position_received_ = true;

//     if (hold_at_setpoint_) return; 

//     if (current_waypoint_idx_ < waypoints_.size()) {
//       const auto& wp = waypoints_[current_waypoint_idx_];
//       double distance = calculateDistance(current_position_[0], current_position_[1], 
//                                          current_position_[2], wp.x, wp.y, wp.z);
      
//       if (distance < waypoint_radius_ && !hovering_) {
//         RCLCPP_INFO(get_logger(), "✓ Reached [%zu]: %s", current_waypoint_idx_, wp.name.c_str());
//         hovering_ = true;
//         hover_start_time_ = this->now();
//       }

//       if (hovering_ && (this->now() - hover_start_time_).seconds() >= hover_time_) {
//         advanceToNextWaypoint();
//       }
//     }
//   }

//   void advanceToNextWaypoint()
//   {
//     hovering_ = false;
//     current_waypoint_idx_++;

//     if (current_waypoint_idx_ >= waypoints_.size()) {
//       if (loop_trajectory_) {
//         current_waypoint_idx_ = 0;
//         RCLCPP_INFO(get_logger(), "↻ Looping Trajectory");
//       } else {
//         trajectory_complete_ = true;
//         RCLCPP_INFO(get_logger(), "🏁 Trajectory Complete");
//       }
//     }
//   }

//   void publishSetpoint()
//   {
//     // Refresh the flag live from parameters
//     this->get_parameter("hold_at_setpoint", hold_at_setpoint_);

//     auto msg = geometry_msgs::msg::PointStamped();
//     msg.header.stamp = this->now();
//     msg.header.frame_id = "map";

//     if (hold_at_setpoint_) {
//       msg.point.x = hold_pos_[0];
//       msg.point.y = hold_pos_[1];
//       msg.point.z = hold_pos_[2];
//     } else {
//       if (waypoints_.empty() || trajectory_complete_) return;
//       const auto& current_wp = waypoints_[current_waypoint_idx_];
//       msg.point.x = current_wp.x;
//       msg.point.y = current_wp.y;
//       msg.point.z = current_wp.z;
//     }
    
//     pub_setpoint_->publish(msg);
//   }

//   double calculateDistance(double x1, double y1, double z1, double x2, double y2, double z2)
//   {
//     return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) + std::pow(z2 - z1, 2));
//   }

//   // Member Variables
//   rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_setpoint_;
//   rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_position_;
//   rclcpp::TimerBase::SharedPtr timer_;

//   std::vector<Waypoint> waypoints_;
//   size_t current_waypoint_idx_ = 0;
//   double current_position_[3] = {0.0, 0.0, 0.0};
//   double hold_pos_[3] = {0.0, 0.0, 0.0};
  
//   bool position_received_ = false;
//   bool hovering_ = false;
//   bool trajectory_complete_ = false;
//   bool hold_at_setpoint_ = false;
  
//   rclcpp::Time hover_start_time_{0, 0, RCL_ROS_TIME};
//   double waypoint_radius_;
//   double hover_time_;
//   bool loop_trajectory_;
// };

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<TrajectoryPlanner>());
//   rclcpp::shutdown();
//   return 0;
// }