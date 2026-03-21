#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

// ============================================================================
// Offboard Manager - Handles Mode Switching & Setpoints
// ============================================================================
class OffboardManager : public rclcpp::Node
{
public:
  OffboardManager() : Node("offboard_manager")
  {
    // Parameters
    this->declare_parameter("takeoff_altitude", -5.0);
    this->declare_parameter("offboard_setpoint_rate_hz", 20.0);
    this->declare_parameter("enable_il_control", false);
    enable_il_control_ = this->get_parameter("enable_il_control").as_bool();


    
    takeoff_altitude_ = this->get_parameter("takeoff_altitude").as_double();
    double setpoint_rate = this->get_parameter("offboard_setpoint_rate_hz").as_double();

    // QoS settings
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.durability_volatile();

    RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
    RCLCPP_INFO(get_logger(), "  Offboard Manager");
    RCLCPP_INFO(get_logger(), "  Mode: %s", enable_il_control_ ? "Il_Control (Torque)" : "PID (Rates)");
    RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");

    // Publishers to PX4
    pub_offboard_mode_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
    
    pub_vehicle_command_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);
    
    pub_trajectory_setpoint_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 10);
    
    // Publisher for internal communication
    pub_position_setpoint_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/control/position_setpoint", 10);

    // Subscribers from PX4
    sub_vehicle_status_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v2", qos,
      std::bind(&OffboardManager::vehicleStatusCallback, this, std::placeholders::_1));
    
    sub_vehicle_control_mode_ = create_subscription<px4_msgs::msg::VehicleControlMode>(
      "/fmu/out/vehicle_control_mode", qos,
      std::bind(&OffboardManager::vehicleControlModeCallback, this, std::placeholders::_1));

    // Timers
    timer_offboard_heartbeat_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / setpoint_rate)),
      std::bind(&OffboardManager::publishOffboardControlMode, this));

    timer_state_machine_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&OffboardManager::stateMachine, this));

    timer_diagnostic_ = create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&OffboardManager::diagnosticCallback, this));

    RCLCPP_INFO(get_logger(), "  Takeoff altitude: %.1f m", -takeoff_altitude_);
    RCLCPP_INFO(get_logger(), "  Waiting for PX4 vehicle status...");
    RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
  }

private:
  enum class State {
    INIT,
    SENDING_SETPOINTS,
    ENGAGING_OFFBOARD,
    ARMING,
    READY
  };

  void diagnosticCallback()
  {
    if (!vehicle_status_received_) {
      diagnostic_count_++;
      RCLCPP_WARN(get_logger(), "[%d] Waiting for PX4...", diagnostic_count_);
    } else {
      timer_diagnostic_->cancel();
    }
  }

  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    if (!vehicle_status_received_) {
      RCLCPP_INFO(get_logger(), "✓ Vehicle status received");
      vehicle_status_received_ = true;
    }
    
    is_armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
  }

  void vehicleControlModeCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
  {
    bool was_offboard = is_offboard_;
    is_offboard_ = msg->flag_control_offboard_enabled;
    
    if (!was_offboard && is_offboard_) {
      RCLCPP_INFO(get_logger(), " Offboard mode active");
    }
  }

  // void publishOffboardControlMode()
  // {
  //   // =====================================================================
  //   // Always publish OffboardControlMode at >2Hz (required heartbeat)
  //   // Keep position=true as safe placeholder
  //   // =====================================================================
  //   auto msg = px4_msgs::msg::OffboardControlMode();
  //   msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  //   msg.position = true;  
  //   msg.velocity = false;
  //   msg.acceleration = false;
  //   msg.attitude = false;
  //   msg.body_rate = true;
  //   msg.direct_actuator = false; 
    
  //   pub_offboard_mode_->publish(msg);
  // }


  void publishOffboardControlMode()
  {
    auto msg = px4_msgs::msg::OffboardControlMode();
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;

    if (enable_il_control_) {
      // Il_Control Mode: We send Torque + Thrust
      msg.attitude = false;
      msg.body_rate = false;
      msg.thrust_and_torque = true;
      msg.direct_actuator = false;   // <<< THIS is the correct field name
    } else {
      // PID mode: body rate offboard via /fmu/in/vehicle_rates_setpoint
      msg.attitude = false;
      msg.body_rate = true;
      msg.thrust_and_torque = false;
      msg.direct_actuator = false;
    }

    pub_offboard_mode_->publish(msg);
  }



  void publishTrajectorySetpoint(double x, double y, double z)
  {
    // =====================================================================
    // 1. Internal Setpoint (ALWAYS PUBLISH)
    // This goes to the Il_Control/PID node so it knows where to fly
    // =====================================================================
    if (state_ == State::READY) {
      // After armed: STOP publishing trajectory_setpoint
      if (!stop_logged_) {
        RCLCPP_INFO(get_logger(), "→ Stopping trajectory_setpoint");
        RCLCPP_INFO(get_logger(), "→ il_control now maintains offboard");
        stop_logged_ = true;
      }
      
      // Still publish to /control/position_setpoint for adaptive_indi
      auto pos_msg = geometry_msgs::msg::PointStamped();
      pos_msg.header.stamp = this->get_clock()->now();
      pos_msg.header.frame_id = "map";
      pos_msg.point.x = x;
      pos_msg.point.y = y;
      pos_msg.point.z = z;
      // pub_position_setpoint_->publish(pos_msg);
      
      return;  // Don't publish trajectory_setpoint to PX4
    }
    
    // During startup: publish to PX4
    auto msg = px4_msgs::msg::TrajectorySetpoint();
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
    msg.yaw = NAN;
    msg.velocity = {NAN, NAN, NAN};
    msg.acceleration = {NAN, NAN, NAN};
    msg.jerk = {NAN, NAN, NAN};
    
    pub_trajectory_setpoint_->publish(msg);
    
    // Also publish for internal use
    auto pos_msg = geometry_msgs::msg::PointStamped();
    pos_msg.header.stamp = this->get_clock()->now();
    pos_msg.header.frame_id = "map";
    pos_msg.point.x = x;
    pos_msg.point.y = y;
    pos_msg.point.z = z;
    
    // pub_position_setpoint_->publish(pos_msg);
  }

  void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0)
  {
    auto msg = px4_msgs::msg::VehicleCommand();
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    
    pub_vehicle_command_->publish(msg);
  }

  void stateMachine()
  {
    switch (state_) {
      case State::INIT:
        if (vehicle_status_received_) {
          RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
          RCLCPP_INFO(get_logger(), "  Starting flight sequence...");
          RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
          state_ = State::SENDING_SETPOINTS;
          setpoint_start_time_ = this->now();
        }
        break;

      case State::SENDING_SETPOINTS:
        // Publish setpoints for 2 seconds to satisfy PX4 requirement
        publishTrajectorySetpoint(0.0, 0.0, takeoff_altitude_);
        
        if ((this->now() - setpoint_start_time_).seconds() > 2.0) {
          RCLCPP_INFO(get_logger(), "✓ Setpoints sent for 2s");
          RCLCPP_INFO(get_logger(), "→ Engaging offboard mode");
          publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
          state_ = State::ENGAGING_OFFBOARD;
          offboard_start_time_ = this->now();
        }
        break;

      case State::ENGAGING_OFFBOARD:
        publishTrajectorySetpoint(0.0, 0.0, takeoff_altitude_);
        
        if (is_offboard_) {
          RCLCPP_INFO(get_logger(), "→ Arming vehicle");
          publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
          state_ = State::ARMING;
          arm_start_time_ = this->now();
        } else if ((this->now() - offboard_start_time_).seconds() > 5.0) {
          RCLCPP_WARN(get_logger(), "⚠ Offboard timeout, retrying");
          publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
          offboard_start_time_ = this->now();
        }
        break;

      case State::ARMING:
        publishTrajectorySetpoint(0.0, 0.0, takeoff_altitude_);
        
        if (is_armed_) {
          RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
          RCLCPP_INFO(get_logger(), "  ✓ ARMED - Ready for flight");
          RCLCPP_INFO(get_logger(), "  il_control_system in control");
          RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
          state_ = State::READY;
        } else if ((this->now() - arm_start_time_).seconds() > 5.0) {
          RCLCPP_WARN(get_logger(), "⚠ Arm timeout, retrying");
          publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
          arm_start_time_ = this->now();
        }
        break;

      case State::READY:
        // After armed: stop trajectory_setpoint to PX4
        // (but keep publishing to /control/position_setpoint for adaptive_indi)
        publishTrajectorySetpoint(0.0, 0.0, takeoff_altitude_);
        
        if (!is_armed_) {
          RCLCPP_INFO(get_logger(), "Vehicle disarmed");
          stop_logged_ = false;
          state_ = State::INIT;
        }
        break;
    }
  }

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_mode_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_vehicle_command_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_trajectory_setpoint_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_position_setpoint_;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_vehicle_status_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr sub_vehicle_control_mode_;

  rclcpp::TimerBase::SharedPtr timer_offboard_heartbeat_;
  rclcpp::TimerBase::SharedPtr timer_state_machine_;
  rclcpp::TimerBase::SharedPtr timer_diagnostic_;

  State state_ = State::INIT;
  bool vehicle_status_received_ = false;
  bool is_armed_ = false;
  bool is_offboard_ = false;
  bool stop_logged_ = false;
  bool enable_il_control_ = false;

  
  double takeoff_altitude_ = -5.0;
  int diagnostic_count_ = 0;
  
  rclcpp::Time setpoint_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time offboard_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time arm_start_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardManager>());
  rclcpp::shutdown();
  return 0;
}