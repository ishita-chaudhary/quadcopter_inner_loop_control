#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include "asmc_system/pid_controllers_clean.hpp"
#include "asmc_system/asmc.hpp"

using namespace asmc_system;

class ASMCSystem : public rclcpp::Node
{
public:
  ASMCSystem() : Node("asmc_system")
  {
    using std::placeholders::_1;

    this->declare_parameter("enable_asmc",       false);
    enable_asmc_       = this->get_parameter("enable_asmc").as_bool();
    this->declare_parameter("bypass_pid", false);
    bypass_pid_ = this->get_parameter("bypass_pid").as_bool();

    // PID gains
    // this->declare_parameter("position_kp_xy",   0.6);
    // this->declare_parameter("position_kp_z",    1.2);
    // this->declare_parameter("velocity_kp_xy",   0.5);
    // this->declare_parameter("velocity_ki_xy",   0.08);
    // this->declare_parameter("velocity_kd_xy",   0.15);
    // this->declare_parameter("velocity_kp_z",    0.8);
    // this->declare_parameter("velocity_ki_z",    0.10);
    // this->declare_parameter("velocity_kd_z",    0.12);
    // this->declare_parameter("attitude_kp_roll",  3.0);
    // this->declare_parameter("attitude_ki_roll",  0.5);
    // this->declare_parameter("attitude_kd_roll",  0.15);
    // this->declare_parameter("attitude_kp_pitch", 3.0);
    // this->declare_parameter("attitude_ki_pitch", 0.5);
    // this->declare_parameter("attitude_kd_pitch", 0.15);
    // this->declare_parameter("attitude_kp_yaw",   2.0);
    // this->declare_parameter("attitude_ki_yaw",   0.3);
    // this->declare_parameter("attitude_kd_yaw",   0.10);


    this->declare_parameter("position_kp_xy",   0.8);
    this->declare_parameter("position_kp_z",    1.2);
    this->declare_parameter("position_ki_xy",   0.15);
    this->declare_parameter("position_ki_z",    0.2);
    this->declare_parameter("velocity_kp_xy",   0.6);
    this->declare_parameter("velocity_ki_xy",   0.08);
    this->declare_parameter("velocity_kd_xy",   0.15);
    this->declare_parameter("velocity_kp_z",    0.8);
    this->declare_parameter("velocity_ki_z",    0.10);
    this->declare_parameter("velocity_kd_z",    0.12);
    this->declare_parameter("attitude_kp_roll",  4.0);
    this->declare_parameter("attitude_ki_roll",  0.5);
    this->declare_parameter("attitude_kd_roll",  0.15);
    this->declare_parameter("attitude_kp_pitch", 4.0);
    this->declare_parameter("attitude_ki_pitch", 0.5);
    this->declare_parameter("attitude_kd_pitch", 0.15);
    this->declare_parameter("attitude_kp_yaw",   2.0);
    this->declare_parameter("attitude_ki_yaw",   0.3);
    this->declare_parameter("attitude_kd_yaw",   0.10);


    pos_controller_        = std::make_unique<PositionController>();
    // vel_att_controller_    = std::make_unique<VelocityToAttitudeController>();
    vel_accel_controller_  = std::make_unique<VelocityToAccelController>(); // Changed!
    accel_att_controller_  = std::make_unique<AccelToAttitudeController>(); // New!
    att_rate_controller_   = std::make_unique<AttitudeToRateController>();

    pos_controller_->setGains(
      this->get_parameter("position_kp_xy").as_double(),
      this->get_parameter("position_kp_z").as_double(),
      this->get_parameter("position_ki_xy").as_double(),
      this->get_parameter("position_ki_z").as_double());

    // vel_att_controller_->setVelocityGain(
    vel_accel_controller_->setVelocityGain(
      this->get_parameter("velocity_kp_xy").as_double(),
      this->get_parameter("velocity_ki_xy").as_double(),
      this->get_parameter("velocity_kd_xy").as_double(),
      this->get_parameter("velocity_kp_z").as_double(),
      this->get_parameter("velocity_ki_z").as_double(),
      this->get_parameter("velocity_kd_z").as_double());

    att_rate_controller_->setGains(
      this->get_parameter("attitude_kp_roll").as_double(),
      this->get_parameter("attitude_ki_roll").as_double(),
      this->get_parameter("attitude_kd_roll").as_double(),
      this->get_parameter("attitude_kp_pitch").as_double(),
      this->get_parameter("attitude_ki_pitch").as_double(),
      this->get_parameter("attitude_kd_pitch").as_double(),
      this->get_parameter("attitude_kp_yaw").as_double(),
      this->get_parameter("attitude_ki_yaw").as_double(),
      this->get_parameter("attitude_kd_yaw").as_double());

    // Initialize ASMC
    asmc_ = std::make_unique<ASMC>();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.durability_volatile();

    sub_pos_setpoint_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/control/position_setpoint", 10,
      std::bind(&ASMCSystem::positionSetpointCb, this, _1));
    sub_position_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos,
      std::bind(&ASMCSystem::positionCb, this, _1));
    sub_attitude_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos,
      std::bind(&ASMCSystem::attitudeCb, this, _1));
    sub_ang_vel_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", qos,
      std::bind(&ASMCSystem::angularVelocityCb, this, _1));
    sub_direct_cmd_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/control/rl_actions", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 7) {
            direct_accel_cmd_ << msg->data[0], msg->data[1], msg->data[2];
            std::cout << "Acc Command recieved: " << direct_accel_cmd_ << std::endl;
            direct_rate_cmd_  << msg->data[3], msg->data[4], msg->data[5];
            std::cout << "Body rate Command recieved: " << direct_rate_cmd_ << std::endl;
            direct_yaw_cmd_   = msg->data[6];
            std::cout << "Yaw Command recieved: " << direct_yaw_cmd_ << std::endl;
            direct_setpoint_received_ = true;
        }
      });

    pub_rates_setpoint_ = create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
      "/fmu/in/vehicle_rates_setpoint", 10);
    pub_thrust_ = create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
      "/fmu/in/vehicle_thrust_setpoint", 10);
    pub_torque_ = create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
      "/fmu/in/vehicle_torque_setpoint", 10);

    timer_control_loop_ = create_wall_timer(
      std::chrono::milliseconds(4),
      std::bind(&ASMCSystem::controlLoopCallback, this));

    RCLCPP_INFO(get_logger(), "ASMC System Initialized");

    // vel_out.attitude_cmd = Eigen::Vector3f::Zero();
    // vel_out.thrust_cmd = 0.0f; // or hover thrust like 0.7
  }

private:

  // ===========================================================================
  // Sensor callbacks — your working PID code, unchanged
  // ===========================================================================

  void positionSetpointCb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    pos_desired_ << msg->point.x, msg->point.y, msg->point.z;
    std::cout << "pos_desired: " << pos_desired_ << std::endl;
    setpoint_received_ = true;
  }

  void positionCb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    pos_current_ << msg->x, msg->y, msg->z;
    std::cout << "pos_current: " << pos_current_ << std::endl;
    vel_current_ << msg->vx, msg->vy, msg->vz;
    std::cout << "vel current: " << vel_current_ << std::endl;

    position_valid_ = true;
  }

  void attitudeCb(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    // Store Quaternion for ASMC
    q_current_ = Eigen::Quaternionf(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);

    float qw=msg->q[0], qx=msg->q[1], qy=msg->q[2], qz=msg->q[3];
    roll_current_  = std::atan2(2.0f*(qw*qx+qy*qz), 1.0f-2.0f*(qx*qx+qy*qy));
    float sp       = 2.0f*(qw*qy-qz*qx);
    pitch_current_ = (std::abs(sp)>=1.0f) ? std::copysign(M_PI/2.0f,sp) : std::asin(sp);
    yaw_current_ = std::atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
    attitude_valid_ = true;
  }

  void angularVelocityCb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    ang_vel_raw_ << msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2];
    // ang_vel_raw_ << msg->angular_velocity[1], msg->angular_velocity[0], msg->angular_velocity[2];
    // ang_vel_raw_[2] = - ang_vel_raw_[2];
    ang_vel_current_ = ang_vel_raw_;
    ang_vel_valid_   = true;
  }

  // ===========================================================================
  // Control loop
  // ===========================================================================
  // void controlLoopCallback()
  // {
  //   if (!position_valid_ || !ang_vel_valid_ || !attitude_valid_) {
  //   // if (!position_valid_ || !attitude_valid_ || !setpoint_received_) {

  //     RCLCPP_INFO(get_logger(), "NOT RECIEVED:");
  //     if (!position_valid_) {
  //       RCLCPP_INFO(get_logger(), "Current Position, Lin Vel");
  //     }
  //     if (!ang_vel_valid_) {
  //       RCLCPP_INFO(get_logger(), "Current Ang Vel");
  //     }
  //     if (!attitude_valid_) {
  //       RCLCPP_INFO(get_logger(), "Current Attitude");
  //     }
  //     return;
  //   }
  //   if (!bypass_pid_ && !setpoint_received_) return;
  //   if (bypass_pid_ && !direct_setpoint_received_) return;

  //   auto  now = this->now();
  //   float dt  = (now - control_loop_prev_time_).seconds();
  //   if (dt <= 0.0f || dt > 0.1f) { control_loop_prev_time_ = now; return; }
  //   control_loop_prev_time_ = now;

  //   Eigen::Vector3f rate_cmd;
  //   float thrust_cmd;
  //   Eigen::Quaternionf q_desired;

  //   Eigen::Vector3f accel_cmd;
  //   float reference_yaw;

  //   // ── Triple-loop PID cascade (unchanged from working PID code) ────────────
  //   if (!bypass_pid_) {
  //     if (loop_counter_ % 5 == 0) {
  //       Eigen::Vector3f vel_cmd    = pos_controller_->update(pos_desired_, pos_current_);
  //       vel_out    = vel_att_controller_->update(vel_cmd, vel_current_, yaw_current_);
  //     }
  //     Eigen::Vector3f att_cmd    = vel_out.attitude_cmd;
  //     float           thrust_cmd = vel_out.thrust_cmd;  // normalized [0,1] collective thrust

  //     Eigen::Vector3f att_cur;
  //     att_cur << roll_current_, pitch_current_, 0.0f;
  //     Eigen::Vector3f rate_cmd = att_rate_controller_->update(att_cmd, att_cur);
  //   }
  //   else {
  //     rate_cmd = direct_rate_cmd_;
  //     thrust_cmd = direct_thrust_cmd_;
  //   }

  //   // ── Mode 1: PID-ONLY ─────────────────────────────────────────────────────
  //   if (!enable_asmc_) {
  //     publishRatesSetpoint(rate_cmd, thrust_cmd);
  //     last_thrust_cmd_ = thrust_cmd;
  //     control_loop_count_++;
  //     reset_q_desired_ = true; 
  //     return;
  //   }

  //   if (reset_q_desired_) {
  //     q_desired_state_ = q_current_; // Align with current orientation to prevent instant flipping
  //     reset_q_desired_ = false;
  //   }

  //   // ── Mode 2: PID-THRUST + ASMC TORQUE ──────────────────────────────────
  //   // Prepare Inputs for ASMC
  //   // a. Convert Attitude Command to Quaternion (q_desired)

  //   // Eigen::Quaternionf q_desired;
  //   // float desired_yaw = yaw_current_;

  //   // Euler to Quaternion (Roll, Pitch, Yaw=0 as per PID)
  //   // R = Rz(yaw) * Ry(pitch) * Rx(roll)

  //   // q_desired = Eigen::AngleAxisf(desired_yaw, Eigen::Vector3f::UnitZ())
  //   //           * Eigen::AngleAxisf(att_cmd(1), Eigen::Vector3f::UnitY())
  //   //           * Eigen::AngleAxisf(att_cmd(0), Eigen::Vector3f::UnitX());
  //   // q_desired.normalize();

  //   //a. for integrated att cmd
  //   float omega_norm = rate_cmd.norm();
  //   Eigen::Quaternionf delta_q;

  //   if (omega_norm > 1e-6f) {
  //       // Calculate rotation axis and angle
  //       Eigen::Vector3f axis = rate_cmd / omega_norm;
  //       float theta = omega_norm * dt;
        
  //       // Eigen AngleAxis maps perfectly to a delta quaternion
  //       delta_q = Eigen::AngleAxisf(theta, axis); 
  //   } else {
  //       // No rotation commanded
  //       delta_q = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
  //   }

  //   // Multiply to integrate: q_new = q_old * delta_q
  //   q_desired_state_ = q_desired_state_ * delta_q;
  //   q_desired_state_.normalize();
    
  //   // b. Desired Angular Acceleration
  //   // PIDs don't give this. We assume 0 for regulation or we could differentiate rate_cmd.
  //   static Eigen::Vector3f rate_cmd_prev = Eigen::Vector3f::Zero();
  //   static bool first_run = true;
    
  //   Eigen::Vector3f alpha_des;
  //   if (first_run) {
  //       alpha_des = Eigen::Vector3f::Zero();
  //       first_run = false;
  //   } else {
  //       // Simple finite difference: (current - prev) / dt
  //       alpha_des = (rate_cmd - rate_cmd_prev) / dt;
  //   }
  //   rate_cmd_prev = rate_cmd; // Update for next loop

  //   RCLCPP_INFO(get_logger(), "Calling ASMC Controller");
  //   std::cout << "rates: " << rate_cmd << std::endl;

  //   // c. ASMC Update -> Torque Command
  //   Eigen::Vector3f torque_cmd = asmc_->update(
  //       q_current_, 
  //       ang_vel_current_, 
  //       q_desired_state_, 
  //       rate_cmd,  // Use rate PID output as omega_desired
  //       alpha_des, 
  //       dt
  //   );

  //   // 5. Publish
  //   publishControl(thrust_cmd, torque_cmd);
  //   last_thrust_cmd_ = thrust_cmd;
  //   control_loop_count_++;
  //   loop_counter_++;

  // }

  void controlLoopCallback()
  {
    if (!position_valid_ || !ang_vel_valid_ || !attitude_valid_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for valid sensors...");
      return;
    }
    if (!bypass_pid_ && !setpoint_received_) return;
    if (bypass_pid_ && !direct_setpoint_received_) return;

    auto  now = this->now();
    float dt  = (now - control_loop_prev_time_).seconds();
    if (dt <= 0.0f || dt > 0.1f) { control_loop_prev_time_ = now; return; }
    control_loop_prev_time_ = now;

    Eigen::Vector3f rate_cmd;
    Eigen::Vector3f accel_cmd;
    float reference_yaw;

    // ── 1. Determine Acceleration and Rates based on Mode ────────────────────
    if (!bypass_pid_) {
      reference_yaw = yaw_current_; // PID maintains current yaw (or setpoint if you have one)
      if (loop_counter_ % 5 == 0) {
        Eigen::Vector3f vel_cmd = pos_controller_->update(pos_desired_, pos_current_);
        accel_cmd = vel_accel_controller_->update(vel_cmd, vel_current_);
      }
    } 
    else {
      // RL Direct Command Mode
      accel_cmd = direct_accel_cmd_;  
      rate_cmd = direct_rate_cmd_;
      
      // We map world accel to body frame using the drone's current yaw
      // reference_yaw = yaw_current_; 
      reference_yaw = direct_yaw_cmd_; 

    }

    // ── 2. Convert Acceleration -> Attitude & Thrust (Used by BOTH modes!) ───
    // This automatically turns accel_cmd(2) into the correct normalized thrust_cmd!
    if (!bypass_pid_ && loop_counter_ % 5 == 0) {
        att_out = accel_att_controller_->update(accel_cmd, reference_yaw);
    } else if (bypass_pid_) {
        att_out = accel_att_controller_->update(accel_cmd, reference_yaw);
    }

    Eigen::Vector3f att_cmd = att_out.attitude_cmd; 
    float thrust_cmd = att_out.thrust_cmd;

    // In RL mode, overwrite the attitude yaw with the RL's desired yaw
    if (bypass_pid_) {
        att_cmd(2) = direct_yaw_cmd_;
    } else if (!bypass_pid_) {
        att_cmd(2) = yaw_current_;
        Eigen::Vector3f att_cur;
        att_cur << roll_current_, pitch_current_, 0.0f;
        rate_cmd = att_rate_controller_->update(att_cmd, att_cur);
    }

    // ── Mode 1: PID-ONLY ─────────────────────────────────────────────────────
    if (!enable_asmc_) {
      Eigen::Vector3f att_cur;
      att_cur << roll_current_, pitch_current_, 0.0f;
      
      // If bypass_pid_ is false, we need to calculate rate_cmd from the PID
      if (!bypass_pid_) {
          rate_cmd = att_rate_controller_->update(att_cmd, att_cur);
      }

      publishRatesSetpoint(rate_cmd, thrust_cmd);
      last_thrust_cmd_ = thrust_cmd;
      control_loop_count_++;
      return;
    }

    // ── Mode 2: ASMC TORQUE ──────────────────────────────────────────────────
    
    // Convert Attitude Command to Quaternion (q_desired)
    Eigen::Quaternionf q_desired;
    q_desired = Eigen::AngleAxisf(att_cmd(2), Eigen::Vector3f::UnitZ())
              * Eigen::AngleAxisf(att_cmd(1), Eigen::Vector3f::UnitY())
              * Eigen::AngleAxisf(att_cmd(0), Eigen::Vector3f::UnitX());
    q_desired.normalize();

    // Desired Angular Acceleration (alpha_des) via finite difference
    static Eigen::Vector3f rate_cmd_prev = Eigen::Vector3f::Zero();
    static bool first_run = true;
    
    Eigen::Vector3f alpha_des;
    if (first_run) {
        alpha_des = Eigen::Vector3f::Zero();
        first_run = false;
    } else {
        alpha_des = (rate_cmd - rate_cmd_prev) / dt;
    }
    rate_cmd_prev = rate_cmd; // Update for next loop

    // ASMC Update -> Torque Command
    Eigen::Vector3f torque_cmd = asmc_->update(
        q_current_, 
        ang_vel_current_, 
        q_desired,   // Fixed typo (was q_desired_)
        rate_cmd,  
        alpha_des, 
        dt
    );

    // Publish
    publishControl(thrust_cmd, torque_cmd);
    last_thrust_cmd_ = thrust_cmd;
    control_loop_count_++;
    loop_counter_++;
  }

  // ===========================================================================
  // Output helpers
  // ===========================================================================

  void publishRatesSetpoint(const Eigen::Vector3f& rates, float thrust)
  {
    auto msg = px4_msgs::msg::VehicleRatesSetpoint();
    msg.timestamp      = this->get_clock()->now().nanoseconds() / 1000;
    msg.roll           = rates(0);
    msg.pitch          = rates(1);
    msg.yaw            = rates(2);
    msg.thrust_body[0] = 0.0f;
    msg.thrust_body[1] = 0.0f;
    msg.thrust_body[2] = -thrust;
    pub_rates_setpoint_->publish(msg);
  }

  void publishControl(float thrust, const Eigen::Vector3f& torque)
  {
    // Publish Thrust
    px4_msgs::msg::VehicleThrustSetpoint thrust_msg;
    thrust_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    thrust_msg.xyz[0] = 0.0f;
    thrust_msg.xyz[1] = 0.0f;
    thrust_msg.xyz[2] = -thrust; // PX4 usually expects negative Z for upward thrust in NED frame
    // std::cout << "publishing thrust: " << thrust_msg.xyz[2] << std::endl;
    pub_thrust_->publish(thrust_msg);

    // Publish Torque
    px4_msgs::msg::VehicleTorqueSetpoint torque_msg;
    torque_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    torque_msg.xyz[0] = torque(0);
    torque_msg.xyz[1] = torque(1);
    torque_msg.xyz[2] = torque(2);
    // std::cout << "publishing torque: " << torque_msg.xyz[0] << "," << torque_msg.xyz[1]<< "," << torque_msg.xyz[2] << "\n" << std::endl;
    pub_torque_->publish(torque_msg);
  }


  // ===========================================================================
  // Members
  // ===========================================================================

  bool  enable_asmc_;
  bool bypass_pid_ = false;

  // PID controllers
  std::unique_ptr<PositionController>           pos_controller_;
  // std::unique_ptr<VelocityToAttitudeController> vel_att_controller_;
  std::unique_ptr<VelocityToAccelController>    vel_accel_controller_; // Changed!
  std::unique_ptr<AccelToAttitudeController>    accel_att_controller_; // New!
  std::unique_ptr<AttitudeToRateController>     att_rate_controller_;

  // SMC controller
   std::unique_ptr<ASMC>                         asmc_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr      sub_pos_setpoint_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr   sub_position_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr        sub_attitude_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_ang_vel_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_direct_cmd_;

  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr pub_rates_setpoint_;
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr pub_thrust_;
  rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr pub_torque_;
 
  rclcpp::TimerBase::SharedPtr timer_control_loop_;

  // Raw sensor state
  Eigen::Vector3f pos_desired_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f pos_current_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f vel_current_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f ang_vel_raw_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f ang_vel_current_{Eigen::Vector3f::Zero()};
  Eigen::Quaternionf q_current_{1, 0, 0, 0};

  Eigen::Quaternionf q_desired_state_{1.0f, 0.0f, 0.0f, 0.0f};
  bool reset_q_desired_ = true; 

  float roll_current_  = 0.0f;
  float pitch_current_ = 0.0f;
  float yaw_current_ = 0.0f;

  bool setpoint_received_ = false;
  bool position_valid_    = false;
  bool attitude_valid_    = false;
  bool ang_vel_valid_     = false;

  
  bool direct_setpoint_received_ = false;
  Eigen::Vector3f direct_rate_cmd_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f direct_accel_cmd_{Eigen::Vector3f::Zero()};
  // float direct_thrust_cmd_ = 0.0f;
  float direct_yaw_cmd_ = 0.0f;

  rclcpp::Time control_loop_prev_time_{0, 0, RCL_ROS_TIME};
  int control_loop_count_ = 0;
  int loop_counter_ = 0;
  float last_thrust_cmd_ = 0.0f;

  // VelocityToAttitudeController::Output vel_out;
  AccelToAttitudeController::Output att_out;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ASMCSystem>());
  rclcpp::shutdown();
  return 0;
}