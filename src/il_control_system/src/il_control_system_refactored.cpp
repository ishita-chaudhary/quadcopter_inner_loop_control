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
#include "il_control_system/pid_controllers_clean.hpp"
#include "il_control_system/il_control.hpp"

using namespace il_control_system;

class IlControlSystem : public rclcpp::Node
{
public:
  IlControlSystem() : Node("il_control_system")
  {
    using std::placeholders::_1;

    this->declare_parameter("enable_il_control", false);
    enable_il_control_ = this->get_parameter("enable_il_control").as_bool();
    this->declare_parameter("bypass_pid", false);
    bypass_pid_ = this->get_parameter("bypass_pid").as_bool();

    // PID gains
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
    vel_accel_controller_  = std::make_unique<VelocityToAccelController>();
    accel_att_controller_  = std::make_unique<AccelToAttitudeController>();
    att_rate_controller_   = std::make_unique<AttitudeToRateController>();

    pos_controller_->setGains(
      this->get_parameter("position_kp_xy").as_double(),
      this->get_parameter("position_kp_z").as_double(),
      this->get_parameter("position_ki_xy").as_double(),
      this->get_parameter("position_ki_z").as_double());

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

    // Initialize IlControl
    il_control_ = std::make_unique<IlControl>();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.durability_volatile();

    // FIXED: Changed Il_ControlSystem to IlControlSystem
    sub_pos_setpoint_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/control/position_setpoint", 10,
      std::bind(&IlControlSystem::positionSetpointCb, this, _1));
    sub_position_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos,
      std::bind(&IlControlSystem::positionCb, this, _1));
    sub_attitude_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos,
      std::bind(&IlControlSystem::attitudeCb, this, _1));
    sub_ang_vel_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", qos,
      std::bind(&IlControlSystem::angularVelocityCb, this, _1));
      
    sub_direct_cmd_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/control/rl_actions", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 7) {
            direct_accel_cmd_ << msg->data[0], msg->data[1], msg->data[2];
            //std::cout << "Acc Command recieved: " << direct_accel_cmd_ << std::endl;
            direct_rate_cmd_  << msg->data[3], msg->data[4], msg->data[5];
            //std::cout << "Body rate Command recieved: " << direct_rate_cmd_ << std::endl;
            direct_yaw_cmd_   = msg->data[6];
            //std::cout << "Yaw Command recieved: " << direct_yaw_cmd_ << std::endl;
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
      std::bind(&IlControlSystem::controlLoopCallback, this));

    RCLCPP_INFO(get_logger(), "Il_Control System Initialized");
  }

private:

  void positionSetpointCb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    pos_desired_ << msg->point.x, msg->point.y, msg->point.z;
    //std::cout << "pos_desired: " << pos_desired_ << std::endl;
    setpoint_received_ = true;
  }

  void positionCb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    pos_current_ << msg->x, msg->y, msg->z;
    //std::cout << "pos_current: " << pos_current_ << std::endl;
    vel_current_ << msg->vx, msg->vy, msg->vz;
    //std::cout << "vel current: " << vel_current_ << std::endl;

    position_valid_ = true;
  }

  void attitudeCb(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
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
    ang_vel_current_ << msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2];
    ang_vel_valid_   = true;
  }

  void controlLoopCallback()
  {
    if (!position_valid_ || !ang_vel_valid_ || !attitude_valid_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for valid sensors...");
      return;
    }
    if (!bypass_pid_ && !setpoint_received_) return;
    if (bypass_pid_ && !direct_setpoint_received_) return;

    auto now = this->now();
    float dt = (now - control_loop_prev_time_).seconds();
    if (dt <= 0.0f || dt > 0.1f) { control_loop_prev_time_ = now; return; }
    control_loop_prev_time_ = now;

    Eigen::Vector3f rate_cmd, accel_cmd;
    float reference_yaw;

    // ── 1. Determine Acceleration and Rates based on Mode ────────────────────
    if (!bypass_pid_) {
      reference_yaw = yaw_current_; 
      if (loop_counter_ % 5 == 0) { // Position/Velocity run at 50Hz (250/5)
        Eigen::Vector3f vel_cmd = pos_controller_->update(pos_desired_, pos_current_);
        accel_cmd = vel_accel_controller_->update(vel_cmd, vel_current_);
        last_accel_cmd_ = accel_cmd;
      }  else {
        accel_cmd = last_accel_cmd_; // Use saved value
      }
    } // FIXED: Removed extra closing brace here
    else {
      // RL Direct Command Mode
      accel_cmd = direct_accel_cmd_;  
      rate_cmd = direct_rate_cmd_;
      reference_yaw = direct_yaw_cmd_; 
    }

    // ── 2. Convert Acceleration -> Attitude & Thrust ─────────────────────────
    att_out_ = accel_att_controller_->update(accel_cmd, reference_yaw);
    
    Eigen::Vector3f att_cmd = att_out_.attitude_cmd; 
    float thrust_cmd = att_out_.thrust_cmd;

    // ── 3. Determine Desired Body Rates ──────────────────────────────────────
    if (bypass_pid_) {
        // In RL mode, attitude controller yaw is forced to RL yaw
        att_cmd(2) = direct_yaw_cmd_;
    } else {
        // In PID mode, maintain current yaw heading if no yaw commands exist
        att_cmd(2) = yaw_current_;
        Eigen::Vector3f att_cur(roll_current_, pitch_current_, yaw_current_);
        rate_cmd = att_rate_controller_->update(att_cmd, att_cur); // Generate rates via PID
    }


    // ── Mode 1: PX4 PID-ONLY ─────────────────────────────────────────────────
    if (!enable_il_control_) {
      publishRatesSetpoint(rate_cmd, thrust_cmd);
      loop_counter_++;
      return;
    }

    // ── Mode 2: ADAPTIVE TORQUE ──────────────────────────────────────────────
    Eigen::Quaternionf q_desired;
    q_desired = Eigen::AngleAxisf(att_cmd(2), Eigen::Vector3f::UnitZ())
              * Eigen::AngleAxisf(att_cmd(1), Eigen::Vector3f::UnitY())
              * Eigen::AngleAxisf(att_cmd(0), Eigen::Vector3f::UnitX());
    q_desired.normalize();

    // Adaptive Update (Returns Physical Torque in N-m)
    Eigen::Vector3f physical_torque = il_control_->update(
        q_current_, ang_vel_current_, q_desired, rate_cmd, dt);

    // Publish Custom Allocated Torque
    publishControl(thrust_cmd, physical_torque);
    
    loop_counter_++;
  }

  void publishRatesSetpoint(const Eigen::Vector3f& rates, float thrust) {
    auto msg = px4_msgs::msg::VehicleRatesSetpoint();
    msg.timestamp      = this->get_clock()->now().nanoseconds() / 1000;
    msg.roll           = rates(0);
    msg.pitch          = rates(1);
    msg.yaw            = rates(2);
    msg.thrust_body[0] = 0.0f; msg.thrust_body[1] = 0.0f; msg.thrust_body[2] = -thrust;
    pub_rates_setpoint_->publish(msg);
  }

  void publishControl(float thrust, const Eigen::Vector3f& torque_nm) {
    // Thrust
    px4_msgs::msg::VehicleThrustSetpoint thrust_msg;
    thrust_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    thrust_msg.xyz[0] = 0.0f; thrust_msg.xyz[1] = 0.0f; thrust_msg.xyz[2] = -thrust;
    pub_thrust_->publish(thrust_msg);

    // PX4 Normalized Allocation Logic
    float raw_roll  = torque_nm(0);
    float raw_pitch = torque_nm(1);
    float raw_yaw   = torque_nm(2);
    
    float norm_pitch_cmd = 0.1493f * raw_pitch;
    float norm_yaw_cmd   = 1.6245f * raw_yaw;
    float norm_roll_cmd  = (0.1493f * raw_roll) - (0.0774f * raw_yaw); 

    // float norm_pitch_cmd = 0.2372f * raw_pitch;
    // float norm_yaw_cmd   = 2.5801f * raw_yaw;
    // float norm_roll_cmd  = (0.2372f * raw_roll) - (0.1229f * raw_yaw);  

    float norm_roll_final  = std::max(-1.0f, std::min(1.0f, norm_roll_cmd));
    float norm_pitch_final = std::max(-1.0f, std::min(1.0f, norm_pitch_cmd));

    float used_effort = std::max(std::abs(norm_roll_final), std::abs(norm_pitch_final));
    float yaw_headroom = std::max(0.0f, 1.0f - used_effort);
    float norm_yaw_final = std::max(-yaw_headroom, std::min(yaw_headroom, norm_yaw_cmd));

    px4_msgs::msg::VehicleTorqueSetpoint torque_msg;
    torque_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    // torque_msg.xyz[0] = norm_roll_final;
    // torque_msg.xyz[1] = norm_pitch_final;
    // torque_msg.xyz[2] = norm_yaw_final;
    torque_msg.xyz[0] = norm_roll_cmd;
    torque_msg.xyz[1] = norm_pitch_cmd;
    torque_msg.xyz[2] = norm_yaw_cmd;
    pub_torque_->publish(torque_msg);
  }


  // ===========================================================================
  // Members
  // ===========================================================================

  bool  enable_il_control_;
  bool bypass_pid_ = false;

  // PID controllers
  std::unique_ptr<PositionController>           pos_controller_;
  std::unique_ptr<VelocityToAccelController>    vel_accel_controller_; 
  std::unique_ptr<AccelToAttitudeController>    accel_att_controller_; 
  std::unique_ptr<AttitudeToRateController>     att_rate_controller_;

  // SMC controller
  std::unique_ptr<IlControl>                    il_control_; // FIXED: Used IlControl

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr      sub_pos_setpoint_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr   sub_position_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr        sub_attitude_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr        sub_ang_vel_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr      sub_direct_cmd_;

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

  Eigen::Vector3f last_accel_cmd_{Eigen::Vector3f::Zero()};
  AccelToAttitudeController::Output att_out_;

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
  float direct_yaw_cmd_ = 0.0f;

  // FIXED: Added missing control_loop_prev_time_ variable back
  rclcpp::Time control_loop_prev_time_{0, 0, RCL_ROS_TIME};
  int loop_counter_ = 0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IlControlSystem>());
  rclcpp::shutdown();
  return 0;
}