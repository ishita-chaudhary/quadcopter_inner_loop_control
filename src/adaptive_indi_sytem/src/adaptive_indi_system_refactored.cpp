// #include <rclcpp/rclcpp.hpp>
// #include <px4_msgs/msg/vehicle_local_position.hpp>
// #include <px4_msgs/msg/vehicle_angular_velocity.hpp>
// #include <px4_msgs/msg/vehicle_attitude.hpp>
// #include <px4_msgs/msg/esc_status.hpp>
// #include <px4_msgs/msg/actuator_motors.hpp>
// #include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
// #include <geometry_msgs/msg/point_stamped.hpp>
// #include <Eigen/Dense>
// #include "adaptive_indi_system/pid_controllers_clean.hpp"
// #include "adaptive_indi_system/controllers.hpp"
// #include "adaptive_indi_system/indi_proper.hpp"

// using namespace adaptive_indi_system;

// // ============================================================================
// // 2nd-Order Butterworth Low-Pass Filter
// // CRITICAL: Same filter MUST be used for both gyro differentiation and motor RPM
// // ============================================================================
// class SecondOrderLowPassFilter
// {
// public:
//   SecondOrderLowPassFilter() : a1_(0), a2_(0), b0_(0), b1_(0), b2_(0),
//                                x1_(0), x2_(0), y1_(0), y2_(0) {}
  
//   void configure(float cutoff_freq_hz, float sample_freq_hz)
//   {
//     // 2nd-order Butterworth filter coefficients
//     const float omega_c = 2.0f * M_PI * cutoff_freq_hz;
//     const float dt = 1.0f / sample_freq_hz;
//     const float omega_d = omega_c * dt;
    
//     // Bilinear transform
//     const float K = std::tan(omega_d / 2.0f);
//     const float K2 = K * K;
//     const float sqrt2 = std::sqrt(2.0f);
//     const float denom = K2 + sqrt2 * K + 1.0f;
    
//     // Filter coefficients
//     b0_ = K2 / denom;
//     b1_ = 2.0f * b0_;
//     b2_ = b0_;
    
//     a1_ = 2.0f * (K2 - 1.0f) / denom;
//     a2_ = (K2 - sqrt2 * K + 1.0f) / denom;
    
//     configured_ = true;
//   }
  
//   float update(float input)
//   {
//     if (!configured_) return input;
    
//     // Direct Form II implementation
//     float output = b0_ * input + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
    
//     // Update state
//     x2_ = x1_;
//     x1_ = input;
//     y2_ = y1_;
//     y1_ = output;
    
//     return output;
//   }
  
//   void reset()
//   {
//     x1_ = x2_ = y1_ = y2_ = 0.0f;
//   }

// private:
//   float a1_, a2_, b0_, b1_, b2_;
//   float x1_, x2_, y1_, y2_;
//   bool configured_ = false;
// };

// // ============================================================================
// // Vector3f Filter (applies same filter to each axis)
// // ============================================================================
// class Vector3Filter
// {
// public:
//   void configure(float cutoff_freq_hz, float sample_freq_hz)
//   {
//     for (int i = 0; i < 3; i++) {
//       filters_[i].configure(cutoff_freq_hz, sample_freq_hz);
//     }
//   }
  
//   Eigen::Vector3f update(const Eigen::Vector3f& input)
//   {
//     Eigen::Vector3f output;
//     for (int i = 0; i < 3; i++) {
//       output(i) = filters_[i].update(input(i));
//     }
//     return output;
//   }
  
//   void reset()
//   {
//     for (int i = 0; i < 3; i++) {
//       filters_[i].reset();
//     }
//   }

// private:
//   SecondOrderLowPassFilter filters_[3];
// };

// // ============================================================================
// // Vector4f Filter (for motor RPMs)
// // ============================================================================
// class Vector4Filter
// {
// public:
//   void configure(float cutoff_freq_hz, float sample_freq_hz)
//   {
//     for (int i = 0; i < 4; i++) {
//       filters_[i].configure(cutoff_freq_hz, sample_freq_hz);
//     }
//   }
  
//   Eigen::Vector4f update(const Eigen::Vector4f& input)
//   {
//     Eigen::Vector4f output;
//     for (int i = 0; i < 4; i++) {
//       output(i) = filters_[i].update(input(i));
//     }
//     return output;
//   }
  
//   void reset()
//   {
//     for (int i = 0; i < 4; i++) {
//       filters_[i].reset();
//     }
//   }

// private:
//   SecondOrderLowPassFilter filters_[4];
// };

// // ============================================================================
// // DUAL-MODE CONTROL SYSTEM - TRIPLE-LOOP PID CASCADE + FILTERED INDI
// // ============================================================================
// class AdaptiveINDISystem : public rclcpp::Node
// {
// public:
//   AdaptiveINDISystem() : Node("adaptive_indi_system")
//   {
//     using std::placeholders::_1;

//     // =====================================================================
//     // CONTROL MODE FLAG
//     // =====================================================================
//     this->declare_parameter("enable_indi", false);
//     enable_indi_ = this->get_parameter("enable_indi").as_bool();

//     // Physical parameters
//     this->declare_parameter("ixx", 0.02f);
//     this->declare_parameter("iyy", 0.02f);
//     this->declare_parameter("izz", 0.04f);
//     this->declare_parameter("arm_length_x", 0.17f);
//     this->declare_parameter("arm_length_y", 0.17f);
//     this->declare_parameter("k1_thrust", 0.00001f);
//     this->declare_parameter("k2_yaw", 0.00001f);
//     this->declare_parameter("rotor_inertia", 0.00005f);
    
//     // Control parameters
//     this->declare_parameter("hover_thrust", 0.70);
//     this->declare_parameter("learning_rate_mu1", 0.0001);
//     this->declare_parameter("learning_rate_mu2", 0.001);
//     this->declare_parameter("enable_adaptation", false);
    
//     // INDI Filter Parameters - CRITICAL FOR STABILITY
//     this->declare_parameter("indi_filter_cutoff_hz", 30.0);  // 30 Hz cutoff
//     this->declare_parameter("indi_sample_rate_hz", 250.0);   // 250 Hz control rate
    
//     // =====================================================================
//     // PID GAINS - TRIPLE-LOOP CASCADE
//     // =====================================================================
    
//     // OUTER LOOP: Position Controller (P only)
//     this->declare_parameter("position_kp_xy", 0.6);
//     this->declare_parameter("position_kp_z", 1.2);
    
//     // MIDDLE LOOP: Velocity Controller (PID)
//     this->declare_parameter("velocity_kp_xy", 0.5);
//     this->declare_parameter("velocity_ki_xy", 0.08);
//     this->declare_parameter("velocity_kd_xy", 0.15);
//     this->declare_parameter("velocity_kp_z", 0.8);
//     this->declare_parameter("velocity_ki_z", 0.10);
//     this->declare_parameter("velocity_kd_z", 0.12);
    
//     // INNER LOOP: Attitude Controller (PID)
//     this->declare_parameter("attitude_kp_roll", 3.0);
//     this->declare_parameter("attitude_ki_roll", 0.5);
//     this->declare_parameter("attitude_kd_roll", 0.15);
//     this->declare_parameter("attitude_kp_pitch", 3.0);
//     this->declare_parameter("attitude_ki_pitch", 0.5);
//     this->declare_parameter("attitude_kd_pitch", 0.15);
//     this->declare_parameter("attitude_kp_yaw", 2.0);
//     this->declare_parameter("attitude_ki_yaw", 0.3);
//     this->declare_parameter("attitude_kd_yaw", 0.10);
    
//     // Rate Controller (for INDI mode only)
//     this->declare_parameter("rate_kp", 2.5);
//     this->declare_parameter("rate_kd", 0.08);

//     // Get parameters
//     float ixx = this->get_parameter("ixx").as_double();
//     float iyy = this->get_parameter("iyy").as_double();
//     float izz = this->get_parameter("izz").as_double();
//     float arm_x = this->get_parameter("arm_length_x").as_double();
//     float arm_y = this->get_parameter("arm_length_y").as_double();
//     float k1 = this->get_parameter("k1_thrust").as_double();
//     float k2 = this->get_parameter("k2_yaw").as_double();
//     float rotor_inertia = this->get_parameter("rotor_inertia").as_double();
    
//     float hover_thrust = this->get_parameter("hover_thrust").as_double();
//     enable_adaptation_ = this->get_parameter("enable_adaptation").as_bool();
    
//     float indi_cutoff = this->get_parameter("indi_filter_cutoff_hz").as_double();
//     float indi_sample_rate = this->get_parameter("indi_sample_rate_hz").as_double();

//     hover_thrust_ = hover_thrust;

//     // =====================================================================
//     // Initialize Filters - SAME FILTER FOR BOTH GYRO AND MOTORS!
//     // =====================================================================
//     if (enable_indi_) {
//       gyro_filter_.configure(indi_cutoff, indi_sample_rate);
//       motor_filter_.configure(indi_cutoff, indi_sample_rate);
      
//       RCLCPP_INFO(get_logger(), "  INDI Filters Configured:");
//       RCLCPP_INFO(get_logger(), "    Cutoff Frequency: %.1f Hz", indi_cutoff);
//       RCLCPP_INFO(get_logger(), "    Sample Rate: %.1f Hz", indi_sample_rate);
//     }

//     // =====================================================================
//     // Initialize PID Controllers
//     // =====================================================================
//     pos_controller_ = std::make_unique<PositionController>();
//     vel_att_controller_ = std::make_unique<VelocityToAttitudeController>();
//     att_rate_controller_ = std::make_unique<AttitudeToRateController>();
//     rate_accel_controller_ = std::make_unique<RateToAccelController>();
    
//     // =====================================================================
//     // Load PID Gains from Config
//     // =====================================================================
    
//     // OUTER LOOP: Position Controller (P only)
//     float pos_kp_xy = this->get_parameter("position_kp_xy").as_double();
//     float pos_kp_z = this->get_parameter("position_kp_z").as_double();
//     pos_controller_->setGains(pos_kp_xy, pos_kp_z);
    
//     // MIDDLE LOOP: Velocity Controller (PID)
//     float vel_kp_xy = this->get_parameter("velocity_kp_xy").as_double();
//     float vel_ki_xy = this->get_parameter("velocity_ki_xy").as_double();
//     float vel_kd_xy = this->get_parameter("velocity_kd_xy").as_double();
//     float vel_kp_z = this->get_parameter("velocity_kp_z").as_double();
//     float vel_ki_z = this->get_parameter("velocity_ki_z").as_double();
//     float vel_kd_z = this->get_parameter("velocity_kd_z").as_double();
//     vel_att_controller_->setVelocityGain(vel_kp_xy, vel_ki_xy, vel_kd_xy, 
//                                          vel_kp_z, vel_ki_z, vel_kd_z);
    
//     // INNER LOOP: Attitude Controller (PID)
//     float att_kp_roll = this->get_parameter("attitude_kp_roll").as_double();
//     float att_ki_roll = this->get_parameter("attitude_ki_roll").as_double();
//     float att_kd_roll = this->get_parameter("attitude_kd_roll").as_double();
//     float att_kp_pitch = this->get_parameter("attitude_kp_pitch").as_double();
//     float att_ki_pitch = this->get_parameter("attitude_ki_pitch").as_double();
//     float att_kd_pitch = this->get_parameter("attitude_kd_pitch").as_double();
//     float att_kp_yaw = this->get_parameter("attitude_kp_yaw").as_double();
//     float att_ki_yaw = this->get_parameter("attitude_ki_yaw").as_double();
//     float att_kd_yaw = this->get_parameter("attitude_kd_yaw").as_double();
//     att_rate_controller_->setGains(att_kp_roll, att_ki_roll, att_kd_roll,
//                                    att_kp_pitch, att_ki_pitch, att_kd_pitch,
//                                    att_kp_yaw, att_ki_yaw, att_kd_yaw);
    
//     // Rate Controller (for INDI mode)
//     float rate_kp = this->get_parameter("rate_kp").as_double();
//     float rate_kd = this->get_parameter("rate_kd").as_double();
//     rate_accel_controller_->setGain(rate_kp, rate_kd);

//     // =====================================================================
//     // Initialize PROPER INDI Controller
//     // =====================================================================
//     indi_controller_proper_ = std::make_unique<INDIController>(
//       ixx, iyy, izz,
//       arm_x, arm_y,
//       k1, k2,
//       rotor_inertia,
//       250.0f,  // 250 Hz control loop
//       hover_thrust
//     );

//     // QoS for PX4
//     auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
//     qos.best_effort();
//     qos.durability_volatile();

//     // =====================================================================
//     // Subscribers
//     // =====================================================================
//     sub_pos_setpoint_ = create_subscription<geometry_msgs::msg::PointStamped>(
//       "/control/position_setpoint", 10,
//       std::bind(&AdaptiveINDISystem::positionSetpointCb, this, _1));

//     sub_position_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
//       "/fmu/out/vehicle_local_position_v1", qos,
//       std::bind(&AdaptiveINDISystem::positionCb, this, _1));

//     sub_attitude_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
//       "/fmu/out/vehicle_attitude", qos,
//       std::bind(&AdaptiveINDISystem::attitudeCb, this, _1));

//     sub_ang_vel_ = create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
//       "/fmu/out/vehicle_angular_velocity", qos,
//       std::bind(&AdaptiveINDISystem::angularVelocityCb, this, _1));

//     sub_esc_status_ = create_subscription<px4_msgs::msg::EscStatus>(
//       "/fmu/out/esc_status", qos,
//       std::bind(&AdaptiveINDISystem::escStatusCb, this, _1));

//     // =====================================================================
//     // Publishers
//     // =====================================================================
//     pub_rates_setpoint_ = create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
//       "/fmu/in/vehicle_rates_setpoint", 10);

//     pub_actuator_motors_ = create_publisher<px4_msgs::msg::ActuatorMotors>(
//       "/fmu/in/actuator_motors", 10);

//     // Timers
//     timer_control_loop_ = create_wall_timer(
//       std::chrono::milliseconds(4),
//       std::bind(&AdaptiveINDISystem::controlLoopCallback, this));

//     timer_diagnostics_ = create_wall_timer(
//       std::chrono::seconds(1),
//       std::bind(&AdaptiveINDISystem::diagnosticsCallback, this));

//     // Print startup info
//     RCLCPP_INFO(get_logger(), " ");
//     RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════════════");
//     RCLCPP_INFO(get_logger(), "  Adaptive INDI - TRIPLE-LOOP PID + SYNCHRONIZED FILTERS");
//     RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════════════");
//     RCLCPP_INFO(get_logger(), "  Control Mode: %s", enable_indi_ ? "PID + INDI" : "PID-ONLY");
//     RCLCPP_INFO(get_logger(), " ");
//     RCLCPP_INFO(get_logger(), "  PID CASCADE STRUCTURE:");
//     RCLCPP_INFO(get_logger(), "    Outer Loop:  Position → Velocity (P)");
//     RCLCPP_INFO(get_logger(), "    Middle Loop: Velocity → Attitude + Thrust (PID)");
//     RCLCPP_INFO(get_logger(), "    Inner Loop:  Attitude → Rate (PID)");
//     RCLCPP_INFO(get_logger(), " ");
//     if (!enable_indi_) {
//       RCLCPP_INFO(get_logger(), "  PID-ONLY MODE");
//       RCLCPP_INFO(get_logger(), "  Publishing to: /fmu/in/vehicle_rates_setpoint");
//     } else {
//       RCLCPP_INFO(get_logger(), "  PID + INDI MODE");
//       RCLCPP_INFO(get_logger(), "  Publishing to: /fmu/in/actuator_motors");
//       RCLCPP_INFO(get_logger(), "  Adaptation: %s", enable_adaptation_ ? "ON" : "OFF");
//       RCLCPP_INFO(get_logger(), "  Filter Cutoff: %.1f Hz", indi_cutoff);
//     }
//     RCLCPP_INFO(get_logger(), " ");
//     RCLCPP_INFO(get_logger(), "  Physical Parameters:");
//     RCLCPP_INFO(get_logger(), "    Ixx: %.6f kg·m²", ixx);
//     RCLCPP_INFO(get_logger(), "    Iyy: %.6f kg·m²", iyy);
//     RCLCPP_INFO(get_logger(), "    Izz: %.6f kg·m²", izz);
//     RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════════════");
//   }

// private:
//   // =====================================================================
//   // Callbacks
//   // =====================================================================
  
//   void positionSetpointCb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
//   {
//     pos_desired_ << msg->point.x, msg->point.y, msg->point.z;
//     setpoint_received_ = true;
//   }

//   void positionCb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
//   {
//     pos_current_ << msg->x, msg->y, msg->z;
//     vel_current_ << msg->vx, msg->vy, msg->vz;
//     position_valid_ = true;
//   }

//   void attitudeCb(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
//   {
//     float qw = msg->q[0];
//     float qx = msg->q[1];
//     float qy = msg->q[2];
//     float qz = msg->q[3];
    
//     float sinr_cosp = 2.0f * (qw * qx + qy * qz);
//     float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
//     roll_current_ = std::atan2(sinr_cosp, cosr_cosp);
    
//     float sinp = 2.0f * (qw * qy - qz * qx);
//     if (std::abs(sinp) >= 1.0f)
//       pitch_current_ = std::copysign(M_PI / 2, sinp);
//     else
//       pitch_current_ = std::asin(sinp);
    
//     attitude_valid_ = true;
//   }

//   void angularVelocityCb(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg)
//   {
//     // Store RAW angular velocity
//     ang_vel_raw_ << msg->xyz[0], msg->xyz[1], msg->xyz[2];
    
//     if (enable_indi_) {
//       // Apply SAME filter to gyro as we do to motors
//       ang_vel_filtered_ = gyro_filter_.update(ang_vel_raw_);
      
//       // Compute FILTERED angular acceleration (for INDI)
//       if (ang_vel_prev_valid_) {
//         float dt = 0.004f;  // 250 Hz
//         ang_accel_filtered_ = (ang_vel_filtered_ - ang_vel_filtered_prev_) / dt;
//       }
//       ang_vel_filtered_prev_ = ang_vel_filtered_;
//       ang_vel_prev_valid_ = true;
//     }
    
//     // For PID mode, use raw angular velocity
//     ang_vel_current_ = ang_vel_raw_;
//     ang_vel_valid_ = true;
//   }

//   void escStatusCb(const px4_msgs::msg::EscStatus::SharedPtr msg)
//   {
//     size_t num_motors = std::min(static_cast<size_t>(msg->esc_count), size_t(4));
    
//     // Store RAW motor RPMs
//     Eigen::Vector4f rpm_raw;
//     for (size_t i = 0; i < num_motors; i++) {
//       rpm_raw(i) = static_cast<float>(msg->esc[i].esc_rpm);
//     }
    
//     if (enable_indi_) {
//       // Apply SAME filter to motors as we do to gyro
//       rpm_filtered_ = motor_filter_.update(rpm_raw);
//       rpm_current_ = rpm_filtered_;
//     } else {
//       // For PID mode, use raw RPM
//       rpm_current_ = rpm_raw;
//     }
    
//     esc_valid_ = true;
//   }

//   // =====================================================================
//   // Control Loop - TRIPLE-LOOP PID CASCADE + SYNCHRONIZED FILTERING
//   // =====================================================================
//   void controlLoopCallback()
//   {
//     if (!position_valid_ || !ang_vel_valid_ || !esc_valid_ || !setpoint_received_) {
//       return;
//     }

//     auto now = this->now();
//     float dt = (now - control_loop_prev_time_).seconds();
//     if (dt <= 0.0f || dt > 0.1f) {
//       control_loop_prev_time_ = now;
//       return;
//     }
//     control_loop_prev_time_ = now;

//     // =====================================================================
//     // TRIPLE-LOOP PID CASCADE
//     // =====================================================================
    
//     // OUTER LOOP: Position → Velocity (P only)
//     Eigen::Vector3f vel_cmd = pos_controller_->update(pos_desired_, pos_current_);
    
//     // MIDDLE LOOP: Velocity → Attitude + Thrust (PID)
//     auto vel_out = vel_att_controller_->update(vel_cmd, vel_current_);
//     Eigen::Vector3f att_cmd = vel_out.attitude_cmd;
//     float thrust_cmd = vel_out.thrust_cmd;
    
//     // INNER LOOP: Attitude → Rate (PID)
//     Eigen::Vector3f att_current;
//     att_current << roll_current_, pitch_current_, 0.0f;
//     Eigen::Vector3f rate_cmd = att_rate_controller_->update(att_cmd, att_current);
    
//     // For INDI mode only: Rate → Angular Acceleration
//     Eigen::Vector3f accel_cmd;
//     if (enable_indi_) {
//       accel_cmd = rate_accel_controller_->update(rate_cmd, ang_vel_filtered_);
//     }

//     // =====================================================================
//     // MODE SELECTION
//     // =====================================================================
    
//     if (!enable_indi_) {
//       // =====================================================================
//       // MODE 1: PID-ONLY (Triple-Loop Cascade to Rate Setpoint)
//       // =====================================================================
//       publishRatesSetpoint(rate_cmd, thrust_cmd);
//       last_thrust_cmd_ = thrust_cmd;
      
//     } else {
//       // =====================================================================
//       // MODE 2: PID + INDI WITH SYNCHRONIZED FILTERING
//       // =====================================================================
      
//       // CRITICAL: Pass FILTERED gyro acceleration and FILTERED motor RPMs
//       // Both use the SAME 2nd-order Butterworth filter (30 Hz cutoff)
//       Eigen::Vector4f motor_increment = indi_controller_proper_->update(
//         accel_cmd,           // Desired acceleration from PID
//         ang_vel_filtered_,   // FILTERED angular velocity (not raw!)
//         rpm_filtered_);      // FILTERED motor RPMs (not raw!)
      
//       // Compute baseline motor commands from thrust
//       float omega_sq_hover = std::max(0.01f, thrust_cmd * thrust_cmd);
//       float omega_hover = std::sqrt(omega_sq_hover);
//       float motor_baseline = omega_hover;  // Normalized
      
//       // Apply INDI increments to baseline
//       Eigen::Vector4f motor_cmd_normalized;
//       for (int i = 0; i < 4; i++) {
//         motor_cmd_normalized(i) = motor_baseline + motor_increment(i);
//       }
      
//       // Safety limits
//       for (int i = 0; i < 4; i++) {
//         motor_cmd_normalized(i) = std::max(0.0f, std::min(1.0f, motor_cmd_normalized(i)));
//       }
      
//       // Publish normalized motor commands
//       publishMotorCommands(motor_cmd_normalized);
//       last_thrust_cmd_ = thrust_cmd;
//       last_motor_cmd_ = motor_cmd_normalized;
//       last_motor_increment_ = motor_increment;
//       last_ang_accel_filtered_ = ang_accel_filtered_;
//     }

//     control_loop_count_++;
    
//     // Debug logging
//     if (control_loop_count_ % 50 == 0) {
//       if (enable_indi_) {
//         RCLCPP_DEBUG(get_logger(),
//                      "[INDI] Thrust: %.3f | AngVel_raw: [%.3f, %.3f, %.3f] | "
//                      "AngVel_filt: [%.3f, %.3f, %.3f] | AngAccel: [%.2f, %.2f, %.2f]",
//                      thrust_cmd,
//                      ang_vel_raw_(0), ang_vel_raw_(1), ang_vel_raw_(2),
//                      ang_vel_filtered_(0), ang_vel_filtered_(1), ang_vel_filtered_(2),
//                      ang_accel_filtered_(0), ang_accel_filtered_(1), ang_accel_filtered_(2));
//       } else {
//         RCLCPP_DEBUG(get_logger(),
//                      "[PID] Thrust: %.3f Rate: [%.3f, %.3f, %.3f]",
//                      thrust_cmd,
//                      ang_vel_current_(0), ang_vel_current_(1), ang_vel_current_(2));
//       }
//     }
//   }

//   // =====================================================================
//   // Output Publishing Functions
//   // =====================================================================

//   void publishRatesSetpoint(const Eigen::Vector3f& rates, float thrust)
//   {
//     auto msg = px4_msgs::msg::VehicleRatesSetpoint();
//     msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
//     msg.roll = rates(0);
//     msg.pitch = rates(1);
//     msg.yaw = rates(2);
    
//     msg.thrust_body[0] = 0.0f;
//     msg.thrust_body[1] = 0.0f;
//     msg.thrust_body[2] = -thrust;  // Negative for NED
    
//     pub_rates_setpoint_->publish(msg);
//   }

//   void publishMotorCommands(const Eigen::Vector4f& motor_cmd)
//   {
//     auto msg = px4_msgs::msg::ActuatorMotors();
//     msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
//     for (int i = 0; i < 4; i++) {
//       msg.control[i] = std::max(0.0f, std::min(1.0f, motor_cmd(i)));
//     }
    
//     for (int i = 4; i < 12; i++) {
//       msg.control[i] = NAN;
//     }
    
//     pub_actuator_motors_->publish(msg);
//   }

//   void diagnosticsCallback()
//   {
//     if (!position_valid_ || !ang_vel_valid_ || !esc_valid_ || !setpoint_received_) {
//       RCLCPP_WARN(get_logger(), "Waiting for data:");
//       if (!position_valid_) RCLCPP_WARN(get_logger(), "  - Position");
//       if (!ang_vel_valid_) RCLCPP_WARN(get_logger(), "  - Angular velocity");
//       if (!esc_valid_) RCLCPP_WARN(get_logger(), "  - ESC status");
//       if (!setpoint_received_) RCLCPP_WARN(get_logger(), "  - Position setpoint");
//     } else {
//       if (enable_indi_) {
//         // INDI mode diagnostics - show filter effectiveness
//         auto motor_inc = indi_controller_proper_->getLastMotorIncrement();
//         RCLCPP_INFO(get_logger(),
//                     "[INDI] Pos: [%.2f, %.2f, %.2f] m | Thrust: %.3f | "
//                     "Motors: [%.3f, %.3f, %.3f, %.3f] | "
//                     "AngAccel_filt: [%.1f, %.1f, %.1f] rad/s² | "
//                     "Motor_incr: [%.4f, %.4f, %.4f, %.4f]",
//                     pos_current_(0), pos_current_(1), pos_current_(2),
//                     last_thrust_cmd_,
//                     last_motor_cmd_(0), last_motor_cmd_(1), 
//                     last_motor_cmd_(2), last_motor_cmd_(3),
//                     last_ang_accel_filtered_(0), last_ang_accel_filtered_(1), 
//                     last_ang_accel_filtered_(2),
//                     motor_inc(0), motor_inc(1), motor_inc(2), motor_inc(3));
//       } else {
//         // PID-only mode diagnostics
//         RCLCPP_INFO(get_logger(),
//                     "[PID] Pos: [%.2f, %.2f, %.2f] m | "
//                     "Err: [%.3f, %.3f, %.3f] m | "
//                     "Thrust: %.3f | "
//                     "AngVel: [%.2f, %.2f, %.2f] rad/s",
//                     pos_current_(0), pos_current_(1), pos_current_(2),
//                     pos_desired_(0) - pos_current_(0),
//                     pos_desired_(1) - pos_current_(1),
//                     pos_desired_(2) - pos_current_(2),
//                     last_thrust_cmd_,
//                     ang_vel_current_(0), ang_vel_current_(1), ang_vel_current_(2));
//       }
//     }
//   }

//   // =====================================================================
//   // Member Variables
//   // =====================================================================
  
//   // Mode flags
//   bool enable_indi_;
//   bool enable_adaptation_;

//   // PID Controllers
//   std::unique_ptr<PositionController> pos_controller_;
//   std::unique_ptr<VelocityToAttitudeController> vel_att_controller_;
//   std::unique_ptr<AttitudeToRateController> att_rate_controller_;
//   std::unique_ptr<RateToAccelController> rate_accel_controller_;
  
//   // INDI Controller
//   std::unique_ptr<INDIController> indi_controller_proper_;
  
//   // CRITICAL: Synchronized Filters
//   Vector3Filter gyro_filter_;       // Same filter for gyro
//   Vector4Filter motor_filter_;      // Same filter for motors
  
//   // Subscribers
//   rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_pos_setpoint_;
//   rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_position_;
//   rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_attitude_;
//   rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr sub_ang_vel_;
//   rclcpp::Subscription<px4_msgs::msg::EscStatus>::SharedPtr sub_esc_status_;
  
//   // Publishers
//   rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr pub_rates_setpoint_;
//   rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr pub_actuator_motors_;
  
//   // Timers
//   rclcpp::TimerBase::SharedPtr timer_control_loop_;
//   rclcpp::TimerBase::SharedPtr timer_diagnostics_;
  
//   // State - Raw signals
//   Eigen::Vector3f pos_desired_{Eigen::Vector3f::Zero()};
//   Eigen::Vector3f pos_current_{Eigen::Vector3f::Zero()};
//   Eigen::Vector3f vel_current_{Eigen::Vector3f::Zero()};
//   Eigen::Vector3f ang_vel_raw_{Eigen::Vector3f::Zero()};      // Raw gyro
//   Eigen::Vector3f ang_vel_current_{Eigen::Vector3f::Zero()};  // Used by PID
//   Eigen::Vector4f rpm_current_{Eigen::Vector4f::Zero()};      // Can be raw or filtered
  
//   // State - Filtered signals (INDI only)
//   Eigen::Vector3f ang_vel_filtered_{Eigen::Vector3f::Zero()};
//   Eigen::Vector3f ang_vel_filtered_prev_{Eigen::Vector3f::Zero()};
//   Eigen::Vector3f ang_accel_filtered_{Eigen::Vector3f::Zero()};
//   Eigen::Vector4f rpm_filtered_{Eigen::Vector4f::Zero()};
//   bool ang_vel_prev_valid_ = false;
  
//   float roll_current_ = 0.0f;
//   float pitch_current_ = 0.0f;
//   float hover_thrust_ = 0.55f;

//   // Diagnostic tracking
//   float last_thrust_cmd_ = 0.0f;
//   Eigen::Vector4f last_motor_cmd_{Eigen::Vector4f::Zero()};
//   Eigen::Vector4f last_motor_increment_{Eigen::Vector4f::Zero()};
//   Eigen::Vector3f last_ang_accel_filtered_{Eigen::Vector3f::Zero()};
  
//   // Flags
//   bool setpoint_received_ = false;
//   bool position_valid_ = false;
//   bool attitude_valid_ = false;
//   bool ang_vel_valid_ = false;
//   bool esc_valid_ = false;
  
//   // Timing
//   rclcpp::Time control_loop_prev_time_{0, 0, RCL_ROS_TIME};
//   int control_loop_count_ = 0;
// };

// // ============================================================================
// // Main function
// // ============================================================================
// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<AdaptiveINDISystem>());
//   rclcpp::shutdown();
//   return 0;
// }






















#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/esc_status.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <Eigen/Dense>
#include "adaptive_indi_system/pid_controllers_clean.hpp"
#include "adaptive_indi_system/controllers.hpp"
#include "adaptive_indi_system/indi_proper.hpp"

using namespace adaptive_indi_system;

// ============================================================================
// 2nd-Order Butterworth Low-Pass Filter
// CRITICAL: Gyro and motor RPM MUST use the same cutoff (Smeur 2016 §III.B)
// This ensures the phase delay is matched, which is necessary for INDI
// to correctly estimate the angular acceleration contribution of each motor.
// ============================================================================
class SecondOrderLowPassFilter
{
public:
  SecondOrderLowPassFilter()
    : a1_(0), a2_(0), b0_(0), b1_(0), b2_(0),
      x1_(0), x2_(0), y1_(0), y2_(0) {}

  void configure(float cutoff_hz, float sample_hz)
  {
    const float K     = std::tan(M_PI * cutoff_hz / sample_hz);
    const float K2    = K * K;
    const float sq2   = std::sqrt(2.0f);
    const float denom = K2 + sq2 * K + 1.0f;
    b0_ = K2 / denom;
    b1_ = 2.0f * b0_;
    b2_ = b0_;
    a1_ = 2.0f * (K2 - 1.0f) / denom;
    a2_ = (K2 - sq2 * K + 1.0f) / denom;
    configured_ = true;
  }

  float update(float x)
  {
    if (!configured_) return x;
    float y = b0_*x + b1_*x1_ + b2_*x2_ - a1_*y1_ - a2_*y2_;
    x2_ = x1_; x1_ = x;
    y2_ = y1_; y1_ = y;
    return y;
  }

  void reset() { x1_ = x2_ = y1_ = y2_ = 0.0f; }

private:
  float a1_, a2_, b0_, b1_, b2_;
  float x1_, x2_, y1_, y2_;
  bool  configured_ = false;
};

class Vector3Filter {
public:
  void configure(float c, float s) { for (auto& f:f_) f.configure(c,s); }
  Eigen::Vector3f update(const Eigen::Vector3f& v) {
    Eigen::Vector3f r; for (int i=0;i<3;i++) r(i)=f_[i].update(v(i)); return r;
  }
  void reset() { for (auto& f:f_) f.reset(); }
private: SecondOrderLowPassFilter f_[3];
};

class Vector4Filter {
public:
  void configure(float c, float s) { for (auto& f:f_) f.configure(c,s); }
  Eigen::Vector4f update(const Eigen::Vector4f& v) {
    Eigen::Vector4f r; for (int i=0;i<4;i++) r(i)=f_[i].update(v(i)); return r;
  }
  void reset() { for (auto& f:f_) f.reset(); }
private: SecondOrderLowPassFilter f_[4];
};

// ============================================================================
// AdaptiveINDISystem
//
// Architecture (Smeur 2016 + cascaded INDI 2018):
//
//   Position → [PID] → Velocity setpoint
//   Velocity → [PID] → Attitude setpoint + THRUST CMD
//   Attitude → [PID] → Rate setpoint
//   Rate     → [PID] → Angular acceleration setpoint (α_des)
//
//   THRUST CMD  ──────────────────────────────────────────────────────────┐
//   (from vel PID, handles altitude)                                       │
//                                                                          ▼
//   α_des ─→ [INDI] ─→ Δω² (differential) ──────────────→ [MIX] ─→ ω²_i = collective + differential_i
//              ↑                                             ↑         ↓
//           α_meas                                   ω²_collective  normalize → motor_cmd [0,1]
//           (filtered)                            = thrust_cmd * ω²_max
//
// Key separation (from Smeur 2016 §III.C):
//   "Control of this fourth axis [thrust] is handled by a separate controller.
//    This controller scales the average motor speed."
//
// INDI provides PURE ATTITUDE corrections (Δω² sums to ~0 across motors).
// Thrust PID provides COLLECTIVE command (same for all motors).
// They are added together — no interference.
// ============================================================================
class AdaptiveINDISystem : public rclcpp::Node
{
public:
  AdaptiveINDISystem() : Node("adaptive_indi_system")
  {
    using std::placeholders::_1;

    this->declare_parameter("enable_indi",       false);
    this->declare_parameter("enable_adaptation", false);
    enable_indi_       = this->get_parameter("enable_indi").as_bool();
    enable_adaptation_ = this->get_parameter("enable_adaptation").as_bool();

    // Physical parameters
    this->declare_parameter("ixx",           0.029125);
    this->declare_parameter("iyy",           0.029125);
    this->declare_parameter("izz",           0.055225);
    this->declare_parameter("arm_length_x",  0.13);
    this->declare_parameter("arm_length_y",  0.21);
    this->declare_parameter("k1_thrust",     0.00001);
    this->declare_parameter("k2_yaw",        0.00001);
    this->declare_parameter("rotor_inertia", 0.00005);
    this->declare_parameter("hover_thrust",  0.60);
    this->declare_parameter("omega_max",     1571.0);  // ~15000 RPM for x500
    this->declare_parameter("learning_rate_mu1", 0.0001);
    this->declare_parameter("learning_rate_mu2", 0.001);

    // Filter
    this->declare_parameter("indi_filter_cutoff_hz", 30.0);
    this->declare_parameter("indi_sample_rate_hz",   250.0);

    // PID gains
    this->declare_parameter("position_kp_xy",   0.6);
    this->declare_parameter("position_kp_z",    1.2);
    this->declare_parameter("velocity_kp_xy",   0.5);
    this->declare_parameter("velocity_ki_xy",   0.08);
    this->declare_parameter("velocity_kd_xy",   0.15);
    this->declare_parameter("velocity_kp_z",    0.8);
    this->declare_parameter("velocity_ki_z",    0.10);
    this->declare_parameter("velocity_kd_z",    0.12);
    this->declare_parameter("attitude_kp_roll",  3.0);
    this->declare_parameter("attitude_ki_roll",  0.5);
    this->declare_parameter("attitude_kd_roll",  0.15);
    this->declare_parameter("attitude_kp_pitch", 3.0);
    this->declare_parameter("attitude_ki_pitch", 0.5);
    this->declare_parameter("attitude_kd_pitch", 0.15);
    this->declare_parameter("attitude_kp_yaw",   2.0);
    this->declare_parameter("attitude_ki_yaw",   0.3);
    this->declare_parameter("attitude_kd_yaw",   0.10);
    this->declare_parameter("rate_kp", 2.5);
    this->declare_parameter("rate_kd", 0.08);

    float ixx         = this->get_parameter("ixx").as_double();
    float iyy         = this->get_parameter("iyy").as_double();
    float izz         = this->get_parameter("izz").as_double();
    float arm_x       = this->get_parameter("arm_length_x").as_double();
    float arm_y       = this->get_parameter("arm_length_y").as_double();
    float k1          = this->get_parameter("k1_thrust").as_double();
    float k2          = this->get_parameter("k2_yaw").as_double();
    float rot_i       = this->get_parameter("rotor_inertia").as_double();
    float hover_thr   = this->get_parameter("hover_thrust").as_double();
    omega_max_        = this->get_parameter("omega_max").as_double();
    omega_sq_max_     = omega_max_ * omega_max_;
    float indi_cutoff = this->get_parameter("indi_filter_cutoff_hz").as_double();
    float indi_sr     = this->get_parameter("indi_sample_rate_hz").as_double();
    hover_thrust_     = hover_thr;

    if (enable_indi_) {
      gyro_filter_.configure(indi_cutoff, indi_sr);
      motor_filter_.configure(indi_cutoff, indi_sr);
    }

    pos_controller_        = std::make_unique<PositionController>();
    vel_att_controller_    = std::make_unique<VelocityToAttitudeController>();
    att_rate_controller_   = std::make_unique<AttitudeToRateController>();
    rate_accel_controller_ = std::make_unique<RateToAccelController>();

    pos_controller_->setGains(
      this->get_parameter("position_kp_xy").as_double(),
      this->get_parameter("position_kp_z").as_double());

    vel_att_controller_->setVelocityGain(
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

    rate_accel_controller_->setGain(
      this->get_parameter("rate_kp").as_double(),
      this->get_parameter("rate_kd").as_double());

    indi_ = std::make_unique<INDIController>(
      ixx, iyy, izz, arm_x, arm_y, k1, k2, rot_i, indi_sr, omega_max_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.durability_volatile();

    sub_pos_setpoint_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/control/position_setpoint", 10,
      std::bind(&AdaptiveINDISystem::positionSetpointCb, this, _1));
    sub_position_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos,
      std::bind(&AdaptiveINDISystem::positionCb, this, _1));
    sub_attitude_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos,
      std::bind(&AdaptiveINDISystem::attitudeCb, this, _1));
    sub_ang_vel_ = create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
      "/fmu/out/vehicle_angular_velocity", qos,
      std::bind(&AdaptiveINDISystem::angularVelocityCb, this, _1));
    sub_esc_status_ = create_subscription<px4_msgs::msg::EscStatus>(
      "/fmu/out/esc_status", qos,
      std::bind(&AdaptiveINDISystem::escStatusCb, this, _1));

    pub_rates_setpoint_ = create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
      "/fmu/in/vehicle_rates_setpoint", 10);
    pub_actuator_motors_ = create_publisher<px4_msgs::msg::ActuatorMotors>(
      "/fmu/in/actuator_motors", 10);

    timer_control_loop_ = create_wall_timer(
      std::chrono::milliseconds(4),
      std::bind(&AdaptiveINDISystem::controlLoopCallback, this));
    timer_diagnostics_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&AdaptiveINDISystem::diagnosticsCallback, this));

    RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════════════");
    RCLCPP_INFO(get_logger(), "  Adaptive INDI System (Smeur 2016 architecture)");
    RCLCPP_INFO(get_logger(), "  Mode: %s", enable_indi_ ? "PID-THRUST + INDI-ATTITUDE" : "PID-ONLY");
    if (enable_indi_) {
      RCLCPP_INFO(get_logger(), " ");
      RCLCPP_INFO(get_logger(), "  INDI domain: ω² (constant G, no singularity)");
      RCLCPP_INFO(get_logger(), "  Thrust:   velocity PID → collective ω²");
      RCLCPP_INFO(get_logger(), "  Attitude: INDI → differential Δω² (zero-mean)");
      RCLCPP_INFO(get_logger(), "  Motor_i:  (collective + differential_i) / ω²_max");
      RCLCPP_INFO(get_logger(), " ");
      RCLCPP_INFO(get_logger(), "  Filter: %.0f Hz cutoff (same for gyro + RPM)", indi_cutoff);
      RCLCPP_INFO(get_logger(), "  omega_max:    %.0f rad/s (%.0f RPM)",
                  omega_max_, omega_max_ * 60.0f / (2.0f * M_PI));
      RCLCPP_INFO(get_logger(), "  omega_sq_max: %.0f (rad/s)²", omega_sq_max_);
    }
    RCLCPP_INFO(get_logger(), "  hover_thrust: %.2f", hover_thr);
    RCLCPP_INFO(get_logger(), "  arm: x=%.3f m  y=%.3f m", arm_x, arm_y);
    RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════════════");
  }

private:

  // ===========================================================================
  // Sensor callbacks — your working PID code, unchanged
  // ===========================================================================

  void positionSetpointCb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    pos_desired_ << msg->point.x, msg->point.y, msg->point.z;
    setpoint_received_ = true;
  }

  void positionCb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    pos_current_ << msg->x, msg->y, msg->z;
    vel_current_ << msg->vx, msg->vy, msg->vz;
    position_valid_ = true;
  }

  void attitudeCb(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    float qw=msg->q[0], qx=msg->q[1], qy=msg->q[2], qz=msg->q[3];
    roll_current_  = std::atan2(2.0f*(qw*qx+qy*qz), 1.0f-2.0f*(qx*qx+qy*qy));
    float sp       = 2.0f*(qw*qy-qz*qx);
    pitch_current_ = (std::abs(sp)>=1.0f) ? std::copysign(M_PI/2.0f,sp) : std::asin(sp);
    attitude_valid_ = true;
  }

  void angularVelocityCb(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg)
  {
    ang_vel_raw_ << msg->xyz[0], msg->xyz[1], msg->xyz[2];

    if (enable_indi_) {
      // Filter gyro with SAME filter as motor RPM (Smeur 2016 §III.B)
      ang_vel_filtered_ = gyro_filter_.update(ang_vel_raw_);

      // Differentiate to get angular acceleration
      if (ang_vel_prev_valid_) {
        ang_accel_filtered_ = (ang_vel_filtered_ - ang_vel_filtered_prev_) / kDt_;
      }
      ang_vel_filtered_prev_ = ang_vel_filtered_;
      ang_vel_prev_valid_    = true;
    }

    ang_vel_current_ = ang_vel_raw_;
    ang_vel_valid_   = true;
  }

  void escStatusCb(const px4_msgs::msg::EscStatus::SharedPtr msg)
  {
    size_t n = std::min(static_cast<size_t>(msg->esc_count), size_t(4));
    Eigen::Vector4f raw = Eigen::Vector4f::Zero();
    for (size_t i = 0; i < n; i++) {
      raw(i) = static_cast<float>(msg->esc[i].esc_rpm);
    }

    if (enable_indi_) {
      // Filter RPM with SAME filter as gyro (Smeur 2016: matched phase delay)
      rpm_filtered_ = motor_filter_.update(raw);
      rpm_current_  = rpm_filtered_;
    } else {
      rpm_current_ = raw;
    }

    esc_valid_ = true;
  }

  // ===========================================================================
  // Control loop
  // ===========================================================================
  void controlLoopCallback()
  {
    if (!position_valid_ || !ang_vel_valid_ || !esc_valid_ || !setpoint_received_) return;

    auto  now = this->now();
    float dt  = (now - control_loop_prev_time_).seconds();
    if (dt <= 0.0f || dt > 0.1f) { control_loop_prev_time_ = now; return; }
    control_loop_prev_time_ = now;

    // ── Triple-loop PID cascade (unchanged from working PID code) ────────────
    Eigen::Vector3f vel_cmd    = pos_controller_->update(pos_desired_, pos_current_);
    auto            vel_out    = vel_att_controller_->update(vel_cmd, vel_current_);
    Eigen::Vector3f att_cmd    = vel_out.attitude_cmd;
    float           thrust_cmd = vel_out.thrust_cmd;  // normalized [0,1] collective thrust

    Eigen::Vector3f att_cur;
    att_cur << roll_current_, pitch_current_, 0.0f;
    Eigen::Vector3f rate_cmd = att_rate_controller_->update(att_cmd, att_cur);

    // ── Mode 1: PID-ONLY ─────────────────────────────────────────────────────
    if (!enable_indi_) {
      publishRatesSetpoint(rate_cmd, thrust_cmd);
      last_thrust_cmd_ = thrust_cmd;
      control_loop_count_++;
      return;
    }

    // ── Mode 2: PID-THRUST + INDI-ATTITUDE ───────────────────────────────────
    //
    // Based on Smeur et al. (2016):
    // "Adaptive Incremental Nonlinear Dynamic Inversion for Attitude Control"
    //
    // ARCHITECTURE:
    // ┌─────────────────────────────────────────────────────────────────┐
    // │  COLLECTIVE  (handles altitude/position)                         │
    // │  thrust_cmd → ω²_collective = thrust_cmd × ω²_max              │
    // │  Applied equally to all 4 motors                                 │
    // │                                                                  │
    // │  DIFFERENTIAL  (handles attitude/disturbances)                  │
    // │  α_des, α_meas → INDI → Δω² per motor                          │
    // │  Δω² is zero-mean across motors (pure attitude correction)      │
    // │                                                                  │
    // │  TOTAL:  ω²_i = ω²_collective + Δω²_i                          │
    // │  OUTPUT: motor_i = clamp(ω²_i / ω²_max, 0, 1)                  │
    // └─────────────────────────────────────────────────────────────────┘
    //
    // Why this works:
    //  • Collective is always correct — velocity PID guarantees altitude
    //  • Differential is always zero-mean — INDI can't shift mean (by design)
    //  • They can't interfere with each other
    //  • Motor average ≈ thrust_cmd ≈ hover_thrust ≈ 0.60  ← fixed!

    // Rate PID → desired angular acceleration
    Eigen::Vector3f alpha_des = rate_accel_controller_->update(rate_cmd, ang_vel_filtered_);

    // ── INDI: compute differential Δω² ───────────────────────────────────────
    // G is constant (ω² domain), no singularity, no recomputation
    // Returns Δω² in (rad/s)² — differential only, zero-mean across motors
    Eigen::Vector4f delta_omega_sq = indi_->update(
      alpha_des,            // desired angular acceleration (rad/s²)
      ang_accel_filtered_,  // measured angular acceleration (rad/s²)
      rpm_filtered_);       // passed but unused in ω² domain (API compat)

    // ── COLLECTIVE: thrust_cmd → ω²_collective ───────────────────────────────
    // thrust_cmd is normalized [0,1] from velocity PID
    // This is what drives altitude — same for all 4 motors
    float omega_sq_collective = thrust_cmd * omega_sq_max_;

    // ── MIX: collective + differential ───────────────────────────────────────
    Eigen::Vector4f motor_cmd;
    for (int i = 0; i < 4; i++) {
      float omega_sq_i = omega_sq_collective + delta_omega_sq(i);

      // Hard clamp to physical limits [0, ω²_max]
      omega_sq_i = std::max(0.0f, std::min(omega_sq_max_, omega_sq_i));

      // Normalize to [0,1] for PX4 actuator_motors
      motor_cmd(i) = omega_sq_i / omega_sq_max_;
    }

    publishMotorCommands(motor_cmd);

    // Store for diagnostics
    last_thrust_cmd_         = thrust_cmd;
    last_motor_cmd_          = motor_cmd;
    last_delta_omega_sq_     = delta_omega_sq;
    last_ang_accel_filtered_ = ang_accel_filtered_;

    control_loop_count_++;

    // Debug at ~2 Hz
    if (control_loop_count_ % 50 == 0) {
      float avg = motor_cmd.mean();
      RCLCPP_DEBUG(get_logger(),
        "[INDI] collective=%.3f | motors=[%.3f,%.3f,%.3f,%.3f] avg=%.3f | "
        "Δω²=[%.0f,%.0f,%.0f,%.0f] | α=[%.1f,%.1f,%.1f]rad/s²",
        thrust_cmd, motor_cmd(0), motor_cmd(1), motor_cmd(2), motor_cmd(3), avg,
        delta_omega_sq(0), delta_omega_sq(1), delta_omega_sq(2), delta_omega_sq(3),
        ang_accel_filtered_(0), ang_accel_filtered_(1), ang_accel_filtered_(2));
    }
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

  void publishMotorCommands(const Eigen::Vector4f& cmd)
  {
    auto msg = px4_msgs::msg::ActuatorMotors();
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    for (int i = 0;  i < 4;  i++) msg.control[i] = std::max(0.0f, std::min(1.0f, cmd(i)));
    for (int i = 4;  i < 12; i++) msg.control[i] = NAN;
    pub_actuator_motors_->publish(msg);
  }

  // ===========================================================================
  // Diagnostics
  // ===========================================================================
  void diagnosticsCallback()
  {
    if (!position_valid_ || !ang_vel_valid_ || !esc_valid_ || !setpoint_received_) {
      RCLCPP_WARN(get_logger(), "Waiting for:%s%s%s%s",
        !position_valid_    ? " Position"  : "",
        !ang_vel_valid_     ? " AngVel"    : "",
        !esc_valid_         ? " ESC"       : "",
        !setpoint_received_ ? " Setpoint"  : "");
      return;
    }

    if (enable_indi_) {
      float avg = last_motor_cmd_.mean();
      // Express Δω² as fraction of ω²_max for readability
      Eigen::Vector4f delta_norm = last_delta_omega_sq_ / omega_sq_max_;

      RCLCPP_INFO(get_logger(),
        "[INDI] Pos:[%.2f,%.2f,%.2f]m | "
        "collective:%.3f | "
        "motors:[%.3f,%.3f,%.3f,%.3f] avg:%.3f | "
        "Δω²_norm:[%.3f,%.3f,%.3f,%.3f] | "
        "AngAccel:[%.1f,%.1f,%.1f]rad/s²",
        pos_current_(0), pos_current_(1), pos_current_(2),
        last_thrust_cmd_,
        last_motor_cmd_(0), last_motor_cmd_(1),
        last_motor_cmd_(2), last_motor_cmd_(3), avg,
        delta_norm(0), delta_norm(1), delta_norm(2), delta_norm(3),
        last_ang_accel_filtered_(0),
        last_ang_accel_filtered_(1),
        last_ang_accel_filtered_(2));

      // Sanity checks
      if (std::abs(avg - last_thrust_cmd_) > 0.10f) {
        RCLCPP_WARN(get_logger(),
          "⚠ avg_motor(%.3f) ≠ collective(%.3f) by >10%% — "
          "check INDI differential symmetry", avg, last_thrust_cmd_);
      }
      if (last_thrust_cmd_ < 0.25f || last_thrust_cmd_ > 0.95f) {
        RCLCPP_WARN(get_logger(),
          "⚠ collective=%.3f out of range — check velocity PID / hover_thrust",
          last_thrust_cmd_);
      }
      for (int i = 0; i < 4; i++) {
        if (last_motor_cmd_(i) < 0.05f)
          RCLCPP_WARN(get_logger(), "⚠ Motor%d near zero: %.3f", i, last_motor_cmd_(i));
        if (last_motor_cmd_(i) > 0.95f)
          RCLCPP_WARN(get_logger(), "⚠ Motor%d near max:  %.3f", i, last_motor_cmd_(i));
      }

    } else {
      RCLCPP_INFO(get_logger(),
        "[PID] Pos:[%.2f,%.2f,%.2f]m | Err:[%.3f,%.3f,%.3f]m | "
        "Thrust:%.3f | AngVel:[%.2f,%.2f,%.2f]rad/s",
        pos_current_(0), pos_current_(1), pos_current_(2),
        pos_desired_(0)-pos_current_(0),
        pos_desired_(1)-pos_current_(1),
        pos_desired_(2)-pos_current_(2),
        last_thrust_cmd_,
        ang_vel_current_(0), ang_vel_current_(1), ang_vel_current_(2));
    }
  }

  // ===========================================================================
  // Members
  // ===========================================================================

  bool  enable_indi_;
  bool  enable_adaptation_;

  // x500 rotor limits — update from find_hover_rpm.py if needed
  float omega_max_    = 104.72f;           // rad/s (~15000 RPM)
  float omega_sq_max_ = 104.72f * 104.72f; // (rad/s)²

  static constexpr float kDt_ = 0.004f;   // 250 Hz control loop

  // PID controllers
  std::unique_ptr<PositionController>           pos_controller_;
  std::unique_ptr<VelocityToAttitudeController> vel_att_controller_;
  std::unique_ptr<AttitudeToRateController>     att_rate_controller_;
  std::unique_ptr<RateToAccelController>        rate_accel_controller_;

  // INDI attitude controller
  std::unique_ptr<INDIController> indi_;

  // Matched filters (same cutoff for gyro and RPM — Smeur 2016 requirement)
  Vector3Filter gyro_filter_;
  Vector4Filter motor_filter_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr      sub_pos_setpoint_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr   sub_position_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr        sub_attitude_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr sub_ang_vel_;
  rclcpp::Subscription<px4_msgs::msg::EscStatus>::SharedPtr              sub_esc_status_;

  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr pub_rates_setpoint_;
  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr        pub_actuator_motors_;

  rclcpp::TimerBase::SharedPtr timer_control_loop_;
  rclcpp::TimerBase::SharedPtr timer_diagnostics_;

  // Raw sensor state
  Eigen::Vector3f pos_desired_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f pos_current_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f vel_current_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f ang_vel_raw_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f ang_vel_current_{Eigen::Vector3f::Zero()};
  Eigen::Vector4f rpm_current_{Eigen::Vector4f::Zero()};

  // Filtered state (INDI only)
  Eigen::Vector3f ang_vel_filtered_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f ang_vel_filtered_prev_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f ang_accel_filtered_{Eigen::Vector3f::Zero()};
  Eigen::Vector4f rpm_filtered_{Eigen::Vector4f::Zero()};
  bool            ang_vel_prev_valid_ = false;

  float           roll_current_  = 0.0f;
  float           pitch_current_ = 0.0f;
  float           hover_thrust_  = 0.60f;

  // Diagnostics
  float           last_thrust_cmd_        = 0.0f;
  Eigen::Vector4f last_motor_cmd_         {Eigen::Vector4f::Zero()};
  Eigen::Vector4f last_delta_omega_sq_    {Eigen::Vector4f::Zero()};
  Eigen::Vector3f last_ang_accel_filtered_{Eigen::Vector3f::Zero()};

  bool setpoint_received_ = false;
  bool position_valid_    = false;
  bool attitude_valid_    = false;
  bool ang_vel_valid_     = false;
  bool esc_valid_         = false;

  rclcpp::Time control_loop_prev_time_{0, 0, RCL_ROS_TIME};
  int          control_loop_count_ = 0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdaptiveINDISystem>());
  rclcpp::shutdown();
  return 0;
}