#ifndef PID_CONTROLLERS_PX4_TUNED_HPP
#define PID_CONTROLLERS_PX4_TUNED_HPP

#include <Eigen/Dense>
#include <cmath>

namespace adaptive_indi_system
{

// ============================================================================
// Position Controller - PURE P CONTROL (Outer Loop)
// Converts Position Error → Velocity Setpoints
// ============================================================================
class PositionController
{
public:
  PositionController()
  {
    kp_xy_ = 0.8f;   // Proportional gain for horizontal position
    kp_z_ = 1.2f;    // Proportional gain for vertical position
  }

  Eigen::Vector3f update(
    const Eigen::Vector3f& pos_desired,
    const Eigen::Vector3f& pos_current)
  {
    Eigen::Vector3f pos_error = pos_desired - pos_current;
    
    // Pure P control law: vel_cmd = Kp * pos_error
    Eigen::Vector3f vel_desired;
    vel_desired(0) = kp_xy_ * pos_error(0);
    vel_desired(1) = kp_xy_ * pos_error(1);
    vel_desired(2) = kp_z_ * pos_error(2);
    
    // Saturate velocity commands
    float max_vel_xy = 2.0f;
    float vel_xy_mag = std::sqrt(vel_desired(0)*vel_desired(0) + 
                                  vel_desired(1)*vel_desired(1));
    if (vel_xy_mag > max_vel_xy) {
      float scale = max_vel_xy / vel_xy_mag;
      vel_desired(0) *= scale;
      vel_desired(1) *= scale;
    }
    vel_desired(2) = std::max(-1.0f, std::min(1.0f, vel_desired(2)));
    
    return vel_desired;
  }

  void setGains(float kp_xy, float kp_z) { 
    kp_xy_ = kp_xy; 
    kp_z_ = kp_z;
  }

private:
  float kp_xy_, kp_z_;
};

// ============================================================================
// Velocity-to-Attitude Controller - FULL PID CONTROL (Middle Loop)
// Converts Velocity Error → Attitude Setpoints + Thrust Command
// ============================================================================
class VelocityToAttitudeController
{
public:
  VelocityToAttitudeController()
  {
    // PID gains for horizontal velocity
    kp_vel_xy_ = 0.6f;   // Proportional
    ki_vel_xy_ = 0.08f;  // Integral
    kd_vel_xy_ = 0.15f;  // Derivative (NEW!)
    
    // PID gains for vertical velocity
    kp_vel_z_ = 0.8f;    // Proportional
    ki_vel_z_ = 0.10f;   // Integral
    kd_vel_z_ = 0.12f;   // Derivative (NEW!)
    
    // State tracking
    int_error_vel_xy_.setZero();
    int_error_vel_z_ = 0.0f;
    last_vel_error_xy_.setZero();
    last_vel_error_z_ = 0.0f;
    
    // Anti-windup limits
    max_int_vel_xy_ = 0.3f;
    max_int_vel_z_ = 0.2f;
  }

  struct Output {
    Eigen::Vector3f attitude_cmd;
    float thrust_cmd;
  };

  Output update(
    const Eigen::Vector3f& vel_desired,
    const Eigen::Vector3f& vel_current)
  {
    Output out;
    
    Eigen::Vector3f vel_error = vel_desired - vel_current;
    
    // === INTEGRAL TERM (accumulate with anti-windup) ===
    int_error_vel_xy_(0) += vel_error(0) * 0.004f;  // dt = 1/250Hz
    int_error_vel_xy_(1) += vel_error(1) * 0.004f;
    int_error_vel_z_ += vel_error(2) * 0.004f;
    
    // Clamp integral to prevent windup
    int_error_vel_xy_(0) = std::max(-max_int_vel_xy_, std::min(max_int_vel_xy_, int_error_vel_xy_(0)));
    int_error_vel_xy_(1) = std::max(-max_int_vel_xy_, std::min(max_int_vel_xy_, int_error_vel_xy_(1)));
    int_error_vel_z_ = std::max(-max_int_vel_z_, std::min(max_int_vel_z_, int_error_vel_z_));
    
    // === DERIVATIVE TERM (rate of change of error) ===
    Eigen::Vector2f vel_error_dot_xy = (vel_error.head(2) - last_vel_error_xy_) / 0.004f;
    float vel_error_dot_z = (vel_error(2) - last_vel_error_z_) / 0.004f;
    
    // Update last error for next iteration
    last_vel_error_xy_ = vel_error.head(2);
    last_vel_error_z_ = vel_error(2);
    
    // === HORIZONTAL CONTROL: PID → Attitude Setpoints ===
    // Full PID law: u = Kp*e + Ki*∫e + Kd*de/dt
    Eigen::Vector2f accel_xy = 
      kp_vel_xy_ * vel_error.head(2) + 
      ki_vel_xy_ * int_error_vel_xy_ +
      kd_vel_xy_ * vel_error_dot_xy;
    
    const float gravity = 9.81f;
    out.attitude_cmd(0) = std::atan2(accel_xy(1), gravity);   // Roll
    out.attitude_cmd(1) = -std::atan2(accel_xy(0), gravity);  // Pitch
    out.attitude_cmd(2) = 0.0f;  // Yaw (not controlled in position mode)
    
    // Saturate attitudes to safe limits
    float max_tilt = 30.0f * M_PI / 180.0f;  // 30 degrees max
    out.attitude_cmd(0) = std::max(-max_tilt, std::min(max_tilt, out.attitude_cmd(0)));
    out.attitude_cmd(1) = std::max(-max_tilt, std::min(max_tilt, out.attitude_cmd(1)));
    
    // === VERTICAL CONTROL: PID → Thrust Command ===
    float hover_thrust = 0.70f;
    
    // Full PID law for thrust correction
    float thrust_correction = 
      (kp_vel_z_ * vel_error(2) + 
       ki_vel_z_ * int_error_vel_z_ +
       kd_vel_z_ * vel_error_dot_z) / gravity;
    
    out.thrust_cmd = hover_thrust - thrust_correction;  // NED frame
    
    // Clamp thrust to safe range
    out.thrust_cmd = std::max(0.05f, std::min(0.95f, out.thrust_cmd));
    
    return out;
  }

  void setVelocityGain(float kp_xy, float ki_xy, float kd_xy, float kp_z, float ki_z, float kd_z) { 
    kp_vel_xy_ = kp_xy;
    ki_vel_xy_ = ki_xy;
    kd_vel_xy_ = kd_xy;
    kp_vel_z_ = kp_z;
    ki_vel_z_ = ki_z;
    kd_vel_z_ = kd_z;
  }
  
  void resetIntegral() {
    int_error_vel_xy_.setZero();
    int_error_vel_z_ = 0.0f;
    last_vel_error_xy_.setZero();
    last_vel_error_z_ = 0.0f;
  }

private:
  // PID gains
  float kp_vel_xy_, ki_vel_xy_, kd_vel_xy_;
  float kp_vel_z_, ki_vel_z_, kd_vel_z_;
  
  // State tracking
  Eigen::Vector2f int_error_vel_xy_;      // Integral term (XY)
  float int_error_vel_z_;                 // Integral term (Z)
  Eigen::Vector2f last_vel_error_xy_;     // For derivative (XY)
  float last_vel_error_z_;                // For derivative (Z)
  
  // Anti-windup limits
  float max_int_vel_xy_;
  float max_int_vel_z_;
};

// ============================================================================
// Attitude-to-Rate Controller - FULL PID CONTROL (Inner Loop)
// Converts Attitude Error → Body Rate Setpoints
// ============================================================================
class AttitudeToRateController
{
public:
  AttitudeToRateController()
  {
    // PID gains for roll
    kp_roll_ = 4.0f;
    ki_roll_ = 0.5f;   // NEW!
    kd_roll_ = 0.15f;  // NEW!
    
    // PID gains for pitch
    kp_pitch_ = 4.0f;
    ki_pitch_ = 0.5f;  // NEW!
    kd_pitch_ = 0.15f; // NEW!
    
    // PID gains for yaw
    kp_yaw_ = 2.0f;
    ki_yaw_ = 0.3f;    // NEW!
    kd_yaw_ = 0.10f;   // NEW!
    
    // State tracking
    int_error_att_.setZero();
    last_att_error_.setZero();
    
    // Anti-windup limits
    max_int_att_ = 0.5f;
  }

  Eigen::Vector3f update(
    const Eigen::Vector3f& attitude_desired,
    const Eigen::Vector3f& attitude_current)
  {
    Eigen::Vector3f attitude_error = attitude_desired - attitude_current;
    
    // Normalize yaw error to [-pi, pi]
    attitude_error(2) = std::atan2(std::sin(attitude_error(2)), std::cos(attitude_error(2)));
    
    // === INTEGRAL TERM (accumulate with anti-windup) ===
    int_error_att_ += attitude_error * 0.004f;  // dt = 1/250Hz
    
    // Clamp integral per axis
    for (int i = 0; i < 3; i++) {
      int_error_att_(i) = std::max(-max_int_att_, std::min(max_int_att_, int_error_att_(i)));
    }
    
    // === DERIVATIVE TERM (rate of change of error) ===
    Eigen::Vector3f att_error_dot = (attitude_error - last_att_error_) / 0.004f;
    last_att_error_ = attitude_error;
    
    // === FULL PID LAW: rate_cmd = Kp*e + Ki*∫e + Kd*de/dt ===
    Eigen::Vector3f ang_vel_cmd;
    ang_vel_cmd(0) = kp_roll_  * attitude_error(0) + ki_roll_  * int_error_att_(0) + kd_roll_  * att_error_dot(0);
    ang_vel_cmd(1) = kp_pitch_ * attitude_error(1) + ki_pitch_ * int_error_att_(1) + kd_pitch_ * att_error_dot(1);
    ang_vel_cmd(2) = kp_yaw_   * attitude_error(2) + ki_yaw_   * int_error_att_(2) + kd_yaw_   * att_error_dot(2);
    
    // Saturate rates to safe limits
    float max_rate = 2.0f;  // rad/s
    for (int i = 0; i < 3; i++) {
      ang_vel_cmd(i) = std::max(-max_rate, std::min(max_rate, ang_vel_cmd(i)));
    }
    
    return ang_vel_cmd;
  }

  void setGains(float kp_roll, float ki_roll, float kd_roll,
                float kp_pitch, float ki_pitch, float kd_pitch,
                float kp_yaw, float ki_yaw, float kd_yaw)
  {
    kp_roll_ = kp_roll;
    ki_roll_ = ki_roll;
    kd_roll_ = kd_roll;
    kp_pitch_ = kp_pitch;
    ki_pitch_ = ki_pitch;
    kd_pitch_ = kd_pitch;
    kp_yaw_ = kp_yaw;
    ki_yaw_ = ki_yaw;
    kd_yaw_ = kd_yaw;
  }
  
  void resetIntegral() {
    int_error_att_.setZero();
    last_att_error_.setZero();
  }

private:
  // PID gains
  float kp_roll_, ki_roll_, kd_roll_;
  float kp_pitch_, ki_pitch_, kd_pitch_;
  float kp_yaw_, ki_yaw_, kd_yaw_;
  
  // State tracking
  Eigen::Vector3f int_error_att_;      // Integral term
  Eigen::Vector3f last_att_error_;     // For derivative
  float max_int_att_;                  // Anti-windup limit
};

// ============================================================================
// Rate-to-Acceleration Controller - KEEP AS IS (for INDI mode)
// This controller is only used in INDI mode to generate acceleration commands
// ============================================================================
class RateToAccelController
{
public:
  RateToAccelController()
  {
    kp_rate_ = 2.5f;  // Proportional - converts rate error to acceleration
    kd_rate_ = 0.08f; // Derivative - dampens rate changes (prevents overshoot)
    
    last_rate_error_.setZero();
  }

  Eigen::Vector3f update(
    const Eigen::Vector3f& ang_vel_desired,
    const Eigen::Vector3f& ang_vel_current)
  {
    Eigen::Vector3f ang_vel_error = ang_vel_desired - ang_vel_current;
    
    // Compute derivative (rate of change of error)
    Eigen::Vector3f rate_error_derivative = (ang_vel_error - last_rate_error_) / 0.004f;
    last_rate_error_ = ang_vel_error;
    
    // PD control law: u = Kp*e + Kd*de/dt
    Eigen::Vector3f ang_accel_cmd = kp_rate_ * ang_vel_error + kd_rate_ * rate_error_derivative;
    
    // Saturate - increased for better dynamic response
    float max_accel = 120.0f;
    for (int i = 0; i < 3; i++) {
      ang_accel_cmd(i) = std::max(-max_accel, std::min(max_accel, ang_accel_cmd(i)));
    }
    
    return ang_accel_cmd;
  }

  void setGain(float kp, float kd) { 
    kp_rate_ = kp;
    kd_rate_ = kd;
  }
  
  void reset() {
    last_rate_error_.setZero();
  }

private:
  float kp_rate_, kd_rate_;
  Eigen::Vector3f last_rate_error_;
};

}  // namespace adaptive_indi_system

#endif












// #ifndef PID_CONTROLLERS_PX4_TUNED_HPP
// #define PID_CONTROLLERS_PX4_TUNED_HPP

// #include <Eigen/Dense>
// #include <cmath>

// namespace adaptive_indi_system
// {

// // ============================================================================
// // Position Controller - PURE P CONTROL (PX4 Style)
// // ============================================================================
// class PositionController
// {
// public:
//   PositionController()
//   // old tuning param
//   // {
//   //   kp_xy_ = 0.25f;   // Proportional gain for horizontal
//   //   kp_z_ = 1.2f;     // Proportional gain for vertical
//   // }


//   {
//     kp_xy_ = 0.6f;   // Proportional gain for horizontal
//     kp_z_ = 1.2f;     // Proportional gain for vertical
//   }

//   Eigen::Vector3f update(
//     const Eigen::Vector3f& pos_desired,
//     const Eigen::Vector3f& pos_current)
//   {
//     Eigen::Vector3f pos_error = pos_desired - pos_current;
    
//     // PURE P CONTROL: vel_cmd = Kp * pos_error (PX4 style)
//     Eigen::Vector3f vel_desired;
//     vel_desired(0) = kp_xy_ * pos_error(0);
//     vel_desired(1) = kp_xy_ * pos_error(1);
//     vel_desired(2) = kp_z_ * pos_error(2);
    
//     // Saturate velocity commands
//     float max_vel_xy = 2.0f;
//     float vel_xy_mag = std::sqrt(vel_desired(0)*vel_desired(0) + 
//                                   vel_desired(1)*vel_desired(1));
//     if (vel_xy_mag > max_vel_xy) {
//       float scale = max_vel_xy / vel_xy_mag;
//       vel_desired(0) *= scale;
//       vel_desired(1) *= scale;
//     }
//     vel_desired(2) = std::max(-1.0f, std::min(1.0f, vel_desired(2)));
    
//     return vel_desired;
//   }

//   void setGains(float kp_xy, float ki_xy, float kp_z, float ki_z) { 
//     kp_xy_ = kp_xy; 
//     kp_z_ = kp_z;
//     // ki_xy and ki_z are ignored (pure P controller)
//   }
  
//   void resetIntegral() {
//     // No integral to reset
//   }

// private:
//   float kp_xy_, kp_z_;
// };

// // ============================================================================
// // Velocity-to-Attitude Controller - FULL PID CONTROL (PX4 Style)
// // ============================================================================
// class VelocityToAttitudeController
// {
// public:
//   VelocityToAttitudeController()
//   {
//     // PID gains for horizontal velocity
//     // kp_vel_xy_ = 0.20f;   // Proportional
//     // ki_vel_xy_ = 0.01f;   // Integral (small, for wind rejection)
//     // kd_vel_xy_ = 0.35f;   // Derivative (damping)


//     // PID gains for horizontal velocity
//     kp_vel_xy_ = 0.50f;   // Proportional
//     ki_vel_xy_ = 0.00f;   // Integral (small, for wind rejection)
//     kd_vel_xy_ = 0.0f;   // Derivative (damping)
    
//     // PID gains for vertical velocity
//     kp_vel_z_ = 0.8f;     // Proportional
//     ki_vel_z_ = 0.10f;    // Integral
//     kd_vel_z_ = 0.12f;    // Derivative
    
//     // State tracking
//     int_error_vel_xy_.setZero();
//     int_error_vel_z_ = 0.0f;
//     last_vel_error_xy_.setZero();
//     last_vel_error_z_ = 0.0f;
    
//     // Anti-windup limits
//     max_int_vel_xy_ = 0.3f;
//     max_int_vel_z_ = 0.2f;
//   }

//   struct Output {
//     Eigen::Vector3f attitude_cmd;
//     float thrust_cmd;
//   };

//   Output update(
//     const Eigen::Vector3f& vel_desired,
//     const Eigen::Vector3f& vel_current)
//   {
//     Output out;
    
//     Eigen::Vector3f vel_error = vel_desired - vel_current;
    
//     // === INTEGRAL TERM (with anti-windup) ===
//     int_error_vel_xy_(0) += vel_error(0) * 0.004f;  // dt = 1/250Hz
//     int_error_vel_xy_(1) += vel_error(1) * 0.004f;
//     int_error_vel_z_ += vel_error(2) * 0.004f;
    
//     // Clamp integral
//     int_error_vel_xy_(0) = std::max(-max_int_vel_xy_, std::min(max_int_vel_xy_, int_error_vel_xy_(0)));
//     int_error_vel_xy_(1) = std::max(-max_int_vel_xy_, std::min(max_int_vel_xy_, int_error_vel_xy_(1)));
//     int_error_vel_z_ = std::max(-max_int_vel_z_, std::min(max_int_vel_z_, int_error_vel_z_));
    
//     // === DERIVATIVE TERM ===
//     Eigen::Vector2f vel_error_dot_xy = (vel_error.head(2) - last_vel_error_xy_) / 0.004f;
//     float vel_error_dot_z = (vel_error(2) - last_vel_error_z_) / 0.004f;
    
//     // Update last error
//     last_vel_error_xy_ = vel_error.head(2);
//     last_vel_error_z_ = vel_error(2);
    
//     // === HORIZONTAL CONTROL: PID → Attitude Setpoints ===
//     // Full PID: u = Kp*e + Ki*∫e + Kd*de/dt (PX4 style)
//     Eigen::Vector2f accel_xy = 
//       kp_vel_xy_ * vel_error.head(2) + 
//       ki_vel_xy_ * int_error_vel_xy_ +
//       kd_vel_xy_ * vel_error_dot_xy;
    
//     const float gravity = 9.81f;
//     out.attitude_cmd(0) = std::atan2(accel_xy(1), gravity);   // Roll
//     out.attitude_cmd(1) = -std::atan2(accel_xy(0), gravity);  // Pitch
//     out.attitude_cmd(2) = 0.0f;  // Yaw
    
//     // Saturate attitudes
//     float max_tilt = 30.0f * M_PI / 180.0f;
//     out.attitude_cmd(0) = std::max(-max_tilt, std::min(max_tilt, out.attitude_cmd(0)));
//     out.attitude_cmd(1) = std::max(-max_tilt, std::min(max_tilt, out.attitude_cmd(1)));
    
//     // === VERTICAL CONTROL: PID → Thrust ===
//     float hover_thrust = 0.70f;
    
//     // Full PID for thrust
//     float thrust_correction = 
//       (kp_vel_z_ * vel_error(2) + 
//        ki_vel_z_ * int_error_vel_z_ +
//        kd_vel_z_ * vel_error_dot_z) / gravity;
    
//     out.thrust_cmd = hover_thrust - thrust_correction;  // NED frame
    
//     // Clamp thrust
//     out.thrust_cmd = std::max(0.05f, std::min(0.95f, out.thrust_cmd));
    
//     return out;
//   }

//   void setVelocityGain(float kp_xy, float ki_xy, float kp_z, float ki_z) { 
//     kp_vel_xy_ = kp_xy;
//     ki_vel_xy_ = ki_xy;
//     kp_vel_z_ = kp_z;
//     ki_vel_z_ = ki_z;
//     // Note: kd gains are set internally, not from parameters
//     // This matches PX4 behavior
//   }
  
//   void resetIntegral() {
//     int_error_vel_xy_.setZero();
//     int_error_vel_z_ = 0.0f;
//     last_vel_error_xy_.setZero();
//     last_vel_error_z_ = 0.0f;
//   }

// private:
//   // PID gains
//   float kp_vel_xy_, ki_vel_xy_, kd_vel_xy_;
//   float kp_vel_z_, ki_vel_z_, kd_vel_z_;
  
//   // State tracking
//   Eigen::Vector2f int_error_vel_xy_;      // Integral term (XY)
//   float int_error_vel_z_;                 // Integral term (Z)
//   Eigen::Vector2f last_vel_error_xy_;     // For derivative (XY)
//   float last_vel_error_z_;                // For derivative (Z)
  
//   // Anti-windup limits
//   float max_int_vel_xy_;
//   float max_int_vel_z_;
// };

// // ============================================================================
// // Attitude-to-Rate Controller - PURE P CONTROL (PX4 Style)
// // ============================================================================
// class AttitudeToRateController
// {
// public:
//   AttitudeToRateController()
//   // old tuning param
//   // {
//   //   kp_roll_ = 1.5f;    // Proportional gain for roll
//   //   kp_pitch_ = 1.5f;   // Proportional gain for pitch
//   //   kp_yaw_ = 0.5f;     // Proportional gain for yaw
//   // }


//   {
//     kp_roll_ = 1.5f;    // Proportional gain for roll
//     kp_pitch_ = 1.5f;   // Proportional gain for pitch
//     kp_yaw_ = 0.5f;     // Proportional gain for yaw
//   }

//   Eigen::Vector3f update(
//     const Eigen::Vector3f& attitude_desired,
//     const Eigen::Vector3f& attitude_current)
//   {
//     Eigen::Vector3f attitude_error = attitude_desired - attitude_current;
    
//     // Normalize yaw error to [-pi, pi]
//     attitude_error(2) = std::atan2(std::sin(attitude_error(2)), std::cos(attitude_error(2)));
    
//     // PURE P CONTROL: rate_cmd = Kp * att_error (PX4 style)
//     Eigen::Vector3f ang_vel_cmd;
//     ang_vel_cmd(0) = kp_roll_ * attitude_error(0);
//     ang_vel_cmd(1) = kp_pitch_ * attitude_error(1);
//     ang_vel_cmd(2) = kp_yaw_ * attitude_error(2);
    
//     // Saturate rates
//     float max_rate = 2.0f;  // rad/s
//     for (int i = 0; i < 3; i++) {
//       ang_vel_cmd(i) = std::max(-max_rate, std::min(max_rate, ang_vel_cmd(i)));
//     }
    
//     return ang_vel_cmd;
//   }

//   void setGains(float kp_roll, float kp_pitch, float kp_yaw)
//   {
//     kp_roll_ = kp_roll;
//     kp_pitch_ = kp_pitch;
//     kp_yaw_ = kp_yaw;
//   }
  
//   void resetIntegral() {
//     // No integral to reset
//   }

// private:
//   float kp_roll_, kp_pitch_, kp_yaw_;
// };

// // ============================================================================
// // Rate-to-Acceleration Controller - PD CONTROL (for INDI mode)
// // ============================================================================
// class RateToAccelController
// {
// public:
//   RateToAccelController()
//   {
//     kp_rate_ = 2.5f;  // Proportional
//     kd_rate_ = 0.08f; // Derivative
    
//     last_rate_error_.setZero();
//   }

//   Eigen::Vector3f update(
//     const Eigen::Vector3f& ang_vel_desired,
//     const Eigen::Vector3f& ang_vel_current)
//   {
//     Eigen::Vector3f ang_vel_error = ang_vel_desired - ang_vel_current;
    
//     // Compute derivative
//     Eigen::Vector3f rate_error_derivative = (ang_vel_error - last_rate_error_) / 0.004f;
//     last_rate_error_ = ang_vel_error;
    
//     // PD control: u = Kp*e + Kd*de/dt
//     Eigen::Vector3f ang_accel_cmd = kp_rate_ * ang_vel_error + kd_rate_ * rate_error_derivative;
    
//     // Saturate
//     float max_accel = 120.0f;
//     for (int i = 0; i < 3; i++) {
//       ang_accel_cmd(i) = std::max(-max_accel, std::min(max_accel, ang_accel_cmd(i)));
//     }
    
//     return ang_accel_cmd;
//   }

//   void setGain(float kp, float kd) { 
//     kp_rate_ = kp;
//     kd_rate_ = kd;
//   }
  
//   void reset() {
//     last_rate_error_.setZero();
//   }

// private:
//   float kp_rate_, kd_rate_;
//   Eigen::Vector3f last_rate_error_;
// };

// }  // namespace adaptive_indi_system

// #endif
