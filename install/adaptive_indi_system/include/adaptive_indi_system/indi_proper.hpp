// #ifndef INDI_PROPER_HPP
// #define INDI_PROPER_HPP

// /**
//  * @file indi_proper.hpp
//  * @brief PROPER Incremental Nonlinear Dynamic Inversion (INDI) Controller
//  *
//  * Theory Reference: Adaptive INDI for MAVs (Paper by Sieberling et al.)
//  *
//  * Key differences from naive implementation:
//  * 1. G matrix is DYNAMIC (depends on rotor speeds via 2ω term)
//  * 2. Works in ω² (rotor squared-speed) domain, NOT PWM
//  * 3. Proper pseudo-inverse computation per update
//  * 4. Includes rotor dynamics compensation
//  * 5. Gyro-based angular acceleration estimation
//  */

// #include <Eigen/Dense>
// #include <cmath>
// #include <array>
// #include <algorithm>

// namespace adaptive_indi_system
// {
// // Note: ButterworthLPF and Vector3LPF are defined in controllers.hpp
// // We reuse them here to avoid duplication

// // ============================================================================
// // PROPER INDI CONTROLLER - Following Theory Exactly
// // ============================================================================
// class INDIController
// {
// public:
//   INDIController(
//     float Ixx = 0.003f,
//     float Iyy = 0.003f,
//     float Izz = 0.005f,
//     float arm_length_x = 0.16f,  // l (pitch moment arm)
//     float arm_length_y = 0.10f,  // b (roll moment arm)
//     float thrust_coeff = 1e-5f,  // k1
//     float yaw_coeff = 1e-5f,     // k2
//     float rotor_inertia = 5e-5f, // Ir_z
//     float sample_rate_hz = 250.0f,
//     float hover_thrust = 0.55f
//   )
//     : Ixx_(Ixx), Iyy_(Iyy), Izz_(Izz),
//       l_(arm_length_x), b_(arm_length_y),
//       k1_(thrust_coeff), k2_(yaw_coeff),
//       Ir_z_(rotor_inertia),
//       fs_(sample_rate_hz),
//       Ts_(1.0f / sample_rate_hz),
//       hover_thrust_(hover_thrust)
//   {
//     // Build constant geometry matrix B
//     buildGeometryMatrix();
    
//     // Build inertia inverse
//     Iv_inv_.setZero();
//     Iv_inv_(0, 0) = 1.0f / Ixx_;
//     Iv_inv_(1, 1) = 1.0f / Iyy_;
//     Iv_inv_(2, 2) = 1.0f / Izz_;
    
//     // Initialize filters for angular acceleration estimation
//     ang_accel_filter_ = std::make_unique<Vector3LPF>(50.0f, fs_);  // Increased to 50 Hz for smoother corrections
    
//     // Initialize rotor speed filters (for gain scheduling in G matrix)
//     for (int i = 0; i < 4; i++) {
//       rotor_speed_filters_[i] = std::make_unique<ButterworthLPF>(40.0f, fs_);  // Increased to 40 Hz for better gain scheduling
//     }
    
//     // Initialize state
//     reset();
//   }

//   /**
//    * @brief Update INDI controller
//    *
//    * @param ang_accel_des [3x1] Desired angular acceleration (rad/s²) from outer loop
//    * @param ang_vel_meas [3x1]  Measured angular velocity (rad/s) from gyro
//    * @param rpm_meas [4x1]      Measured motor RPMs
//    *
//    * @return [4x1] Motor command increments (Δω²) in normalized units [0, 1]
//    */
//   Eigen::Vector4f update(
//     const Eigen::Vector3f& ang_accel_des,
//     const Eigen::Vector3f& ang_vel_meas,
//     const Eigen::Vector4f& rpm_meas)
//   {
//     // =====================================================================
//     // Step 1: Estimate angular acceleration from gyro
//     // =====================================================================
//     Eigen::Vector3f ang_accel_meas;
//     if (first_run_) {
//       ang_accel_meas.setZero();
//       ang_vel_prev_ = ang_vel_meas;
//       first_run_ = false;
//     } else {
//       // Raw differentiation
//       ang_accel_meas = (ang_vel_meas - ang_vel_prev_) / Ts_;
//       ang_vel_prev_ = ang_vel_meas;
//     }
    
//     // Filter the acceleration estimate to reduce noise
//     Eigen::Vector3f ang_accel_filt = ang_accel_filter_->update(ang_accel_meas);
    
//     // =====================================================================
//     // Step 2: Convert RPM to rotor angular speeds (rad/s)
//     // =====================================================================
//     Eigen::Vector4f omega_rad = rpm_meas * (2.0f * M_PI / 60.0f);
    
//     // Filter rotor speeds (for gain scheduling - delay compensation)
//     Eigen::Vector4f omega_filt;
//     for (int i = 0; i < 4; i++) {
//       omega_filt(i) = rotor_speed_filters_[i]->update(omega_rad(i));
//     }
    
//     // =====================================================================
//     // Step 3: Compute DYNAMIC G1 matrix (THIS IS KEY!)
//     // =====================================================================
//     // G1 = I_v^{-1} * B * 2*ω_filtered
//     // Shape: 3x4
//     Eigen::Matrix<float, 4, 4> omega_diag = omega_filt.asDiagonal();
//     Eigen::Matrix<float, 3, 4> G1 = Iv_inv_ * B_ * 2.0f * omega_diag;
    
//     // =====================================================================
//     // Step 4: Compute control error
//     // =====================================================================
//     Eigen::Vector3f accel_error = ang_accel_des - ang_accel_filt;
    
//     // Saturate error to prevent extreme control actions
//     float max_error = 30.0f;  // rad/s²
//     for (int i = 0; i < 3; i++) {
//       accel_error(i) = std::max(-max_error, std::min(max_error, accel_error(i)));
//     }
    
//     // =====================================================================
//     // Step 5: Compute pseudo-inverse of G1
//     // =====================================================================
//     // G1^+ = G1^T * (G1 * G1^T + λI)^{-1}
//     Eigen::Matrix3f GGt = G1 * G1.transpose();
    
//     // Regularization to ensure invertibility (increased for stability)
//     float lambda = 5e-3f;  // Increased from 1e-3 to prevent extreme corrections
//     GGt += lambda * Eigen::Matrix3f::Identity();
    
//     Eigen::Matrix<float, 4, 3> G1_pinv = G1.transpose() * GGt.inverse();
    
//     // =====================================================================
//     // Step 6: INDI control law - compute motor command increments
//     // =====================================================================
//     // Δω²_i = G1^+ * (ν_des - ν_meas)
//     Eigen::Vector4f motor_increment = G1_pinv * accel_error;
    
//     // =====================================================================
//     // Step 7: Saturate motor increments (CONSERVATIVE for stability)
//     // =====================================================================
//     float max_increment = 0.05f;  // Reduced from 0.15 to prevent violent corrections
//     for (int i = 0; i < 4; i++) {
//       motor_increment(i) = std::max(-max_increment, 
//                                       std::min(max_increment, motor_increment(i)));
//     }
    
//     // Store state for next iteration
//     last_ang_accel_error_ = accel_error;
//     last_motor_increment_ = motor_increment;
    
//     return motor_increment;
//   }

//   void reset()
//   {
//     first_run_ = true;
//     ang_vel_prev_.setZero();
//     ang_accel_filter_->reset();
    
//     for (int i = 0; i < 4; i++) {
//       rotor_speed_filters_[i]->reset();
//     }
    
//     last_ang_accel_error_.setZero();
//     last_motor_increment_.setZero();
//   }

//   // Getters for debugging
//   Eigen::Vector3f getLastAccelError() const { return last_ang_accel_error_; }
//   Eigen::Vector4f getLastMotorIncrement() const { return last_motor_increment_; }
//   float getHoverThrust() const { return hover_thrust_; }

// private:
//   void buildGeometryMatrix()
//   {
//     // B matrix: geometry for the quadrotor
//     // Each column corresponds to one motor
//     // Row 0 (roll):  b*k1*(-1,+1,+1,-1)
//     // Row 1 (pitch): l*k1*(+1,+1,-1,-1)
//     // Row 2 (yaw):   k2*(+1,-1,+1,-1)
//     //
//     // For X configuration:
//     // Motor 0: front-right
//     // Motor 1: back-left
//     // Motor 2: front-left
//     // Motor 3: back-right
    
//     B_.setZero();
//     B_(0, 0) = -b_ * k1_;  // Motor 0 roll contribution
//     B_(0, 1) =  b_ * k1_;  // Motor 1
//     B_(0, 2) =  b_ * k1_;  // Motor 2
//     B_(0, 3) = -b_ * k1_;  // Motor 3
    
//     B_(1, 0) =  l_ * k1_;  // Motor 0 pitch contribution
//     B_(1, 1) =  l_ * k1_;  // Motor 1
//     B_(1, 2) = -l_ * k1_;  // Motor 2
//     B_(1, 3) = -l_ * k1_;  // Motor 3
    
//     B_(2, 0) =  k2_;       // Motor 0 yaw contribution
//     B_(2, 1) = -k2_;       // Motor 1
//     B_(2, 2) =  k2_;       // Motor 2
//     B_(2, 3) = -k2_;       // Motor 3
//   }

//   // Physical parameters
//   float Ixx_, Iyy_, Izz_;
//   float l_, b_;          // Moment arms
//   float k1_, k2_;        // Thrust and yaw coefficients
//   float Ir_z_;           // Rotor inertia (for future use)
//   float fs_;             // Sample rate (Hz)
//   float Ts_;             // Sample time (seconds)
//   float hover_thrust_;
  
//   // Constant geometry
//   Eigen::Matrix<float, 3, 4> B_;
//   Eigen::Matrix3f Iv_inv_;  // I^{-1}
  
//   // Filters
//   std::unique_ptr<Vector3LPF> ang_accel_filter_;
//   std::array<std::unique_ptr<ButterworthLPF>, 4> rotor_speed_filters_;
  
//   // State
//   bool first_run_;
//   Eigen::Vector3f ang_vel_prev_;
  
//   // Debug/monitoring
//   Eigen::Vector3f last_ang_accel_error_;
//   Eigen::Vector4f last_motor_increment_;
// };

// } // namespace adaptive_indi_system

// #endif










#ifndef INDI_PROPER_HPP
#define INDI_PROPER_HPP

/**
 * @file indi_proper.hpp
 *
 * INDI Attitude Controller — Smeur et al. 2016 formulation
 * "Adaptive Incremental Nonlinear Dynamic Inversion for Attitude Control of MAVs"
 *
 * =========================================================================
 * CORRECT DOMAIN: ω² (rotor speed squared)
 * =========================================================================
 *
 * Why ω² and NOT ω?
 *   Thrust: T_i = k1 * ω_i²  (linear in ω²)
 *   Torque: τ   = B * ω²     (linear in ω²)
 *
 *   So: angular_accel = I⁻¹ * B * ω²  = G * ω²
 *       where G = I⁻¹ * B  (CONSTANT 3×4 matrix — no RPM dependency!)
 *
 *   INDI law in ω² domain:
 *     Δω² = G⁺ * (α_des − α_meas)
 *
 * Benefits over ω domain:
 *   ✓ G is CONSTANT — computed once in constructor, never recomputed
 *   ✓ NO singularity at zero RPM (G doesn't depend on current ω)
 *   ✓ Physically correct — thrust and torque are linear in ω²
 *   ✓ Increments add correctly: ω²_new = ω²_old + Δω²
 *
 * =========================================================================
 * WHAT THIS CLASS DOES:
 * =========================================================================
 *   Input:  α_des  = desired angular acceleration (rad/s²) [from rate PID]
 *           α_meas = measured angular acceleration (rad/s²) [filtered IMU]
 *
 *   Output: Δω² per motor (rad/s)²  [DIFFERENTIAL only, zero-mean ≈ 0]
 *
 * The caller (main controller) handles:
 *   - COLLECTIVE thrust from velocity PID
 *   - Adding collective + differential
 *   - Normalizing to [0,1] for PX4
 *
 * This separation is exactly the Smeur 2016 architecture:
 *   "Control of this fourth axis (thrust) is handled by a separate controller.
 *    This controller scales the average motor speed."
 */

#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace adaptive_indi_system
{

class INDIController
{
public:
  /**
   * @param Ixx, Iyy, Izz  Body moments of inertia (kg·m²)
   * @param arm_length_x    Fore-aft motor arm (m)  — CA_ROTOR0_PX
   * @param arm_length_y    Lateral motor arm (m)   — avg |CA_ROTORx_PY|
   * @param thrust_coeff    k1: T = k1*ω²  (N/(rad/s)²)
   * @param yaw_coeff       k2: τz = k2*ω² (N·m/(rad/s)²)
   * @param omega_max       Maximum rotor speed (rad/s) — e.g. 1571 = 15000 RPM
   */
  INDIController(
    float Ixx          = 0.029125f,
    float Iyy          = 0.029125f,
    float Izz          = 0.055225f,
    float arm_length_x = 0.13f,
    float arm_length_y = 0.21f,
    float thrust_coeff = 1e-5f,
    float yaw_coeff    = 1e-5f,
    float rotor_inertia = 5e-5f,   // reserved
    float sample_rate_hz = 250.0f, // reserved
    float omega_max    = 1571.0f
  )
    : Ixx_(Ixx), Iyy_(Iyy), Izz_(Izz),
      l_(arm_length_x), b_(arm_length_y),
      k1_(thrust_coeff), k2_(yaw_coeff),
      omega_max_(omega_max),
      omega_sq_max_(omega_max * omega_max)
  {
    (void)rotor_inertia;
    (void)sample_rate_hz;

    // Inertia inverse
    Iv_inv_.setZero();
    Iv_inv_(0,0) = 1.0f / Ixx_;
    Iv_inv_(1,1) = 1.0f / Iyy_;
    Iv_inv_(2,2) = 1.0f / Izz_;

    // Build geometry matrix B  (3×4, maps ω² → body torques)
    buildGeometryMatrix();

    // G = I⁻¹ · B  (3×4, maps Δω² → angular acceleration)
    // This is CONSTANT — computed once here, never recomputed!
    G_ = Iv_inv_ * B_;

    // Precompute pseudo-inverse: G⁺ = Gᵀ(GGᵀ + λI)⁻¹
    // Also constant — no inversion needed at runtime!
    Eigen::Matrix3f GGt = G_ * G_.transpose();
    GGt += kLambda_ * Eigen::Matrix3f::Identity();
    G_pinv_ = G_.transpose() * GGt.inverse();

    reset();
  }

  /**
   * @brief INDI update — returns DIFFERENTIAL Δω² per motor.
   *
   * @param alpha_des  Desired angular acceleration (rad/s²) from rate PID
   * @param alpha_meas Filtered angular acceleration (rad/s²) from gyro diff
   *
   * @return Δω² per motor (rad/s)² — DIFFERENTIAL only (zero-mean ≈ 0)
   *         Caller adds to ω²_collective before normalizing.
   *
   * Note: rpm_meas is NOT needed in ω² domain because G is constant.
   *       It is accepted here only for API compatibility / future use.
   */
  Eigen::Vector4f update(
    const Eigen::Vector3f& alpha_des,
    const Eigen::Vector3f& alpha_meas,
    const Eigen::Vector4f& /* rpm_meas — unused in ω² domain */ )
  {
    // Angular acceleration error
    Eigen::Vector3f alpha_error = alpha_des - alpha_meas;

    // Clamp error to prevent extreme outputs on startup / sensor spikes
    const float kMaxError = 50.0f;  // rad/s²
    for (int i = 0; i < 3; i++) {
      alpha_error(i) = std::max(-kMaxError, std::min(kMaxError, alpha_error(i)));
    }

    // INDI law: Δω² = G⁺ · (α_des − α_meas)
    // Units: (rad/s)²
    Eigen::Vector4f delta_omega_sq = G_pinv_ * alpha_error;

    // Clamp each motor's Δω² to ±20% of ω²_max
    // This limits how much INDI can correct in one step
    const float kMaxDeltaSq = 0.20f * omega_sq_max_;
    for (int i = 0; i < 4; i++) {
      delta_omega_sq(i) = std::max(-kMaxDeltaSq,
                                    std::min(kMaxDeltaSq, delta_omega_sq(i)));
    }

    last_alpha_error_ = alpha_error;
    last_delta_omega_sq_ = delta_omega_sq;

    return delta_omega_sq;
  }

  void reset()
  {
    last_alpha_error_.setZero();
    last_delta_omega_sq_.setZero();
  }

  // Getters for diagnostics
  Eigen::Vector3f getLastAlphaError()    const { return last_alpha_error_; }
  Eigen::Vector4f getLastDeltaOmegaSq() const { return last_delta_omega_sq_; }
  // Legacy alias
  Eigen::Vector4f getLastMotorIncrement() const { return last_delta_omega_sq_; }

  Eigen::Matrix<float, 3, 4> getG()     const { return G_; }
  Eigen::Matrix<float, 4, 3> getGpinv() const { return G_pinv_; }
  float getOmegaMax()                   const { return omega_max_; }
  float getOmegaSqMax()                 const { return omega_sq_max_; }

private:
  /**
   * Build B matrix: body torques = B * [ω1², ω2², ω3², ω4²]ᵀ
   *
   * PX4 gz_x500 layout (verified from airframe 4001_gz_x500):
   *
   *        +X (Front)
   *   FL(CW)     FR(CCW)
   *    M2           M0
   *       \       /
   *        \     /
   *    M1       M3
   *   BL(CCW)  BR(CW)
   *        -X (Back)
   *
   *   Left = −Y,  Right = +Y
   *
   * Sign conventions (verified correct by simulation, Smeur 2016):
   *   Roll  (+p): Left motors ↑, Right motors ↓  → [−,+,+,−] * b*k1
   *   Pitch (+q): Front motors ↑, Back motors ↓  → [+,−,+,−] * l*k1
   *   Yaw   (+r): CCW motors ↑, CW motors ↓      → [+,+,−,−] * k2
   *
   * Units of B: N·m / (rad/s)²
   */
  void buildGeometryMatrix()
  {
    B_.setZero();

    // Roll torque (τx):
    B_(0,0) = -b_ * k1_;  // M0: FR, right  → −
    B_(0,1) =  b_ * k1_;  // M1: BL, left   → +
    B_(0,2) =  b_ * k1_;  // M2: FL, left   → +
    B_(0,3) = -b_ * k1_;  // M3: BR, right  → −

    // Pitch torque (τy):
    B_(1,0) =  l_ * k1_;  // M0: FR, front  → +
    B_(1,1) = -l_ * k1_;  // M1: BL, back   → −
    B_(1,2) =  l_ * k1_;  // M2: FL, front  → +
    B_(1,3) = -l_ * k1_;  // M3: BR, back   → −

    // Yaw torque (τz):
    B_(2,0) =  k2_;        // M0: FR, CCW    → +
    B_(2,1) =  k2_;        // M1: BL, CCW    → +
    B_(2,2) = -k2_;        // M2: FL, CW     → −
    B_(2,3) = -k2_;        // M3: BR, CW     → −
  }

  // Physical parameters
  float Ixx_, Iyy_, Izz_;
  float l_, b_, k1_, k2_;
  float omega_max_, omega_sq_max_;

  // Constant matrices (computed once in constructor)
  Eigen::Matrix3f             Iv_inv_;   // I⁻¹
  Eigen::Matrix<float, 3, 4> B_;        // Geometry (ω² → torque)
  Eigen::Matrix<float, 3, 4> G_;        // Control effectiveness (ω² → α)
  Eigen::Matrix<float, 4, 3> G_pinv_;   // Pseudo-inverse of G

  // Regularization for pseudo-inverse
  static constexpr float kLambda_ = 1e-4f;

  // Debug state
  Eigen::Vector3f last_alpha_error_;
  Eigen::Vector4f last_delta_omega_sq_;
};

}  // namespace adaptive_indi_system
#endif  // INDI_PROPER_HPP