#ifndef ADAPTIVE_INDI_CONSERVATIVE_HPP
#define ADAPTIVE_INDI_CONSERVATIVE_HPP

#include <Eigen/Dense>
#include <cmath>

namespace adaptive_indi_system
{

// ============================================================================
// Butterworth Filter (same as before)
// ============================================================================
class ButterworthLPF
{
public:
  ButterworthLPF(float cutoff_freq_hz = 50.0f, float sample_rate_hz = 250.0f)
  {
    setCutoffFrequency(cutoff_freq_hz, sample_rate_hz);
    reset();
  }

  void setCutoffFrequency(float cutoff_freq_hz, float sample_rate_hz)
  {
    float wc = 2.0f * M_PI * cutoff_freq_hz / sample_rate_hz;
    float sqrt2 = std::sqrt(2.0f);
    float c = 1.0f / std::tan(wc / 2.0f);
    float c_sq = c * c;
    float norm = 1.0f / (1.0f + sqrt2 * c + c_sq);
    
    b0_ = norm;
    b1_ = 2.0f * norm;
    b2_ = norm;
    a1_ = 2.0f * (1.0f - c_sq) * norm;
    a2_ = (1.0f - sqrt2 * c + c_sq) * norm;
  }

  float update(float input)
  {
    float output = b0_ * input + z1_;
    z1_ = b1_ * input - a1_ * output + z2_;
    z2_ = b2_ * input - a2_ * output;
    return output;
  }

  void reset()
  {
    z1_ = 0.0f;
    z2_ = 0.0f;
  }

private:
  float b0_, b1_, b2_;
  float a1_, a2_;
  float z1_, z2_;
};

class Vector3LPF
{
public:
  Vector3LPF(float cutoff_hz = 50.0f, float sample_rate_hz = 250.0f)
    : filter_x_(cutoff_hz, sample_rate_hz),
      filter_y_(cutoff_hz, sample_rate_hz),
      filter_z_(cutoff_hz, sample_rate_hz)
  {
  }

  Eigen::Vector3f update(const Eigen::Vector3f& input)
  {
    Eigen::Vector3f output;
    output(0) = filter_x_.update(input(0));
    output(1) = filter_y_.update(input(1));
    output(2) = filter_z_.update(input(2));
    return output;
  }

  void reset()
  {
    filter_x_.reset();
    filter_y_.reset();
    filter_z_.reset();
  }

private:
  ButterworthLPF filter_x_, filter_y_, filter_z_;
};

// ============================================================================
// CONSERVATIVE INDI Controller - Much Lower Gains!
// ============================================================================
class AdaptiveINDI
{
public:
  AdaptiveINDI(
    float Ixx = 0.003f,
    float Iyy = 0.003f,
    float Izz = 0.005f,
    float l = 0.16f,
    float b = 0.10f,
    float k1 = 0.00001f,
    float k2 = 0.00001f,
    float Irzz = 0.00005f,
    float Ts = 0.004f,
    float hover_thrust = 0.6f
  )
    : Ixx_(Ixx), Iyy_(Iyy), Izz_(Izz),
      l_(l), b_(b), k1_(k1), k2_(k2),
      Irzz_(Irzz), Ts_(Ts),
      hover_thrust_(hover_thrust)
  {
    computeG1Matrix();
    computeG2Matrix();
    computeControlAllocation();
    
    // MUCH MORE aggressive filtering for stability
    ang_accel_filter_ = std::make_unique<Vector3LPF>(20.0f, 250.0f);  // Was 30Hz
    
    motor_increment_prev_.setZero();
    ang_accel_prev_.setZero();
    first_run_ = true;
    
    reset();
  }

  Eigen::Vector4f update(
    const Eigen::Vector3f& ang_accel_desired,
    const Eigen::Vector3f& ang_vel_measured,
    const Eigen::Vector4f& /* motor_rpm */)  // Not used in old implementation
  {
    // =====================================================================
    // Step 1: Estimate angular acceleration
    // =====================================================================
    Eigen::Vector3f ang_accel_measured;
    if (first_run_) {
      ang_accel_measured.setZero();
      ang_vel_prev_ = ang_vel_measured;
      first_run_ = false;
    } else {
      ang_accel_measured = (ang_vel_measured - ang_vel_prev_) / Ts_;
      ang_vel_prev_ = ang_vel_measured;
    }
    
    // Heavy filtering
    Eigen::Vector3f ang_accel_filtered = ang_accel_filter_->update(ang_accel_measured);
    
    // =====================================================================
    // Step 2: Compute control error
    // =====================================================================
    Eigen::Vector3f ang_accel_error = ang_accel_desired - ang_accel_filtered;
    
    // MUCH tighter saturation
    float max_error = 20.0f;  // Was 50, now 20 rad/s²
    for (int i = 0; i < 3; i++) {
      ang_accel_error(i) = std::max(-max_error, std::min(max_error, ang_accel_error(i)));
    }
    
    // =====================================================================
    // Step 3: INDI Control Law - VERY CONSERVATIVE!
    // =====================================================================
    Eigen::Vector4f motor_increment = G_inv_ * ang_accel_error;
    
    // CRITICAL FIX: Much lower gain!
    float increment_gain = 0.15f;  // Was 0.5, now 0.15!
    motor_increment *= increment_gain;
    
    // CRITICAL FIX: Much smaller max increment!
    float max_increment = 0.03f;  // Was 0.15, now 0.03!
    for (int i = 0; i < 4; i++) {
      motor_increment(i) = std::max(-max_increment, std::min(max_increment, motor_increment(i)));
    }
    
    // Store for next iteration
    motor_increment_prev_ = motor_increment;
    ang_accel_prev_ = ang_accel_filtered;
    
    return motor_increment;
  }

  void reset()
  {
    motor_increment_prev_.setZero();
    ang_vel_prev_.setZero();
    ang_accel_prev_.setZero();
    ang_accel_filter_->reset();
    first_run_ = true;
  }

  void setLearningRates(float /* mu1 */, float /* mu2 */) {}  // Not used

  Eigen::Matrix<float, 3, 4> getG1() const { return G1_; }
  Eigen::Matrix<float, 3, 4> getG2() const { return G2_; }

private:
  void computeG1Matrix()
  {
    Eigen::Matrix<float, 3, 4> thrust_matrix;
    thrust_matrix <<
      -b_ * k1_,  b_ * k1_,  b_ * k1_, -b_ * k1_,
       l_ * k1_,  l_ * k1_, -l_ * k1_, -l_ * k1_,
       k2_,      -k2_,       k2_,      -k2_;
    
    Eigen::Matrix3f Iv_inv;
    Iv_inv.setZero();
    Iv_inv(0, 0) = 1.0f / Ixx_;
    Iv_inv(1, 1) = 1.0f / Iyy_;
    Iv_inv(2, 2) = 1.0f / Izz_;
    
    G1_ = 2.0f * Iv_inv * thrust_matrix;
  }

  void computeG2Matrix()
  {
    Eigen::Matrix<float, 3, 4> rotor_matrix;
    rotor_matrix.setZero();
    rotor_matrix(2, 0) =  Irzz_;
    rotor_matrix(2, 1) = -Irzz_;
    rotor_matrix(2, 2) =  Irzz_;
    rotor_matrix(2, 3) = -Irzz_;
    
    Eigen::Matrix3f Iv_inv;
    Iv_inv.setZero();
    Iv_inv(0, 0) = 1.0f / Ixx_;
    Iv_inv(1, 1) = 1.0f / Iyy_;
    Iv_inv(2, 2) = 1.0f / Izz_;
    
    G2_ = Iv_inv * (1.0f / Ts_) * rotor_matrix;
  }

  void computeControlAllocation()
  {
    Eigen::Matrix<float, 3, 4> G_total = G1_;
    Eigen::Matrix3f AAT = G_total * G_total.transpose();
    AAT += 0.01f * Eigen::Matrix3f::Identity();
    G_inv_ = G_total.transpose() * AAT.inverse();
  }

  float Ixx_, Iyy_, Izz_;
  float l_, b_;
  float k1_, k2_;
  float Irzz_;
  float Ts_;
  float hover_thrust_;
  
  Eigen::Matrix<float, 3, 4> G1_;
  Eigen::Matrix<float, 3, 4> G2_;
  Eigen::Matrix<float, 4, 3> G_inv_;
  
  Eigen::Vector4f motor_increment_prev_;
  Eigen::Vector3f ang_vel_prev_;
  Eigen::Vector3f ang_accel_prev_;
  
  std::unique_ptr<Vector3LPF> ang_accel_filter_;
  
  bool first_run_;
};

} // namespace adaptive_indi_system

#endif



// #ifndef ADAPTIVE_INDI_CONTROLLER_HPP
// #define ADAPTIVE_INDI_CONTROLLER_HPP

// #include <Eigen/Dense>
// #include <cmath>
// #include <algorithm>
// #include <array>

// namespace adaptive_indi_system
// {

// // ============================================================================
// // Simple 2nd-order Butterworth LPF (biquad) in Direct Form II Transposed
// // ============================================================================
// class ButterworthLPF
// {
// public:
//   ButterworthLPF(float cutoff_hz = 20.0f, float sample_rate_hz = 250.0f)
//   {
//     configure(cutoff_hz, sample_rate_hz);
//     reset();
//   }

//   void configure(float cutoff_hz, float sample_rate_hz)
//   {
//     // Bilinear transform of analog 2nd-order Butterworth
//     // This is a standard biquad design for LPF.
//     const float w0 = 2.0f * static_cast<float>(M_PI) * (cutoff_hz / sample_rate_hz);
//     const float cosw0 = std::cos(w0);
//     const float sinw0 = std::sin(w0);
//     const float Q = std::sqrt(2.0f) / 2.0f; // Butterworth Q = 1/sqrt(2)
//     const float alpha = sinw0 / (2.0f * Q);

//     float b0 = (1.0f - cosw0) * 0.5f;
//     float b1 = 1.0f - cosw0;
//     float b2 = (1.0f - cosw0) * 0.5f;

//     float a0 = 1.0f + alpha;
//     float a1 = -2.0f * cosw0;
//     float a2 = 1.0f - alpha;

//     // normalize
//     b0_ = b0 / a0;
//     b1_ = b1 / a0;
//     b2_ = b2 / a0;
//     a1_ = a1 / a0;
//     a2_ = a2 / a0;
//   }

//   float update(float x)
//   {
//     // DF2T
//     const float y = b0_ * x + z1_;
//     z1_ = b1_ * x - a1_ * y + z2_;
//     z2_ = b2_ * x - a2_ * y;
//     return y;
//   }

//   void reset()
//   {
//     z1_ = 0.0f;
//     z2_ = 0.0f;
//   }

// private:
//   float b0_{0}, b1_{0}, b2_{0};
//   float a1_{0}, a2_{0};
//   float z1_{0}, z2_{0};
// };

// class Vector3LPF
// {
// public:
//   Vector3LPF(float cutoff_hz = 20.0f, float sample_rate_hz = 250.0f)
//     : fx_(cutoff_hz, sample_rate_hz),
//       fy_(cutoff_hz, sample_rate_hz),
//       fz_(cutoff_hz, sample_rate_hz)
//   {}

//   void configure(float cutoff_hz, float sample_rate_hz)
//   {
//     fx_.configure(cutoff_hz, sample_rate_hz);
//     fy_.configure(cutoff_hz, sample_rate_hz);
//     fz_.configure(cutoff_hz, sample_rate_hz);
//   }

//   Eigen::Vector3f update(const Eigen::Vector3f& v)
//   {
//     Eigen::Vector3f out;
//     out.x() = fx_.update(v.x());
//     out.y() = fy_.update(v.y());
//     out.z() = fz_.update(v.z());
//     return out;
//   }

//   void reset()
//   {
//     fx_.reset(); fy_.reset(); fz_.reset();
//   }

// private:
//   ButterworthLPF fx_, fy_, fz_;
// };

// // ============================================================================
// // Adaptive INDI (attitude inner-loop) - consistent with INDI derivation
// // - Works in rotor angular speed ω (rad/s).
// // - Uses RPM feedback to build G1(ωf) = I^{-1} * (2*B*diag(ωf)).
// // - Delay compensation: filters ω with same cutoff as Ω̇ estimate.
// // - Optional LMS adaptation on effectiveness matrix.
// // ============================================================================
// class AdaptiveINDI
// {
// public:
//   AdaptiveINDI(float Ixx,
//                float Iyy,
//                float Izz,
//                float arm_x,
//                float arm_y,
//                float k1_thrust,
//                float k2_yaw,
//                float rotor_inertia_zz,
//                float sample_rate_hz,
//                float hover_thrust)
//     : Ixx_(Ixx), Iyy_(Iyy), Izz_(Izz),
//       l_(arm_x), b_(arm_y),
//       k1_(k1_thrust), k2_(k2_yaw),
//       Irzz_(rotor_inertia_zz),
//       fs_hz_(sample_rate_hz),
//       hover_thrust_(hover_thrust)
//   {
//     Iv_inv_.setZero();
//     Iv_inv_(0,0) = 1.0f / Ixx_;
//     Iv_inv_(1,1) = 1.0f / Iyy_;
//     Iv_inv_(2,2) = 1.0f / Izz_;

//     // Geometry matrix B from Mc(ω^2) (paper Eq. 7), used for derivative wrt ω.
//     // roll:  b*k1*(-w1^2 + w2^2 + w3^2 - w4^2)
//     // pitch: l*k1*( w1^2 + w2^2 - w3^2 - w4^2)
//     // yaw:   k2*( w1^2 - w2^2 + w3^2 - w4^2)
//     B_ <<
//       -b_*k1_,  b_*k1_,  b_*k1_, -b_*k1_,
//        l_*k1_,  l_*k1_, -l_*k1_, -l_*k1_,
//        k2_,    -k2_,     k2_,    -k2_;

//     // Filters (control)
//     accel_lpf_.configure(accel_cutoff_hz_, fs_hz_);
//     for (auto& f : omega_lpf_) f.configure(accel_cutoff_hz_, fs_hz_);

//     // Filters (adaptation uses heavier filtering per paper)
//     accel_lpf_adapt_.configure(adapt_accel_cutoff_hz_, fs_hz_);
//     for (auto& f : omega_lpf_adapt_) f.configure(adapt_accel_cutoff_hz_, fs_hz_);

//     reset();
//   }

//   void setRegularization(float lambda) { lambda_ = std::max(1e-9f, lambda); }
//   void setAccelErrorLimit(float lim)   { max_accel_err_ = std::max(1.0f, lim); }
//   void setDeltaOmegaLimit(float lim)   { max_domega_ = std::max(1.0f, lim); }

//   void setAdaptationEnabled(bool en)   { adaptation_enabled_ = en; }
//   void setLearningRates(float mu1, float mu2)
//   {
//     // We interpret these as scalar multipliers; keep it simple & stable.
//     // mu1 scales per-motor update, mu2 scales per-axis update.
//     mu1_ = std::max(0.0f, mu1);
//     mu2_ = std::max(0.0f, mu2);
//   }

//   // Update returns Δω (rad/s) DIFFERENTIAL component (mean removed),
//   // so outer code can add collective ω_col separately.
//   Eigen::Vector4f update(const Eigen::Vector3f& nu_ang_accel_des,  // ν (rad/s^2)
//                          const Eigen::Vector3f& omega_body_meas,   // Ω (rad/s)
//                          const Eigen::Vector4f& motor_rpm_meas,    // RPM feedback
//                          float dt,
//                          bool allow_adapt = false)
//   {
//     if (dt <= 1e-4f) return Eigen::Vector4f::Zero();

//     // RPM -> rad/s
//     const float rpm2rads = 2.0f * static_cast<float>(M_PI) / 60.0f;
//     Eigen::Vector4f omega_rot = motor_rpm_meas * rpm2rads;

//     // Filter ω for delay compensation (ωf aligns with filtered Ω̇f)
//     Eigen::Vector4f omega_f;
//     for (int i = 0; i < 4; i++) omega_f(i) = omega_lpf_[i].update(omega_rot(i));

//     // Body angular acceleration estimate from gyro
//     Eigen::Vector3f omega_dot_meas = Eigen::Vector3f::Zero();
//     if (!have_prev_) {
//       omega_body_prev_ = omega_body_meas;
//       omega_dot_meas.setZero();
//       have_prev_ = true;
//     } else {
//       omega_dot_meas = (omega_body_meas - omega_body_prev_) / dt;
//       omega_body_prev_ = omega_body_meas;
//     }

//     // Filter Ω̇
//     const Eigen::Vector3f omega_dot_f = accel_lpf_.update(omega_dot_meas);

//     // Accel error
//     Eigen::Vector3f e = nu_ang_accel_des - omega_dot_f;
//     for (int i = 0; i < 3; i++) {
//       e(i) = std::clamp(e(i), -max_accel_err_, max_accel_err_);
//     }

//     // Build G1(ωf) = Iv^{-1} * (2*B*diag(ωf))
//     Eigen::Matrix<float,4,4> D = omega_f.asDiagonal();
//     Eigen::Matrix<float,3,4> G_model = Iv_inv_ * (2.0f * B_ * D);

//     // Use estimated/adapted G if enabled, otherwise model G.
//     Eigen::Matrix<float,3,4> G_use = (use_G_est_ ? G_est_ : G_model);

//     // Pseudoinverse: G^+ = G^T (G G^T + λ I)^-1
//     Eigen::Matrix3f GGt = G_use * G_use.transpose();
//     GGt += lambda_ * Eigen::Matrix3f::Identity();
//     Eigen::Matrix<float,4,3> G_pinv = G_use.transpose() * GGt.inverse();

//     // Δω raw
//     Eigen::Vector4f domega = G_pinv * e;

//     // Limit per motor
//     for (int i = 0; i < 4; i++) {
//       domega(i) = std::clamp(domega(i), -max_domega_, max_domega_);
//     }

//     // Remove collective component (we only want attitude differentials)
//     const float mean = domega.mean();
//     domega.array() -= mean;

//     // ---------- Adaptation (LMS) ----------
//     // Based on the paper's adaptive INDI idea: adapt control effectiveness online :contentReference[oaicite:3]{index=3}
//     // We keep it stable: only adapt if allowed, excited enough, and we have history.
//     if (adaptation_enabled_ && allow_adapt) {

//       // Heavier filtered signals for adaptation (noise robustness)
//       Eigen::Vector4f omega_fa;
//       for (int i = 0; i < 4; i++) omega_fa(i) = omega_lpf_adapt_[i].update(omega_rot(i));

//       Eigen::Vector3f omega_dot_fa = accel_lpf_adapt_.update(omega_dot_meas);

//       if (!have_prev_adapt_) {
//         omega_dot_fa_prev_ = omega_dot_fa;
//         have_prev_adapt_ = true;
//       } else {
//         Eigen::Vector3f dOmegaDot_meas = omega_dot_fa - omega_dot_fa_prev_;
//         omega_dot_fa_prev_ = omega_dot_fa;

//         // Use Δω for excitation check
//         const float excite = domega.cwiseAbs().maxCoeff();
//         if (excite > adapt_min_excitation_) {

//           // Predict ΔΩ̇ from current G_est (or init from model if first time)
//           if (!use_G_est_) {
//             // Initialize G_est from model (good starting point)
//             G_est_ = Iv_inv_ * (2.0f * B_ * omega_fa.asDiagonal());
//             use_G_est_ = true;
//           }

//           Eigen::Vector3f dOmegaDot_hat = G_est_ * domega;
//           Eigen::Vector3f eps = dOmegaDot_meas - dOmegaDot_hat;

//           // LMS update: G <- G + α * eps * domega^T
//           // α combines mu2 (axis) and mu1 (input). Keep conservative.
//           const float alpha = (mu1_ * mu2_);
//           G_est_ += alpha * (eps * domega.transpose());

//           // Mild regularization to avoid blow-up (shrink towards model)
//           Eigen::Matrix<float,3,4> G_ref = Iv_inv_ * (2.0f * B_ * omega_fa.asDiagonal());
//           G_est_ = (1.0f - reg_shrink_) * G_est_ + reg_shrink_ * G_ref;

//           // Optional clamp on G_est magnitude (very important in practice)
//           clampG(G_est_, G_max_abs_);
//         }
//       }
//     }

//     last_domega_ = domega;
//     last_omega_dot_f_ = omega_dot_f;

//     return domega;
//   }

//   void reset()
//   {
//     have_prev_ = false;
//     have_prev_adapt_ = false;

//     omega_body_prev_.setZero();
//     omega_dot_fa_prev_.setZero();
//     last_domega_.setZero();
//     last_omega_dot_f_.setZero();

//     accel_lpf_.reset();
//     accel_lpf_adapt_.reset();
//     for (auto& f : omega_lpf_) f.reset();
//     for (auto& f : omega_lpf_adapt_) f.reset();

//     use_G_est_ = false;
//     G_est_.setZero();
//   }

//   Eigen::Vector4f getLastDeltaOmega() const { return last_domega_; }
//   Eigen::Vector3f getLastOmegaDotFiltered() const { return last_omega_dot_f_; }
//   bool usingEstimatedG() const { return use_G_est_; }
//   Eigen::Matrix<float,3,4> getGEstimate() const { return G_est_; }

// private:
//   static void clampG(Eigen::Matrix<float,3,4>& G, float max_abs)
//   {
//     for (int r = 0; r < 3; r++) {
//       for (int c = 0; c < 4; c++) {
//         G(r,c) = std::clamp(G(r,c), -max_abs, max_abs);
//       }
//     }
//   }

// private:
//   // physical
//   float Ixx_, Iyy_, Izz_;
//   float l_, b_;
//   float k1_, k2_;
//   float Irzz_; // not explicitly used in simplified core (kept for future extension)
//   float fs_hz_;
//   float hover_thrust_;

//   Eigen::Matrix3f Iv_inv_{Eigen::Matrix3f::Zero()};
//   Eigen::Matrix<float,3,4> B_{Eigen::Matrix<float,3,4>::Zero()};

//   // filtering
//   float accel_cutoff_hz_ = 20.0f;
//   float adapt_accel_cutoff_hz_ = 10.0f;

//   Vector3LPF accel_lpf_{20.0f, 250.0f};
//   Vector3LPF accel_lpf_adapt_{10.0f, 250.0f};
//   std::array<ButterworthLPF,4> omega_lpf_{
//     ButterworthLPF(20.0f,250.0f), ButterworthLPF(20.0f,250.0f),
//     ButterworthLPF(20.0f,250.0f), ButterworthLPF(20.0f,250.0f)
//   };
//   std::array<ButterworthLPF,4> omega_lpf_adapt_{
//     ButterworthLPF(10.0f,250.0f), ButterworthLPF(10.0f,250.0f),
//     ButterworthLPF(10.0f,250.0f), ButterworthLPF(10.0f,250.0f)
//   };

//   // state
//   bool have_prev_{false};
//   Eigen::Vector3f omega_body_prev_{Eigen::Vector3f::Zero()};

//   Eigen::Vector4f last_domega_{Eigen::Vector4f::Zero()};
//   Eigen::Vector3f last_omega_dot_f_{Eigen::Vector3f::Zero()};

//   // tuning
//   float lambda_ = 1e-3f;
//   float max_accel_err_ = 30.0f;  // rad/s^2
//   float max_domega_ = 200.0f;    // rad/s per update

//   // adaptation
//   bool adaptation_enabled_{false};
//   float mu1_{0.0f};
//   float mu2_{0.0f};

//   float adapt_min_excitation_ = 5.0f; // rad/s
//   float reg_shrink_ = 0.02f;          // pull towards model a little each update
//   float G_max_abs_ = 1e4f;            // hard clamp

//   bool have_prev_adapt_{false};
//   bool use_G_est_{false};
//   Eigen::Vector3f omega_dot_fa_prev_{Eigen::Vector3f::Zero()};
//   Eigen::Matrix<float,3,4> G_est_{Eigen::Matrix<float,3,4>::Zero()};
// };

// } // namespace adaptive_indi_system

// #endif

