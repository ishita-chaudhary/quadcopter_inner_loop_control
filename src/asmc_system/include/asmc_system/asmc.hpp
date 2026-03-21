#ifndef ASMC_HPP
#define ASMC_HPP

#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace asmc_system
{

class ASMC
{
public:
  ASMC()
  {
    // Default gains from Python script (asmc_controller.py)
    // Lam_t_ << 0.06f, 0.06f, 0.5f;
    // Phi_t_ << 0.01f, 0.01f, 0.01f;
    Lam_t_ << 0.12f, 0.12f, 1.0f;
    Phi_t_ << 4.0f, 4.0f, 4.0f;
    // Phi_t_ << 2.0f, 2.0f, 2.0f;

    
    // Adaptive gains state
    // Kp0_t_ << 0.01f, 0.01f, 0.01f;
    Kp0_t_ << 0.0f, 0.0f, 0.0f;
    Kp1_t_ << 0.0f, 0.0f, 0.0f;

    alpha0_t_ << 2.0f, 2.0f, 2.0f;  //0.1
    alpha1_t_ << 2.0f, 2.0f, 2.0f;  //0.1

    
    // Tuning parameters
    M_bar_t_ = 0.03f;
    v_t_ = 0.1f;
    max_torque_ = 1.0f; // As per python script

    pre_time_ns_ = 0;
  }

  // Corresponds to des_tq() in python
  Eigen::Vector3f update(
    const Eigen::Quaternionf& q_current,
    const Eigen::Vector3f& omega_current,
    const Eigen::Quaternionf& q_desired,
    const Eigen::Vector3f& omega_desired,
    const Eigen::Vector3f& alpha_desired,
    float dt)
  {
    // 1. Rotation Matrices
    Eigen::Matrix3f R_curr = q_current.toRotationMatrix();
    Eigen::Matrix3f R_des  = q_desired.toRotationMatrix();

    // 2. Attitude Error (Variational)
    // errQ = 0.5 * vee(R_des^T * R_curr - R_curr^T * R_des)
    Eigen::Matrix3f R_err_mat = R_des.transpose() * R_curr - R_curr.transpose() * R_des;
    Eigen::Vector3f errQ = 0.5f * vee(R_err_mat);

    // 3. Angular Velocity Error
    // errAngVel = currAngVel - (R_curr^T * R_des * desAngVel)
    // Note: Python logic: currAngVel - (rot_curr.T @ rot_des @ self.desAngVel)
    Eigen::Vector3f errAngVel = omega_current - (R_curr.transpose() * R_des * omega_desired);

    // 4. Sliding Surface (s)
    // sv = errAngVel + Phi * errQ
    Eigen::Vector3f sv = errAngVel + Phi_t_.cwiseProduct(errQ);
    // std::cout << "Angvel err: " << errAngVel << std::endl;
    // std::cout << "Att err: " << errQ << std::endl;


    // 5. Adaptive Gain Update
    // Kp0_t = norm(sv) - alpha_t * Kp0_t (integrated over time usually, but python does iterative update)
    // Python: self.Kp0_t = np.linalg.norm(sv) - np.multiply(self.alpha_t, self.Kp0_t)
    // NOTE: The python script does NOT multiply by dt here for Kp0_t in des_tq, unlike a_des. 
    // However, for physical consistency in C++, this usually requires dt. 
    // We will follow the Python logic exactly: It acts as a low-pass filter / observer if iterated.
    // Ideally this should be: Kp0 += (norm(sv) - alpha * Kp0) * dt.
    // Given the Python script: self.Kp0_t = ... 
    // We will implement the Python logic:
    float sv_norm = sv.norm();
    // std::cout << "sv: " << sv << std::endl;

    float deadzone = 0.1f; 
    float driving_term = 0.0f;
    
    if (sv_norm > deadzone) {
        // Only drive the gain up if error exceeds noise floor
        driving_term = sv_norm - deadzone;
    }

    // To match Python exact behavior (which looks like an instantaneous assignment rather than integration in des_tq):
    // If the python script meant integration, it was missing `+=` and `* dt`. 
    // Assuming we want the adaptive behavior described:
    Eigen::Vector3f Kp0_dot_ = (Eigen::Vector3f::Constant(sv_norm) - alpha0_t_.cwiseProduct(Kp0_t_));
    Eigen::Vector3f Kp1_dot_ = (Eigen::Vector3f::Constant(sv_norm) - alpha1_t_.cwiseProduct(Kp1_t_));

    Kp0_t_ = Kp0_dot_ * dt;
    Kp1_t_ = Kp1_dot_ * dt;

    
    Kp0_t_ = Kp0_t_.cwiseMax(0.001f); // Safety clamp to prevent negative gains if dynamics oscillate
    Kp0_t_ = Kp0_t_.cwiseMin(0.3f); // Don't let adaptive gain exceed 2.0
    Kp1_t_ = Kp1_t_.cwiseMax(0.001f); // Safety clamp to prevent negative gains if dynamics oscillate
    Kp1_t_ = Kp1_t_.cwiseMin(0.3f); // Don't let adaptive gain exceed 2.0

    Eigen::Vector3f rho = Kp0_t_ + Kp1_t_.cwiseProduct(errQ.cwiseAbs());

    // 6. Switching Term (delTau)
    Eigen::Vector3f delTau;
    delTau(0) = rho(0) * sigmoid(sv(0), v_t_);
    delTau(1) = rho(1) * sigmoid(sv(1), v_t_);
    delTau(2) = rho(2) * sigmoid(sv(2), v_t_);

    // 7. Control Law
    // des_tq = -Lam * sv + M_bar * (desAngAcc - Phi * errAngVel) - delTau
    Eigen::Vector3f term1 = -Lam_t_.cwiseProduct(sv);
    Eigen::Vector3f term2 = M_bar_t_ * (alpha_desired - Phi_t_.cwiseProduct(errAngVel));
    
    Eigen::Vector3f des_tq = term1 + term2 - delTau;
    // std::cout << "term1: " << term1 << std::endl;
    // std::cout << "term2: " << term2 << std::endl;
    // std::cout << "del_Tau: " << -delTau << std::endl;
    // std::cout << "des_tq: " << des_tq << "\n" << std::endl;

    

    // 8. Saturation
    float max_t = max_torque_;
    des_tq(0) = std::max(-max_t, std::min(max_t, des_tq(0)));
    des_tq(1) = std::max(-max_t, std::min(max_t, des_tq(1)));
    des_tq(2) = std::max(-max_t, std::min(max_t, des_tq(2)));

    //9. Normalised torque for px4
    float raw_roll  = des_tq(0);
    float raw_pitch = des_tq(1);
    float raw_yaw   = des_tq(2);
    
    // Step A: Convert N-m to normalized commands using the Inverse Allocation Matrix
    float norm_pitch_cmd = 0.2372f * raw_pitch;
    float norm_yaw_cmd   = 2.5801f * raw_yaw;
    float norm_roll_cmd  = (0.2372f * raw_roll) - (0.1229f * raw_yaw);  

    // Step B: Apply Tilt-Prioritization
    // 1. TILT PRIORITY: Clamp Roll and Pitch strictly to [-1.0, 1.0]
    float norm_roll_final  = std::max(-1.0f, std::min(1.0f, norm_roll_cmd));
    float norm_pitch_final = std::max(-1.0f, std::min(1.0f, norm_pitch_cmd));

    // 2. CALCULATE REMAINING HEADROOM FOR YAW
    // Headroom = 1.0 - max(|Roll|, |Pitch|)
    float used_effort = std::max(std::abs(norm_roll_final), std::abs(norm_pitch_final));
    float yaw_headroom = std::max(0.0f, 1.0f - used_effort);

    // 3. CLAMP YAW to use only the remaining headroom
    float norm_yaw_final = std::max(-yaw_headroom, std::min(yaw_headroom, norm_yaw_cmd));

    // 4. Assign the safely allocated values back to the output vector
    des_tq(0) = norm_roll_final;
    des_tq(1) = norm_pitch_final;
    des_tq(2) = norm_yaw_final;

    return des_tq;
  }

  void setGains(const Eigen::Vector3f& Lam, const Eigen::Vector3f& Phi, const Eigen::Vector3f& alpha, float M_bar)
  {
    Lam_t_ = Lam;
    Phi_t_ = Phi;
    alpha0_t_ = alpha;
    M_bar_t_ = M_bar;
  }

private:
  // Helper: vee map (skew-symmetric matrix to vector)
  Eigen::Vector3f vee(const Eigen::Matrix3f& M)
  {
    return Eigen::Vector3f(M(2, 1), M(0, 2), M(1, 0));
  }

  // Helper: sigmoid / saturation function
  float sigmoid(float s, float v)
  {
    if (std::abs(s) > v) {
      return s / std::abs(s);
    } else {
      return s / v;
    }
  }

  // Parameters
  Eigen::Vector3f Lam_t_;
  Eigen::Vector3f Phi_t_;
  Eigen::Vector3f alpha0_t_;
  Eigen::Vector3f alpha1_t_;

  float M_bar_t_;
  float v_t_;
  float max_torque_;

  // State
  Eigen::Vector3f Kp0_t_;
  Eigen::Vector3f Kp1_t_;

  uint64_t pre_time_ns_;
};

} // namespace adaptive_indi_system

#endif