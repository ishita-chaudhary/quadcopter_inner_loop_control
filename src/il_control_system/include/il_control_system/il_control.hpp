#ifndef IL_CONTROL_HPP
#define IL_CONTROL_HPP

#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

namespace il_control_system
{

// ============================================================================
// Adaptive Torque Inner Loop (Synchronized)
// ============================================================================

class IlControl
{
public:
    IlControl()
    {
        // Default nominal inertia (Jbar)
        Jbar_ = Eigen::Vector3f(0.06f, 0.06f, 0.10f).asDiagonal();
        
        // Default Gains
        Kp_eta_ = Eigen::Vector3f(0.8f, 0.8f, 0.5f).asDiagonal();
        Kd_eta_ = Eigen::Vector3f(4.0f, 4.0f, 2.0f).asDiagonal();
        // Kp_eta_ = Eigen::Vector3f(8.0f, 8.0f, 4.0f).asDiagonal();
        // Kd_eta_ = Eigen::Vector3f(0.8f, 0.8f, 0.5f).asDiagonal();
        
        torque_limits_ << 3.0f, 3.0f, 1.5f; // N-m limits
        use_current_body_rate_frame_ = false;

        alpha_filter_ = 0.1f; 

        reset();
    }

    void reset()
    {
        prev_tau_.setZero();
        prev_omega_.setZero();
        prev_omega_timestamp_us_ = 0;
        prev_omega_ff_current_.setZero();
        
        filtered_omega_dot_.setZero();
        filtered_tau_.setZero();
        Hhat_bar_.setZero();
        
        // Multi-rate integration memory
        integral_tau_.setZero();
        time_since_last_meas_ = 0.0f;
        
        first_run_ = true;
    }

    void setParameters(const Eigen::Matrix3f& Jbar, 
                       const Eigen::Matrix3f& Kp_eta, 
                       const Eigen::Matrix3f& Kd_eta, 
                       const Eigen::Vector3f& torque_limits,
                       bool use_current_body_rate_frame)
    {
        Jbar_ = Jbar;
        Kp_eta_ = Kp_eta;
        Kd_eta_ = Kd_eta;   
        torque_limits_ = torque_limits;
        use_current_body_rate_frame_ = use_current_body_rate_frame;
    }

    Eigen::Vector3f update(
        const Eigen::Quaternionf& q_current,
        const Eigen::Vector3f& omega_current,
        uint64_t omega_timestamp_us,
        const Eigen::Quaternionf& q_desired,
        const Eigen::Vector3f& omega_d_ff,
        float loop_dt)
    {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        
        // // 1. Rotation Matrices
        // Eigen::Matrix3f R = q_current.toRotationMatrix();
        // Eigen::Matrix3f R_d = q_desired.toRotationMatrix();

        // // 2. SO(3) Attitude Error
        // Eigen::Matrix3f R_err_mat = R_d.transpose() * R - R.transpose() * R_d;
        // Eigen::Vector3f e_R = 0.5f * vee(R_err_mat);

        // // 3. Map Rate Feedforward
        // Eigen::Vector3f omega_ff_current;
        // if (use_current_body_rate_frame_) {
        //     omega_ff_current = omega_d_ff;
        // } else {
        //     omega_ff_current = R.transpose() * R_d * omega_d_ff;
        // }

        // // 4. Rate Error
        // Eigen::Vector3f e_omega = omega_current - omega_ff_current;

        // // 5. Finite Differences (Accelerations)
        // Eigen::Vector3f raw_omega_dot_meas = Eigen::Vector3f::Zero();
        // Eigen::Vector3f omega_ff_dot = Eigen::Vector3f::Zero();
        
        // if (!first_run_ && dt > 0.0f) {
        //     raw_omega_dot_meas = (omega_current - prev_omega_) / dt;    
        //     // --- APPLY LOW PASS FILTER TO ACCELERATION ---
        //     filtered_omega_dot_ = (alpha_filter_ * raw_omega_dot_meas) + 
        //                           ((1.0f - alpha_filter_) * filtered_omega_dot_);

        //     omega_ff_dot = (omega_ff_current - prev_omega_ff_current_) / dt;
                                  
        //     // --- APPLY LOW PASS FILTER TO PREVIOUS TORQUE ---
        //     // Motors have a delay. The drone doesn't instantly feel the torque we requested last loop.
        //     filtered_tau_ = (alpha_filter_ * prev_tau_) + 
        //                     ((1.0f - alpha_filter_) * filtered_tau_);
        // } else {
        //     filtered_omega_dot_ = Eigen::Vector3f::Zero();
        //     filtered_tau_ = Eigen::Vector3f::Zero();
        // }

        // // 6. Delay-based Lumped Uncertainty Estimate (Eq 14)
        // Eigen::Vector3f Hhat_bar = filtered_tau_ - Jbar_ * filtered_omega_dot_;

        // // Precompute terms to safely print them
        // Eigen::Vector3f kp_term = -Kp_eta_ * e_R;
        // Eigen::Vector3f kd_term = -Kd_eta_ * e_omega;
        // Eigen::Vector3f Ia_term = Jbar_ * (omega_ff_dot + kp_term + kd_term);

        // // 7. Adaptive Torque Law (Eq 13)
        // Eigen::Vector3f torque_unsat = Ia_term + Hhat_bar;

        // 1. Rotation Matrices
        Eigen::Matrix3f R = q_current.toRotationMatrix();
        Eigen::Matrix3f R_d = q_desired.toRotationMatrix();

        // 2. SO(3) Attitude Error
        Eigen::Matrix3f R_err_mat = R_d.transpose() * R - R.transpose() * R_d;
        Eigen::Vector3f e_R = 0.5f * vee(R_err_mat);

        // 3. Map Rate Feedforward
        Eigen::Vector3f omega_ff_current;
        if (use_current_body_rate_frame_) {
            omega_ff_current = omega_d_ff;
        } else {
            omega_ff_current = R.transpose() * R_d * omega_d_ff;
        }

        // 4. Rate Error
        Eigen::Vector3f e_omega = omega_current - omega_ff_current;

        // ====================================================================
        // 5. ASYNCHRONOUS MULTI-RATE ESTIMATOR
        // ====================================================================
        
        // Integrate the torque applied over this control step (Zero-Order Hold assumption)
        integral_tau_ += prev_tau_ * loop_dt;
        time_since_last_meas_ += loop_dt;

        bool is_new_omega = (omega_timestamp_us > prev_omega_timestamp_us_);
        
        if (is_new_omega && !first_run_ && prev_omega_timestamp_us_ > 0) {
            
            // Calculate actual dt between gyro samples (typically ~0.01s for 100Hz)
            float dt_meas = (omega_timestamp_us - prev_omega_timestamp_us_) * 1e-6f;
            
            // Sanity check to prevent divide by zero if timestamps get corrupted
            if (dt_meas > 0.0001f && time_since_last_meas_ > 0.0001f) {
                
                // Average acceleration over the measurement window
                Eigen::Vector3f raw_omega_dot_meas = (omega_current - prev_omega_) / dt_meas;    
                
                // Average torque applied over the EXACT same measurement window
                Eigen::Vector3f tau_avg = integral_tau_ / time_since_last_meas_;

                // --- APPLY SYNCHRONIZED LOW PASS FILTERS ---
                filtered_omega_dot_ = (alpha_filter_ * raw_omega_dot_meas) + 
                                      ((1.0f - alpha_filter_) * filtered_omega_dot_);

                filtered_tau_ = (alpha_filter_ * tau_avg) + 
                                ((1.0f - alpha_filter_) * filtered_tau_);
                                
                // --- UPDATE DISTURBANCE ESTIMATOR ---
                // Physically accurate: Average Torque - J * Average Acceleration
                Hhat_bar_ = filtered_tau_ - Jbar_ * filtered_omega_dot_;
            }
            
            // Reset integrators for the next window
            integral_tau_.setZero();
            time_since_last_meas_ = 0.0f;
            
            // Save state
            prev_omega_ = omega_current;
            prev_omega_timestamp_us_ = omega_timestamp_us;
        } else if (is_new_omega && first_run_) {
            // First time receiving data, just initialize timestamps
            prev_omega_ = omega_current;
            prev_omega_timestamp_us_ = omega_timestamp_us;
        }

        // ====================================================================
        // 6. CONTROL LAW
        // ====================================================================
        
        // Feedforward Acceleration (using loop_dt for smooth command transitions)
        Eigen::Vector3f omega_ff_dot = Eigen::Vector3f::Zero();
        if (!first_run_ && loop_dt > 0.0f) {
            omega_ff_dot = (omega_ff_current - prev_omega_ff_current_) / loop_dt;
        }

        Eigen::Vector3f kp_term = -Kp_eta_ * e_R;
        Eigen::Vector3f kd_term = -Kd_eta_ * e_omega;
        Eigen::Vector3f Ia_term = Jbar_ * (omega_ff_dot + kp_term + kd_term);

        // Adaptive Torque Law (Eq 13)
        // Uses the Hhat_bar_ which only updates when fresh gyro data arrives
        Eigen::Vector3f torque_unsat = Ia_term + Hhat_bar_;

        // 7. Symmetric Clipping
        Eigen::Vector3f torque_cmd;
        for (int i = 0; i < 3; i++) {
            torque_cmd(i) = std::max(-torque_limits_(i), std::min(torque_limits_(i), torque_unsat(i)));
        }

        // RCLCPP_INFO_THROTTLE(
        //     rclcpp::get_logger("IL_Debug"), steady_clock, 4,
        //     "Att Err: [%.2f, %.2f, %.2f]", e_R(0), e_R(1), e_R(2)
        // );
        
        // RCLCPP_INFO_THROTTLE(
        //     rclcpp::get_logger("IL_Debug"), steady_clock, 4,
        //     "w_d: [%.2f, %.2f, %.2f]", omega_ff_current(0), omega_ff_current(1), omega_ff_current(2)
        // );
        
        // RCLCPP_INFO_THROTTLE(
        //     rclcpp::get_logger("IL_Debug"), steady_clock, 4,
        //     "w_curr: [%.2f, %.2f, %.2f]", omega_current(0), omega_current(1), omega_current(2)
        // );

        // RCLCPP_INFO_THROTTLE(
        //     rclcpp::get_logger("IL_Debug"), steady_clock, 4,
        //     "w_err: [%.2f, %.2f, %.2f]", e_omega(0), e_omega(1), e_omega(2)
        // );

        // RCLCPP_INFO_THROTTLE(
        //     rclcpp::get_logger("IL_Debug"), steady_clock, 4,
        //     "a_act: [%.2f, %.2f, %.2f]", filtered_omega_dot_(0), filtered_omega_dot_(1), filtered_omega_dot_(2)
        // );

        // RCLCPP_INFO_THROTTLE(
        //     rclcpp::get_logger("IL_Debug"), steady_clock, 4,
        //     "a_d: [%.2f, %.2f, %.2f]", omega_ff_dot(0), omega_ff_dot(1), omega_ff_dot(2)
        // );
        
        // RCLCPP_INFO_THROTTLE(
        //     rclcpp::get_logger("IL_Debug"), steady_clock, 4,
        //     "prev_tau: [%.2f, %.2f, %.2f]", filtered_tau_(0), filtered_tau_(1), filtered_tau_(2)
        // );
        
        // RCLCPP_INFO_THROTTLE(
        //     rclcpp::get_logger("IL_Debug"), steady_clock, 4,
        //     "H_pred: [%.2f, %.2f, %.2f]", Hhat_bar_(0), Hhat_bar_(1), Hhat_bar_(2)
        // );

        // RCLCPP_INFO_THROTTLE(
        //     rclcpp::get_logger("IL_Debug"), steady_clock, 4,
        //     "-kp*att_err: [%.2f, %.2f, %.2f]", kp_term(0), kp_term(1), kp_term(2)
        // );
        // RCLCPP_INFO_THROTTLE(
        //     rclcpp::get_logger("IL_Debug"), steady_clock, 4,
        //     "-kd*w_err: [%.2f, %.2f, %.2f]", kd_term(0), kd_term(1), kd_term(2)
        // );
        // RCLCPP_INFO_THROTTLE(
        //     rclcpp::get_logger("IL_Debug"), steady_clock, 4,
        //     "Ia: [%.2f, %.2f, %.2f]", Ia_term(0), Ia_term(1), Ia_term(2)
        // );
        // RCLCPP_INFO_THROTTLE(
        //     rclcpp::get_logger("IL_Debug"), steady_clock, 4,
        //     "tau_unsat: [%.2f, %.2f, %.2f]", torque_unsat(0), torque_unsat(1), torque_unsat(2)
        // );

        // 8. Update Loop Memory
        prev_tau_ = torque_cmd;
        prev_omega_ff_current_ = omega_ff_current;
        first_run_ = false;

        // // 8. Symmetric Clipping
        // Eigen::Vector3f torque_cmd;
        // for (int i = 0; i < 3; i++) {
        //     torque_cmd(i) = std::max(-torque_limits_(i), std::min(torque_limits_(i), torque_unsat(i)));
        // }

        // 9. Update Delayed State Memory
        // prev_tau_ = torque_cmd;
        // prev_omega_dot_ = raw_omega_dot_meas;
        // prev_omega_ = omega_current;
        // prev_omega_ff_current_ = omega_ff_current;
        // first_run_ = false;

        return torque_cmd;
    }

private:
    Eigen::Vector3f vee(const Eigen::Matrix3f& M)
    {
        return Eigen::Vector3f(M(2, 1) - M(1, 2), 
                               M(0, 2) - M(2, 0), 
                               M(1, 0) - M(0, 1)) * 0.5f;
    }

    Eigen::Matrix3f Jbar_;
    Eigen::Matrix3f Kp_eta_;
    Eigen::Matrix3f Kd_eta_;
    Eigen::Vector3f torque_limits_;
    bool use_current_body_rate_frame_;

    Eigen::Vector3f filtered_omega_dot_{Eigen::Vector3f::Zero()};
    Eigen::Vector3f filtered_tau_{Eigen::Vector3f::Zero()};
    Eigen::Vector3f Hhat_bar_{Eigen::Vector3f::Zero()};
    
    // Filter coefficient (0.0 to 1.0). 
    // Lower = smoother but more delayed. 0.1 is a good start for 250Hz.
    float alpha_filter_ = 0.1f;

    Eigen::Vector3f prev_tau_;
    Eigen::Vector3f prev_omega_;
    uint64_t prev_omega_timestamp_us_ = 0;
    
    // Memory for Multi-Rate Integration
    Eigen::Vector3f integral_tau_{Eigen::Vector3f::Zero()};
    float time_since_last_meas_ = 0.0f;
    
    Eigen::Vector3f prev_omega_ff_current_;
    bool first_run_;
};

} // namespace il_control_system

#endif