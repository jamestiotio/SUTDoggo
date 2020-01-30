
#include "odrive_main.h"


Controller::Controller(Config_t& config) :
    config_(config)
{}

void Controller::reset() {
    pos_setpoint_ = 0.0f;
    vel_setpoint_ = 0.0f;
    vel_integrator_current_ = 0.0f;
    current_setpoint_ = 0.0f;
}

void Controller::set_error(Error_t error) {
    error_ |= error;
    axis_->error_ |= Axis::ERROR_CONTROLLER_FAILED;
}

//--------------------------------
// Command Handling
//--------------------------------

void Controller::set_pos_setpoint(float pos_setpoint, float vel_feed_forward, float current_feed_forward) {
    pos_setpoint_ = pos_setpoint;
    vel_setpoint_ = vel_feed_forward;
    current_setpoint_ = current_feed_forward;
    config_.control_mode = CTRL_MODE_POSITION_CONTROL;
#ifdef DEBUG_PRINT
    printf("POSITION_CONTROL %6.0f %3.3f %3.3f\n", pos_setpoint, vel_setpoint_, current_setpoint_);
#endif
}

void Controller::set_vel_setpoint(float vel_setpoint, float current_feed_forward) {
    vel_setpoint_ = vel_setpoint;
    current_setpoint_ = current_feed_forward;
    config_.control_mode = CTRL_MODE_VELOCITY_CONTROL;
#ifdef DEBUG_PRINT
    printf("VELOCITY_CONTROL %3.3f %3.3f\n", vel_setpoint_, motor->current_setpoint_);
#endif
}

void Controller::set_current_setpoint(float current_setpoint) {
    current_setpoint_ = current_setpoint;
    config_.control_mode = CTRL_MODE_CURRENT_CONTROL;
#ifdef DEBUG_PRINT
    printf("CURRENT_CONTROL %3.3f\n", current_setpoint_);
#endif
}

void Controller::set_coupled_setpoints(float theta_setpoint, float gamma_setpoint) {
    theta_setpoint_ = theta_setpoint;
    gamma_setpoint_ = gamma_setpoint;
    config_.control_mode = CTRL_MODE_COUPLED_CONTROL;
#ifdef DEBUG_PRINT
    printf("COUPLED_CONTROL %3.3f %3.3f\n", theta_setpoint_, gamma_setpoint_);
#endif
}

/**
 * Set gains for coupled PD control
 */
void Controller::set_coupled_gains(float kp_theta, float kd_theta, float kp_gamma, float kd_gamma) {
    config_.kp_theta = kp_theta;
    config_.kd_theta = kd_theta;
    config_.kp_gamma = kp_gamma;
    config_.kd_gamma = kd_gamma;
    config_.control_mode = CTRL_MODE_COUPLED_CONTROL;
#ifdef DEBUG_PRINT
    printf("COUPLED_CONTROL %3.3f %3.3f %3.3f %3.3f\n", kp_theta, kd_theta, kp_gamma, kd_gamma);
#endif
}

void Controller::set_xy_setpoints(float x_setpoint, float y_setpoint) {
  x_setpoint_ = x_setpoint;
  y_setpoint_ = y_setpoint;
  config_.control_mode = CTRL_MODE_XY_CONTROL;
#ifdef DEBUG_PRINT
    printf("XY_CONTROL %3.3f %3.3f\n", x_setpoint_, y_setpoint_);
#endif
}

void Controller::set_xy_gains(float kp_x, float kd_x, float kp_y, float kd_y) {
  config_.kp_x = kp_x;
  config_.kd_x = kd_x;
  config_.kp_y = kp_y;
  config_.kd_y = kd_y;
  config_.control_mode = CTRL_MODE_XY_CONTROL;
  #ifdef DEBUG_PRINT
    printf("XY_CONTROL %3.3f %3.3f %3.3f %3.3f\n", kp_x, kd_x, kp_y, kd_y);
  #endif
}

float Controller::encoder_to_rad(float x) {
    return x / (axis_->encoder_.config_.cpr * config_.gear_ratio) * 2.0f * M_PI;
}

void Controller::move_to_pos(float goal_point) {
    axis_->trap_.planTrapezoidal(goal_point, pos_setpoint_, vel_setpoint_,
                                 axis_->trap_.config_.vel_limit,
                                 axis_->trap_.config_.accel_limit,
                                 axis_->trap_.config_.decel_limit);
    traj_start_loop_count_ = axis_->loop_counter_;
    config_.control_mode = CTRL_MODE_TRAJECTORY_CONTROL;
    goal_point_ = goal_point;
}

void Controller::move_incremental(float displacement, bool from_goal_point = true){
    if(from_goal_point){
        move_to_pos(goal_point_ + displacement);
    } else{
        move_to_pos(pos_setpoint_ + displacement);
    }
}

void Controller::start_anticogging_calibration() {
    // Ensure the cogging map was correctly allocated earlier and that the motor is capable of calibrating
    if (anticogging_.cogging_map != NULL && axis_->error_ == Axis::ERROR_NONE) {
        anticogging_.calib_anticogging = true;
    }
}

/*
 * This anti-cogging implementation iterates through each encoder position,
 * waits for zero velocity & position error,
 * then samples the current required to maintain that position.
 * 
 * This holding current is added as a feedforward term in the control loop.
 */
bool Controller::anticogging_calibration(float pos_estimate, float vel_estimate) {
    if (anticogging_.calib_anticogging && anticogging_.cogging_map != NULL) {
        float pos_err = anticogging_.index - pos_estimate;
        if (fabsf(pos_err) <= anticogging_.calib_pos_threshold &&
            fabsf(vel_estimate) < anticogging_.calib_vel_threshold) {
            anticogging_.cogging_map[anticogging_.index++] = vel_integrator_current_;
        }
        if (anticogging_.index < axis_->encoder_.config_.cpr) { // TODO: remove the dependency on encoder CPR
            set_pos_setpoint(anticogging_.index, 0.0f, 0.0f);
            return false;
        } else {
            anticogging_.index = 0;
            set_pos_setpoint(0.0f, 0.0f, 0.0f);  // Send the motor home
            anticogging_.use_anticogging = true;  // We're good to go, enable anti-cogging
            anticogging_.calib_anticogging = false;
            return true;
        }
    }
    return false;
}

bool Controller::update(float pos_estimate, float vel_estimate, float* current_setpoint_output) {
    // Only runs if anticogging_.calib_anticogging is true; non-blocking
    anticogging_calibration(pos_estimate, vel_estimate);
    float anticogging_pos = pos_estimate;

    // Trajectory control
    if (config_.control_mode == CTRL_MODE_TRAJECTORY_CONTROL) {
        // Note: uint32_t loop count delta is OK across overflow
        // Beware of negative deltas, as they will not be well behaved due to uint!
        float t = (axis_->loop_counter_ - traj_start_loop_count_) * current_meas_period;
        if (t > axis_->trap_.Tf_) {
            // Drop into position control mode when done to avoid problems on loop counter delta overflow
            config_.control_mode = CTRL_MODE_POSITION_CONTROL;
            // pos_setpoint already set by trajectory
            vel_setpoint_ = 0.0f;
            current_setpoint_ = 0.0f;
        } else {
            TrapezoidalTrajectory::Step_t traj_step = axis_->trap_.eval(t);
            pos_setpoint_ = traj_step.Y;
            vel_setpoint_ = traj_step.Yd;
            current_setpoint_ = traj_step.Ydd * axis_->trap_.config_.A_per_css;
        }
        anticogging_pos = pos_setpoint_; // FF the position setpoint instead of the pos_estimate
    }

    // Ramp rate limited velocity setpoint
    if (config_.control_mode == CTRL_MODE_VELOCITY_CONTROL && vel_ramp_enable_) {
        float max_step_size = current_meas_period * config_.vel_ramp_rate;
        float full_step = vel_ramp_target_ - vel_setpoint_;
        float step;
        if (fabsf(full_step) > max_step_size) {
            step = std::copysignf(max_step_size, full_step);
        } else {
            step = full_step;
        }
        vel_setpoint_ += step;
    }

    // PD control
    float Iq = current_setpoint_;

    if (config_.control_mode == CTRL_MODE_POSITION_CONTROL) {
        float pos_err;
        if (config_.setpoints_in_cpr) {
            // TODO this breaks the semantics that estimates come in on the arguments.
            // It's probably better to call a get_estimate that will arbitrate (enc vs sensorless) instead.
            float cpr = (float)(axis_->encoder_.config_.cpr);
            // Keep pos setpoint from drifting
            pos_setpoint_ = fmodf_pos(pos_setpoint_, cpr);
            // Circular delta
            pos_err = pos_setpoint_ - axis_->encoder_.pos_cpr_;
            pos_err = wrap_pm(pos_err, 0.5f * cpr);
        } else {
            pos_err = pos_setpoint_ - pos_estimate;
        }

        Iq += config_.pos_gain * pos_err;

        float vel_err = 0 - vel_estimate;
        Iq += config_.vel_gain * vel_err;
    }

    // Check for overspeed fault (done in this module (controller) for cohesion with vel_lim)
    /*
    if (config_.vel_limit_tolerance > 0.0f) { // 0.0f to disable
        if (fabsf(vel_estimate) > config_.vel_limit_tolerance * vel_lim) {
            set_error(ERROR_OVERSPEED);
            return false;
        }
    }
    */

    // Coupled PD control
    // The convention we're using is that alpha and beta are measured from the downward vertical
    // both increasing in the CCW direction.
    if (config_.control_mode == CTRL_MODE_COUPLED_CONTROL) {
        float alpha = encoder_to_rad(axes[0]->encoder_.pos_estimate_) + M_PI/2.0f;
        float beta = encoder_to_rad(axes[1]->encoder_.pos_estimate_) - M_PI/2.0f; // Assumes legs started 180 apart
        // float d_alpha = encoder_to_rad(axes[0]->encoder_.pll_vel_);
        // float d_beta = encoder_to_rad(axes[1]->encoder_.pll_vel_);
        float d_alpha = encoder_to_rad(axes[0]->encoder_.pll_kp_);
        float d_beta = encoder_to_rad(axes[1]->encoder_.pll_kp_);

        float theta = alpha/2.0f + beta/2.0f;
        float gamma = alpha/2.0f - beta/2.0f;

        theta_ = theta;
        gamma_ = gamma;

        float d_theta = d_alpha/2.0f + d_beta/2.0f;
        float d_gamma = d_alpha/2.0f - d_beta/2.0f;

        float p_term_theta = config_.kp_theta * (theta_setpoint_ - theta);
        float d_term_theta = config_.kd_theta * (0.0f - d_theta);

        float p_term_gamma = config_.kp_gamma * (gamma_setpoint_ - gamma);
        float d_term_gamma = config_.kd_gamma * (0.0f - d_gamma);

        float tau_theta = p_term_theta + d_term_theta;
        float tau_gamma = p_term_gamma + d_term_gamma;

        axes[0]->controller_.current_setpoint_ = tau_theta*0.5f + tau_gamma*0.5f;
        axes[1]->controller_.current_setpoint_ = tau_theta*0.5f - tau_gamma*0.5f;

        Iq = current_setpoint_;
    } else if(config_.control_mode == CTRL_MODE_XY_CONTROL) { //change condition for now...
///////////////////////////////////////////////////////////////////////////////////
//current theta, gamma
        float alpha = encoder_to_rad(axes[0]->encoder_.pos_estimate_) + M_PI/2.0f;
        float beta = encoder_to_rad(axes[1]->encoder_.pos_estimate_) - M_PI/2.0f; // Assumes legs started 180 apart
        // float d_alpha = encoder_to_rad(axes[0]->encoder_.pll_vel_);
        // float d_beta = encoder_to_rad(axes[1]->encoder_.pll_vel_);
        float d_alpha = encoder_to_rad(axes[0]->encoder_.pll_kp_);
        float d_beta = encoder_to_rad(axes[1]->encoder_.pll_kp_);

        float theta = alpha/2.0f + beta/2.0f;
        float gamma = alpha/2.0f - beta/2.0f;
        theta_ = theta;
        gamma_ = gamma;

        float d_theta = d_alpha/2.0f + d_beta/2.0f;
        float d_gamma = d_alpha/2.0f - d_beta/2.0f;

        //leg parameters
        float L1 = 0.09f; // upper leg length (m)
        float L2 = 0.162f; // lower leg length (m)
        float L = L1*cos(gamma) + sqrt(L2*L2 - L1*L1*sin(gamma)*sin(gamma));

        // jacobian stuff
        float dradius_dgamma = -L1*sin(gamma) - (L1*L1*sin(gamma)*cos(gamma))/(sqrt(L2*L2 - L1*L1*sin(gamma)*sin(gamma)));
        float dx_dtheta = L*cos(theta);
        float dy_dtheta = -L*sin(theta);
        float dx_dradius = sin(theta);
        float dy_dradius = cos(theta);

        float jacobian[2][2] = {
                                  {dx_dtheta, dx_dradius*dradius_dgamma},
                                  {dy_dtheta, dy_dradius*dradius_dgamma},
                                };

        J00 = jacobian[0][0];
        J01 = jacobian[0][1];
        J10 = jacobian[1][0];
        J11 = jacobian[1][1];

        //current x, y
        float x = L * sin(theta); //How to get leg_direction here?
        float y = L * cos(theta);

        x_pos_ = x;
        y_pos_ = y;

        float d_x = jacobian[0][1]*d_gamma + jacobian[0][0]*d_theta; //derivative of x wrt time
        float d_y = jacobian[1][1]*d_gamma + jacobian[1][0]*d_theta;

        //x, y setpoints set manually here if doing x compliance, y setpoint should be same as curr y
        // and vice versa.
        float x_sp = x_setpoint_; //make x set point 0
        float y_sp = y_setpoint_; //

        //gains wanted for x and y
        //when doing x compliance, set y kp's and kd's to 0... no desire to move in those directions
        float kp_x = config_.kp_x;
        float kp_y = config_.kp_y;
        float kd_x = config_.kd_x;
        float kd_y = config_.kd_y;

        float p_term_x = kp_x * (x_sp - x);
        float d_term_x = kd_x * (0.0f - d_x);

        float p_term_y = kp_y * (y_sp - y);
        float d_term_y = kd_y * (0.0f - d_y);

        float force_x = p_term_x + d_term_x;
        float force_y = p_term_y + d_term_y;
        force_x_ = force_x;
        force_y_ = force_y;


        float tau_theta = force_x * jacobian[0][0] + force_y * jacobian[1][0]; //mutliplying by jacobian transpose
        float tau_gamma = force_x * jacobian[0][1] + force_y * jacobian[1][1];
        tau_theta_ = tau_theta;
        tau_gamma_ = tau_gamma;

        axes[0]->controller_.current_setpoint_ = tau_theta*0.5f + tau_gamma*0.5f;
        axes[1]->controller_.current_setpoint_ = tau_theta*0.5f - tau_gamma*0.5f;

        Iq = current_setpoint_;

///////////////////////////////////////////////////////////////////////////////////


    }

    // Anti-cogging is enabled after calibration
    // We get the current position and apply a current feed-forward
    // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
    if (anticogging_.use_anticogging) {
        Iq += anticogging_.cogging_map[mod(static_cast<int>(anticogging_pos), axis_->encoder_.config_.cpr)];
    }

    // Current limiting
    bool limited = false;
    float Ilim = axis_->motor_.effective_current_lim();
    if (Iq > Ilim) {
        limited = true;
        Iq = Ilim;
    }
    if (Iq < -Ilim) {
        limited = true;
        Iq = -Ilim;
    }

    if (current_setpoint_output) *current_setpoint_output = Iq;
    return true;
}
