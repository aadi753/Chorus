#include "single_dof_otg.h"

/**
 * @brief Compute normalized errors for position, velocity, and acceleration.
 */
void Chorus::OTG::computeNormalizedError_( ) {
    // Compute normalized error based on the target parameters
    U_ = params_.max_jerk;
    if ( std::abs( U_ ) < EPSILON ) {
        U_ = EPSILON;
    }
    error_pos_ = ( output_.position - params_.target_position ) / U_;
    error_vel_ = ( output_.velocity - params_.target_velocity ) / U_;
    error_acc_ = ( output_.acceleration - params_.target_acceleration ) / U_;


    // std::cout << "error_pos: " << error_pos_ << " error_vel: " << error_vel_ << " error_acc: " << error_acc_ << "\n";

}

/**
 * @brief Update motion constraints based on the target parameters.
 */
void Chorus::OTG::updateMotionConstraints_( ) {
    // Update motion constraints based on the target parameters
    min_velocity_ = ( params_.min_velocity - params_.target_velocity ) / U_;
    max_velocity_ = ( params_.max_velocity - params_.target_velocity ) / U_;
    max_acceleration_ = ( params_.max_acceleration - params_.target_acceleration ) / U_;
    min_acceleration_ = ( params_.min_acceleration - params_.target_acceleration ) / U_;
    // std::cout << "min_velocity: " << min_velocity_ << " max_velocity: " << max_velocity_ << " min_acceleration: " << min_acceleration_ << " max_acceleration: " << max_acceleration_ << "\n";
}

/**
 * @brief Compute the sigma parameter for the C3 filter.
 * @return The computed sigma value.
 */
double Chorus::OTG::computeSigma_( ) {
    // Compute sigma based on the current error and motion constraints
    // double sigma = error_pos_ + ( error_vel_ * error_acc_ * sign_ ) - ( ( std::pow( error_acc_, 3 ) / 6.0 ) * ( 1 - 3 * std::abs( sign_ ) ) + ( ( sign_ / 4.0 ) * sqrt( 2 * std::pow( ( ( error_acc_ * error_acc_ ) + ( 2 * error_vel_ * sign_ ) ), 3 ) ) ) );

    double term1 = error_pos_;
    double term2 = error_vel_ * error_acc_ * sign_;
    double term3 = ( std::pow( error_acc_, 3 ) / 6.0 ) * ( 1.0 - 3.0 * std::abs( sign_ ) );
    double inner = std::pow( error_acc_, 2 ) + 2.0 * error_vel_ * sign_;
    if ( inner < 0.0 ) {
        inner = 0.0;
    }
    double term4 = ( sign_ / 4.0 ) * std::sqrt( 2.0 * std::pow( inner, 3 ) );

    double sigma = term1 + term2 - term3 + term4;

    return sigma;
}

/**
 * @brief Compute the mu_positive parameter for the C3 filter.
 * @return The computed mu_positive value.
 */
double Chorus::OTG::computeMuPositive_( ) {
    // Compute mu_positive based on the current error and motion constraints
    // double mu_positive = error_pos_ - ( ( max_acceleration_ * ( std::pow( error_acc_, 2 ) - ( 2 * error_vel_ ) ) ) / 4.0 ) - ( ( std::pow( ( std::pow( error_acc_, 2 ) - ( 2 * error_vel_ ) ), 2 ) ) / ( 8 * max_velocity_ ) ) - ( ( error_acc_ * ( ( 3 * error_vel_ ) - std::pow( error_acc_, 2 ) ) ) / 3.0 );

    double mu_positive = error_pos_ - ( ( max_acceleration_ * ( std::pow( error_acc_, 2 ) - ( 2 * error_vel_ ) ) ) / 4.0 ) - ( ( std::pow( ( std::pow( error_acc_, 2 ) - ( 2 * error_vel_ ) ), 2 ) ) / ( 8 * max_acceleration_ ) ) - ( ( error_acc_ * ( ( 3 * error_vel_ ) - std::pow( error_acc_, 2 ) ) ) / 3.0 );

    return mu_positive;
}


/**
 * @brief Compute the mu_negative parameter for the C3 filter.
 * @return The computed mu_negative value.
 */
double Chorus::OTG::computeMuNegative_( ) {
    double mu_negative = error_pos_ - ( ( min_acceleration_ * ( std::pow( error_acc_, 2 ) + ( 2 * error_vel_ ) ) ) / 4.0 ) - ( ( std::pow( ( std::pow( error_acc_, 2 ) + ( 2 * error_vel_ ) ), 2 ) ) / ( 8 * min_acceleration_ ) ) + ( ( error_acc_ * ( ( 3 * error_vel_ ) + std::pow( error_acc_, 2 ) ) ) / 3.0 );

    return mu_negative;
}

/**
 * @brief Compute the summation parameter for the C3 filter.
 * @return The computed summation value.
 */
double Chorus::OTG::computeSummation_( ) {
    if ( error_acc_ <= max_acceleration_ && error_vel_ <= ( ( std::pow( error_acc_, 2 ) / 2.0 ) - std::pow( max_acceleration_, 2 ) ) ) {
        return mu_positive_;
    }
    else if ( error_acc_ >= min_acceleration_ && error_vel_ >= ( std::pow( min_acceleration_, 2 ) - ( std::pow( error_acc_, 2 ) / 2.0 ) ) ) {
        return mu_negative_;
    }

    return sigma_;



}

/**
 * @brief Compute the control variable uc.
 * @return True if successful.
 */
bool Chorus::OTG::computeUc_( ) {
    const double dt = params_.sampling_rate;
    const double expr = summation_ + ( 1.0 - std::abs( sign_summation_ ) ) * ( delta_ + ( 1.0 - std::abs( sign_ ) ) * error_acc_ );
    
    // Continuous boundary layer for the position sliding surface to eliminate discrete relay chatter
    const double phi_pos = std::max( EPSILON, 0.5 * std::abs( max_acceleration_ ) * ( dt * dt ) + ( std::pow( dt, 3 ) / 6.0 ) );
    uc_ = -U_ * sat_( expr, phi_pos );
    return true;
}

/**
 * @brief Compute the final control variable uk.
 * @return True if successful.
 */
bool Chorus::OTG::computeUk_( ) {
    double uv_min = computeUv_( min_velocity_ );
    double uv_max = computeUv_( max_velocity_ );
    uk_ = std::max( uv_min, std::min( uc_, uv_max ) );

    // Critical Deceleration Braking Envelope Guard:
    // When decelerating to rest (target_velocity == 0), the maximum achievable deceleration without
    // velocity zero-crossing overshoot is: |a_crit| = sqrt(2 * J_max * |v|).
    // If |a| >= |a_crit|, applying maximum opposing jerk is required to ramp acceleration back to zero
    // simultaneously as velocity reaches zero, preventing negative velocity ripples.
    if ( error_vel_ > 0.0 && error_acc_ < 0.0 && std::abs( params_.target_velocity ) < EPSILON ) {
        const double a_crit = -std::sqrt( 2.0 * std::abs( error_vel_ ) );
        if ( error_acc_ <= a_crit ) {
            uk_ = std::max( uk_, U_ );
        }
    }
    else if ( error_vel_ < 0.0 && error_acc_ > 0.0 && std::abs( params_.target_velocity ) < EPSILON ) {
        const double a_crit = std::sqrt( 2.0 * std::abs( error_vel_ ) );
        if ( error_acc_ >= a_crit ) {
            uk_ = std::min( uk_, -U_ );
        }
    }

    uk_ = std::max( -U_, std::min( U_, uk_ ) );
    return true;
}

/**
 * @brief Compute the velocity control variable for a given velocity constraint.
 * @param vel The velocity constraint.
 * @return The computed control variable.
 */
double Chorus::OTG::computeUv_( double& vel ) {
    double ua_min = computeUa_( min_acceleration_ );
    double ua_max = computeUa_( max_acceleration_ );
    double ucv = computeUcv_( vel );

    uv_ = std::max( ua_min, std::min( ucv, ua_max ) );
    return uv_;
}

/**
 * @brief Compute the control variable for a given velocity (ucv).
 * @param vel The velocity constraint.
 * @return The computed ucv value.
 */
double Chorus::OTG::computeUcv_( double& vel ) {
    double delta_v = computeDeltaV_( vel );
    const double dt = params_.sampling_rate;
    const double phi_v = std::max( EPSILON, 2.0 * std::abs( max_acceleration_ ) * dt + ( dt * dt ) );
    const double val = delta_v + ( 1.0 - std::abs( getSign( delta_v ) ) ) * error_acc_;
    ucv_ = -U_ * sat_( val, phi_v );
    return ucv_;
}

/**
 * @brief Compute the delta value for a given velocity.
 * @param vel The velocity constraint.
 * @return The computed delta_v value.
 */
double Chorus::OTG::computeDeltaV_( double& vel ) {
    delta_v_ = ( error_acc_ * std::abs( error_acc_ ) ) + ( 2.0 * ( error_vel_ - vel ) );
    return delta_v_;
}

/**
 * @brief Compute the control variable for a given acceleration constraint.
 * @param acc The acceleration constraint.
 * @return The computed ua value.
 */
double Chorus::OTG::computeUa_( double& acc ) {
    const double diff = acc - error_acc_;
    const double dt = params_.sampling_rate;
    if ( dt > EPSILON ) {
        const double j = ( diff / dt ) * U_;
        ua_ = std::max( -U_, std::min( U_, j ) );
    }
    else {
        ua_ = -U_ * getSign( error_acc_ - acc );
    }
    return ua_;
}

/**
 * @brief Apply the C3 nonlinear filter to the control variable.
 * @return True if successful, false otherwise.
 */
bool Chorus::OTG::nonLinearFilterC3_( ) {

    // Apply C3 nonlinear filter to the control variable
    computeNormalizedError_( );
    updateMotionConstraints_( );

    // computing delta
    delta_ = ( error_vel_ + ( ( error_acc_ * std::abs( error_acc_ ) ) / 2.0 ) );
    sign_ = getSignOfDelta_( delta_ );

    // computing sigma
    sigma_ = computeSigma_( );

    // computing mu_positive and mu_negative
    mu_positive_ = computeMuPositive_( );
    mu_negative_ = computeMuNegative_( );

    // computing summation
    summation_ = computeSummation_( );

    sign_summation_ = getSignOfSummation_( summation_ );

    // computing uc
    if ( !computeUc_( ) ) {
        return false;
    }

    // computing uk
    computeUk_( );

    // avoid the residual that may explode because of chattering
    if ( std::abs( params_.max_jerk ) < EPSILON ) {
        prev_acc_ = 0;
    }

    integrateControlVariable_( );
    return true;
}

/**
 * @brief Integrate the control variable to update position, velocity, and acceleration.
 */
void Chorus::OTG::integrateControlVariable_( ) {
    // Direct discrete-time integration using the current step's computed jerk (uk_)
    // to eliminate 1-step phase lag and limit-cycle oscillation
    output_.acceleration = ( prev_acc_ + params_.sampling_rate * uk_ );
    output_.velocity = ( prev_vel_ + ( ( params_.sampling_rate * 0.5 ) * ( output_.acceleration + prev_acc_ ) ) );
    output_.position = ( prev_pos_ + ( ( params_.sampling_rate * 0.5 ) * ( output_.velocity + prev_vel_ ) ) );
    output_.jerk = uk_;

    // Terminal deadband settling check to eliminate discrete-time limit-cycle chattering and velocity ripple
    const double pos_error = std::abs( output_.position - params_.target_position );
    const double vel_error = std::abs( output_.velocity - params_.target_velocity );
    const double acc_error = std::abs( output_.acceleration - params_.target_acceleration );

    const double dt = params_.sampling_rate;
    const double max_jerk = std::abs( params_.max_jerk );
    const double max_acc = std::abs( params_.max_acceleration );
    const double max_vel = std::abs( params_.max_velocity );

    const double acc_tolerance = std::max( 1e-7, 2.0 * max_jerk * dt );
    const double vel_tolerance = std::max( 1e-7, max_jerk * dt * dt + 1.5 * max_acc * dt );
    const double pos_tolerance = std::max( 1e-7, max_vel * dt + 0.5 * max_acc * dt * dt );

    if ( pos_error <= pos_tolerance && vel_error <= vel_tolerance && acc_error <= acc_tolerance ) {
        output_.position = params_.target_position;
        output_.velocity = params_.target_velocity;
        output_.acceleration = params_.target_acceleration;
        output_.jerk = 0.0;
    }

    passToInput( output_ );
}

/**
 * @brief Continuous saturation function for discrete-time sliding mode boundary layers.
 * @param val Input value to be saturated.
 * @param phi Boundary layer thickness.
 * @return Saturated value in [-1, 1].
 */
double Chorus::OTG::sat_( double val, double phi ) {
    if ( phi <= EPSILON ) {
        return static_cast<double>( getSign( val ) );
    }
    const double ratio = val / phi;
    if ( ratio > 1.0 ) {
        return 1.0;
    }
    if ( ratio < -1.0 ) {
        return -1.0;
    }
    return ratio;
}

/**
 * @brief Get the sign of delta.
 * @param delta The delta value.
 * @return 1 if positive, -1 if negative, 0 if zero.
 */
int Chorus::OTG::getSignOfDelta_( const double& delta ) {
    if ( delta > EPSILON ) {
        return 1;
    }
    else if ( delta < -EPSILON ) {
        return -1;
    }
    return 0;
}

/**
 * @brief Get the sign of the summation.
 * @param summation The summation value.
 * @return 1 if positive, -1 if negative, 0 if zero.
 */
int Chorus::OTG::getSignOfSummation_( const double& summation ) {
    if ( summation > EPSILON ) {
        return 1;
    }
    else if ( summation < -EPSILON ) {
        return -1;
    }
    return 0;
}

/**
 * @brief Get the sign of a value.
 * @param value The value to check.
 * @return 1 if positive, -1 if negative, 0 if zero.
 */
int Chorus::OTG::getSign( double value ) {
    if ( value > EPSILON ) {
        return 1;
    }
    else if ( value < -EPSILON ) {
        return -1;
    }
    return 0;
}

/**
 * @brief Pass the output state to the input for the next iteration.
 * @param output The OTGOutput struct to use as the new input state.
 */
void Chorus::OTG::passToInput( Chorus::OTGOutput& output ) {
    prev_acc_ = output.acceleration;
    prev_vel_ = output.velocity;
    prev_pos_ = output.position;
    uk_prev_ = output.jerk;
}