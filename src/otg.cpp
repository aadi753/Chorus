#include "single_dof_otg.h"

/**
 * @brief Compute normalized errors for position, velocity, and acceleration.
 */
void Chorus::OTG::computeNormalizedError_( ) {
    // Compute normalized error based on the target parameters
    U_ = params_.max_jerk;
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

    double expr = summation_ + ( 1.0 - std::abs( sign_summation_ ) ) * ( delta_ + ( 1.0 - std::abs( sign_ ) ) * error_acc_ );
    uc_ = ( -U_ * getSign( expr ) );
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
    return true;

}

/**
 * @brief Compute the velocity control variable for a given velocity constraint.
 * @param vel The velocity constraint.
 * @return The computed control variable.
 */
double  Chorus::OTG::computeUv_( double& vel ) {
    double ua_min = computeUa_( min_acceleration_ );
    double ua_max = computeUa_( max_acceleration_ );
    double ucv = computeUcv_( vel );

    uv_ = std::max( ua_min, std::min( ucv, ua_max ) );
    // std::cout << "uv: " << uv_ << "\n";
    return uv_;


}

/**
 * @brief Compute the control variable for a given velocity (ucv).
 * @param vel The velocity constraint.
 * @return The computed ucv value.
 */
double Chorus::OTG::computeUcv_( double& vel ) {
    double delta_v = computeDeltaV_( vel );

    ucv_ = ( -U_ * getSign( delta_v + ( 1 - std::abs( getSign( delta_v ) ) ) * error_acc_ ) );
    // std::cout << "ucv: " << ucv_ << "\n";

    return ucv_;
}

/**
 * @brief Compute the delta value for a given velocity.
 * @param vel The velocity constraint.
 * @return The computed delta_v value.
 */
double Chorus::OTG::computeDeltaV_( double& vel ) {
    delta_v_ = ( error_acc_ * std::abs( error_acc_ ) ) + ( 2 * ( error_vel_ - vel ) );
    // std::cout << "delta_v: " << delta_v_ << "\n";
    return delta_v_;
}

/**
 * @brief Compute the control variable for a given acceleration constraint.
 * @param acc The acceleration constraint.
 * @return The computed ua value.
 */
double Chorus::OTG::computeUa_( double& acc ) {
    // std::cout << ( error_acc_ - acc ) << "\n";
    ua_ = ( -U_ * getSign( error_acc_ - acc ) );
    // std::cout << "ua: " << ua_ << "\n";
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

    //avoid the residual that may explode because of chattering
    if ( std::abs( params_.max_jerk ) < EPSILON ) {
        // uk_prev_ = 0;
        prev_acc_ = 0;
    }
    // std::cout << "delta: " << delta_ << " sigma: " << sigma_ << " mu_positive: " << mu_positive_ << " mu_negative: " << mu_negative_ << " summation: " << summation_ << " sign_summation: " << sign_summation_ << " uc: " << uc_ << " uk: " << uk_ << "\n";
    integrateControlVariable_( );
    return true;
}

/**
 * @brief Integrate the control variable to update position, velocity, and acceleration.
 */
void Chorus::OTG::integrateControlVariable_( ) {
    output_.acceleration = ( prev_acc_ + params_.sampling_rate * uk_prev_ );
    // std::cout<< "acceleration: " << output_.acceleration << "\n";
    output_.velocity = ( prev_vel_ + ( ( params_.sampling_rate * 0.5 ) * ( output_.acceleration + prev_acc_ ) ) );
    output_.position = ( prev_pos_ + ( ( params_.sampling_rate * 0.5 ) * ( output_.velocity + prev_vel_ ) ) );
    output_.jerk = uk_;


    passToInput( output_ );


}

/**
 * @brief Get the sign of delta.
 * @param delta The delta value.
 * @return 1 if positive, -1 if negative, 0 if zero.
 */
int Chorus::OTG::getSignOfDelta_( const double& delta ) {
    if ( delta > 0 ) {
        return 1;
    }
    else if ( delta < 0 ) {
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
    if ( summation > 0 ) {
        return 1;
    }
    else if ( summation < 0 ) {
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
    if ( value > 0 ) {
        return 1;
    }
    else if ( value < 0 ) {
        return -1;
    }
    return 0;
}

/**
 * @brief Pass the output state to the input for the next iteration.
 * @param output The OTGOutput struct to use as the new input state.
 */
void Chorus::OTG::passToInput( Chorus::OTGOutput& output ) {
    // Pass the output to the input
    // params_.initial_position = output.position;
    // params_.target_position = output.position;
    // params_.target_velocity = output.velocity;
    // params_.target_acceleration = output.acceleration;

    prev_acc_ = output.acceleration;
    prev_vel_ = output.velocity;
    prev_pos_ = output.position;
    uk_prev_ = output.jerk;
    // output_ = output;
}