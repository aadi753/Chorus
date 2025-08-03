#include "chorus.h"


Chorus::MultiDofOtg::MultiDofOtg( ) {
    // Constructor initializes the number of degrees of freedom to zero
}

Chorus::MultiDofOtg::~MultiDofOtg( ) {
    // Destructor does nothing for now
}

void Chorus::MultiDofOtg::setDof( const int dof ) {
    // Set the number of degrees of freedom and resize the vectors accordingly

    dof_ = dof;
    otg_.resize( dof_ );
    output_.resize( dof_ );
    final_otg_params_.resize( dof_ );
    params_.resize( dof_ );
    final_otg_params_.resize( dof_ );
    V0_.resize( dof_ );
    V1_.resize( dof_ );
    target_position_.resize( dof_ );
    system_states_.initial_position.resize( dof_ );
    system_states_.initial_velocity.resize( dof_ );
    system_states_.initial_acceleration.resize( dof_ );
    position_error_.resize( dof_ );
    integeral_error_.resize( dof_ );
}

//! DO NOT uncomment this shit :/
// void Chorus::MultiDofOtg::setTarget( const MultiDofOTGParams& params ) {
//     if ( !checkTargetupdate_( params ) ) {
//         params_ = params;

//         std::cout << "target parameters have changed, recomputing trajectory...\n";
//         recompute_( );
//         for ( int i = 0; i < dof_; i++ ) {
//             otg_ [i].setTarget( final_otg_params_ [i] );
//         }

//     }


// }


void Chorus::MultiDofOtg::update( const Chorus::OTGConstraints& constraints,
    const Chorus::OTGTargetPosition& target, Chorus::SystemStates& states ) {

    // set the system states
    setSystemStates_( states );

    //check for contraints updates
    checkForConstraintsUpdate_( constraints );
    // check for target updates
    checkForTargetUpdate_( target );

    if ( is_constraints_updated_ ) {
        constraints_ = constraints;
        should_recompute_ = true;
        is_constraints_updated_ = false;
    }

    if ( is_target_updated_ ) {
        target_position_ = target;
        should_recompute_ = true;
        is_target_updated_ = false;
    }

    // fill the params ,if nothing changes then the old params will be used 
    fillParams_( );
    if ( should_recompute_ ) {
        // recompute the new params for the dofs based on the constraints and target
        recompute_( );
        should_recompute_ = false;
        // since target is updated we need reset the integeral else it may cause jerks while using controller
        resetIntegeral_( );
    }

    // set the target for all OTGs
    for ( int i = 0; i < dof_; i++ ) {
        otg_ [i].setTarget( final_otg_params_ [i] );
    }

}


/**
 * @brief resets the integeral error for all DOFs.
 *
 */
void Chorus::MultiDofOtg::resetIntegeral_( ) {
    for ( int i = 0; i < dof_; i++ ) {
        integeral_error_ [i] = 0;
    }
}

/**
 * @brief sets the gains for all dofs
 *
 * @param gains
 */
void Chorus::MultiDofOtg::setGains( Chorus::MultiDofOTGControllerGains& gains ) {
    gains_ = gains;
}


/**
 * @brief computes the error for all dofs to be used by the controller
 *
 */
void Chorus::MultiDofOtg::computeError_( ) {

    for ( int i = 0; i < dof_; i++ ) {
        position_error_ [i] = output_ [i].position - system_states_.initial_position [i];
        integeral_error_ [i] += position_error_ [i];
    }
    clampIntegeralError_( );
}


/**
 * @brief clamps the integeral
 *
 */
void Chorus::MultiDofOtg::clampIntegeralError_( ) {
    for ( int i = 0; i < dof_; i++ ) {
        if ( integeral_error_ [i] > gains_ [i].upper_limit ) {
            integeral_error_ [i] = gains_ [i].upper_limit;
        }
        else if ( integeral_error_ [i] < gains_ [i].lower_limit ) {
            integeral_error_ [i] = gains_ [i].lower_limit;
        }
    }
}

/**
 * @brief gets the output from all otg's and performs the controller calculations if gains are set
 *
 * @param output
 */
void Chorus::MultiDofOtg::getOutput( MultiDofOTGOutput& output ) {
    output.resize( dof_ );
    for ( int i = 0; i < dof_; i++ ) {
        otg_ [i].getTrajectory( output_ [i] );
    }
    computeError_( );

    for ( size_t i = 0; i < dof_; i++ )
    {   // you can add the derivative term on your own if needed , but this works pretty good for the systems 

        //! use the velocity term if the system works in velocity control else use the position term directly of if needed then write a controller in the wrapper where this output will be used.
        output [i].position = output_ [i].position;
        output [i].velocity = output_ [i].velocity + ( gains_ [i].kp * position_error_ [i] ) + ( gains_ [i].ki * integeral_error_ [i] );
        output [i].acceleration = output_ [i].acceleration;
        output [i].jerk = output_ [i].jerk;


    }

    // output = output_;
}

// check for target updates
bool Chorus::MultiDofOtg::checkTargetupdate_( const MultiDofOTGParams& params ) {
    // Check if the target parameters have changed
    for ( int i = 0; i < dof_; i++ ) {
        if ( params [i].target_position != params_ [i].target_position ||
            params [i].max_velocity != params_ [i].max_velocity ||
            params [i].max_acceleration != params_ [i].max_acceleration ||
            params [i].max_jerk != params_ [i].max_jerk ||
            params [i].min_velocity != params_ [i].min_velocity ||
            params [i].min_acceleration != params_ [i].min_acceleration ||
            params [i].min_jerk != params_ [i].min_jerk ) {
            return false;
        }

    }
    return true;
}

/**
 * @brief computes  the trajectory durations based on constraints and target parameters.
 *
 * @return double
 */
double Chorus::MultiDofOtg::computeTrajectoryDuration_( ) {
    findMaxDisplacement_( );
    computeTJStar_( );
    computeJerkDurations_( );
    computeConstantVelocityDurations_( );
    // final_time_ = Ta_ + Tv_ + Td_;
    if ( DEBUG ) {

        std::cout << "final time: " << final_time_ << "\n";
    }

    return final_time_;
}

/**
 * @brief finds which dof has max displacement
 *
 */
void Chorus::MultiDofOtg::findMaxDisplacement_( ) {

    if ( diff_vec_.size( ) != 0 ) {
        diff_vec_.clear( );
    }

    for ( size_t i = 0; i < dof_; i++ )
    {
        diff_vec_.emplace_back( std::abs( params_ [i].target_position - params_ [i].initial_position ) );
    }

    displacement_ = *std::max_element( diff_vec_.begin( ), diff_vec_.end( ) );
    auto it = std::find( diff_vec_.begin( ), diff_vec_.end( ),
        displacement_ );

    maxDistIndex_ = it - diff_vec_.begin( );

    max_displacement_ = diff_vec_ [maxDistIndex_];

    if ( DEBUG ) {

        std::cout << "dof:" << maxDistIndex_ << "has max displacement" << "\n";
    }

    Vmax_ = params_ [maxDistIndex_].max_velocity;
    Vmin_ = params_ [maxDistIndex_].min_velocity;
    Amax_ = params_ [maxDistIndex_].max_acceleration;
    Amin_ = params_ [maxDistIndex_].min_acceleration;
    Jmax_ = params_ [maxDistIndex_].max_jerk;
    Jmin_ = params_ [maxDistIndex_].min_jerk;



}

/**
 * @brief computes tj_sta
 *
 */
void  Chorus::MultiDofOtg::computeTJStar_( ) {
    TJ_star_ = ( Amax_ / Jmax_ );

}

/**
 * @brief computes jerk durations
 *
 */
void Chorus::MultiDofOtg::computeJerkDurations_( ) {
    //? assuming case 1:
    Vlim_ = Vmax_;

    if ( ( Vmax_ - V0_ [maxDistIndex_] ) * Jmax_ < pow( Amax_, 2 ) ) {
        // std::cout << "Amax will not be reached..! \n";
        Tj1_ = sqrt( ( Vmax_ - V0_ [maxDistIndex_] ) / Jmax_ );
        Ta_ = 2 * Tj1_;
        // std::cout << "Tj1: " << Tj1_ << " || " << "Ta: " << Ta_ << "\n";
    }
    else {
        Tj1_ = Amax_ / Jmax_;
        Ta_ = Tj1_ + ( ( Vmax_ - V0_ [maxDistIndex_] ) / Amax_ );
        // std::cout << "Tj1: " << Tj1_ << " || " << "Ta: " << Ta_ << "\n";
    }

    if ( ( Vmax_ - V1_ [maxDistIndex_] ) * Jmax_ < pow( Amax_, 2 ) ) {
        // std::cout << "Amin will not be reached..!\n";
        Tj2_ = sqrt( ( Vmax_ - V1_ [maxDistIndex_] ) / Jmax_ );
        Td_ = 2 * Tj2_;
        // std::cout << "Tj2: " << Tj2_ << " || " << "Td: " << Td_ << "\n";

    }
    else {
        Tj2_ = Amax_ / Jmax_;
        Td_ = Tj2_ + ( ( Vmax_ - V1_ [maxDistIndex_] ) / Amax_ );
        // std::cout << "Tj2: " << Tj2_ << " || " << "Td: " << Td_ << "\n";
    }
}

/**
 * @brief computes constant velocity durations and if velocity is reachable then finds the optimal achievable velocity
 *
 */
void Chorus::MultiDofOtg::computeConstantVelocityDurations_( ) {


    Tv_ = ( displacement_ / Vmax_ ) - ( ( Ta_ / 2 ) * ( 1 + V0_ [maxDistIndex_] / Vmax_ ) ) - ( ( Td_ / 2 ) * ( 1 + V1_ [maxDistIndex_] / Vmax_ ) );
    // std::cout << "time for const velocity segment: " << Tv_<<"\n";
    if ( Tv_ > 0 )   // means our assumption of vlim=Vmax was right and can continue with the above values
    {
        // std::cout << "max vel will be reached.. \n";
    }

    else {   // means our assumption of Vlim=Vmax was wrong and we have to recalculate using case 2.
        Tv_ = 0;
        // std::cout << "\ncase2:\n";
        double delta =
            ( pow( Amax_, 4 ) / pow( Jmax_, 2 ) ) + ( 2 * ( pow( V0_ [maxDistIndex_], 2 ) + pow( V1_ [maxDistIndex_], 2 ) ) ) +
            Amax_ * ( 4 * ( displacement_ ) -( ( 2 * ( Amax_ / Jmax_ ) ) * ( V0_ [maxDistIndex_] + V1_ [maxDistIndex_] ) ) );

        Tj1_ = Tj2_ = Amax_ / Jmax_;
        Ta_ = ( ( pow( Amax_, 2 ) / Jmax_ ) - ( 2 * V0_ [maxDistIndex_] ) + sqrt( delta ) ) / ( 2 * Amax_ );
        Td_ = ( ( pow( Amax_, 2 ) / Jmax_ ) - ( 2 * V1_ [maxDistIndex_] ) + sqrt( delta ) ) / ( 2 * Amax_ );

        if ( Ta_ < 2 * Tj1_ || Td_ < 2 * Tj1_ ) {
            for ( Y_ = 1; Y_ > 0; Y_ -= 0.01 ) {
                // std::cout << Y_ << "\n";
                Amax_ = Y_ * Amax_;
                delta =
                    ( pow( Amax_, 4 ) / pow( Jmax_, 2 ) ) +
                    ( 2 * ( pow( V0_ [maxDistIndex_], 2 ) + pow( V1_ [maxDistIndex_], 2 ) ) ) +
                    Amax_ * ( 4 * ( displacement_ ) -( 2 * ( Amax_ / Jmax_ ) * ( V0_ [maxDistIndex_] + V1_ [maxDistIndex_] ) ) );

                Tj1_ = Tj2_ = Amax_ / Jmax_;
                Ta_ = ( ( pow( Amax_, 2 ) / Jmax_ ) - ( 2 * V0_ [maxDistIndex_] ) + sqrt( delta ) ) / ( 2 * Amax_ );
                Td_ = ( ( pow( Amax_, 2 ) / Jmax_ ) - ( 2 * V1_ [maxDistIndex_] ) + sqrt( delta ) ) / ( 2 * Amax_ );
                if ( Ta_ > 2 * Tj1_ && Td_ > 2 * Tj1_ ) {
                    // std::cout << "auto adjustments done..\n";
                    break;
                }
                else if ( Ta_ < 0 ) {
                    Ta_ = 0;
                    Tj1_ = 0;
                    Td_ = ( 2 * displacement_ ) / ( V0_ [maxDistIndex_] + V1_ [maxDistIndex_] );
                    Tj2_ = ( ( Jmax_ * displacement_ ) -
                        ( sqrt( Jmax_ * ( Jmax_ * ( pow( displacement_, 2 ) ) +
                            ( pow( ( V0_ [maxDistIndex_] + V1_ [maxDistIndex_] ), 2 ) * ( V1_ [maxDistIndex_] - V0_ [maxDistIndex_] ) ) ) ) ) ) /
                        ( Jmax_ * ( V0_ [maxDistIndex_] + V1_ [maxDistIndex_] ) );
                    break;
                }
                else if ( Td_ < 0 ) {
                    Td_ = 0;
                    Tj2_ = 0;
                    Ta_ = ( 2 * displacement_ ) / ( V0_ [maxDistIndex_] + V1_ [maxDistIndex_] );
                    Tj1_ = ( ( Jmax_ * displacement_ ) -
                        ( sqrt( Jmax_ * ( Jmax_ * ( pow( displacement_, 2 ) ) +
                            ( pow( ( V0_ [maxDistIndex_] + V1_ [maxDistIndex_] ), 2 ) * ( V1_ [maxDistIndex_] - V0_ [maxDistIndex_] ) ) ) ) ) ) /
                        ( Jmax_ * ( V0_ [maxDistIndex_] + V1_ [maxDistIndex_] ) );
                    break;
                }
            }
        }
    }

    Alim_a_ = Jmax_ * Tj1_;
    Alim_d_ = -Jmax_ * Tj2_;
    Vlim_ = V1_ [maxDistIndex_] - ( Td_ - Tj2_ ) * Alim_d_;

    // std::cout << "Ta: " << Ta_ << " || "
    //      << "Td: " << Td_ << " || "
    //      << "Tv: " << Tv_ << " || "
    //      << "Tj1_: " << Tj1_ << " || "
    //      << "Tj2: " << Tj2_ << " || "
    //      << "Alim_a: " << Alim_a_ << " || "
    //      << "Alim_d: " << Alim_d_ << " || "
    //      << "Vlim_: " << Vlim_ << "\n";

    final_time_ = ( Ta_ + Tv_ + Td_ );


}

/**
 * @brief finds the optimal constraints for the other DOFs
 *
 */
void Chorus::MultiDofOtg::computeConstraintsFromTrajDuration_( ) {

    for ( size_t i = 0; i < dof_; i++ ) {
        if ( i == maxDistIndex_ ) {
            final_otg_params_ [i].sampling_rate = params_ [i].sampling_rate;
            final_otg_params_ [i].initial_position = params_ [i].initial_position;
            final_otg_params_ [i].target_position = params_ [i].target_position;
            final_otg_params_ [i].max_velocity = params_ [i].max_velocity;
            final_otg_params_ [i].min_velocity = params_ [i].min_velocity;
            final_otg_params_ [i].max_acceleration = params_ [i].max_acceleration;
            final_otg_params_ [i].min_acceleration = params_ [i].min_acceleration;
            final_otg_params_ [i].max_jerk = params_ [i].max_jerk;
            final_otg_params_ [i].min_jerk = params_ [i].min_jerk;
        }
        else {
            Vmax_ = ( diff_vec_ [i] ) / ( ( 1 - ALPHA ) * final_time_ );
            Amax_ =
                diff_vec_ [i] / ( ALPHA * ( 1 - ALPHA ) * ( 1 - BETA ) * pow( final_time_, 2 ) );
            Jmax_ = diff_vec_ [i] / ( pow( ALPHA, 2 ) * BETA * ( 1 - ALPHA ) * ( 1 - BETA ) * pow( final_time_, 3 ) );
            Vlim_ = Vmax_;
            Alim_a_ = Amax_;
            Alim_d_ = -Alim_a_;
            Jmin_ = -Jmax_;
            Ta_ = ALPHA * final_time_;
            Td_ = Ta_;
            Tj1_ = Tj2_ = BETA * Ta_;
            Tv_ = final_time_ - ( 2 * Ta_ );


            final_otg_params_ [i].sampling_rate = params_ [i].sampling_rate;
            final_otg_params_ [i].initial_position = params_ [i].initial_position;
            final_otg_params_ [i].target_position = params_ [i].target_position;
            final_otg_params_ [i].max_velocity = Vmax_;
            final_otg_params_ [i].min_velocity = -Vmax_;
            final_otg_params_ [i].max_acceleration = Alim_a_;
            final_otg_params_ [i].min_acceleration = -Alim_a_;
            final_otg_params_ [i].max_jerk = Jmax_;
            final_otg_params_ [i].min_jerk = -Jmax_;


        }

        if ( DEBUG ) {

            std::cout << "final params for dof: " << i << "\n";
            std::cout << "sampling rate: " << final_otg_params_ [i].sampling_rate << "\n";
            std::cout << "initial position: " << final_otg_params_ [i].initial_position << "\n";
            std::cout << "target position: " << final_otg_params_ [i].target_position << "\n";
            std::cout << "max velocity: " << final_otg_params_ [i].max_velocity << "\n";
            std::cout << "min velocity: " << final_otg_params_ [i].min_velocity << "\n";
            std::cout << "max acceleration: " << final_otg_params_ [i].max_acceleration << "\n";
            std::cout << "min acceleration: " << final_otg_params_ [i].min_acceleration << "\n";
            std::cout << "max jerk: " << final_otg_params_ [i].max_jerk << "\n";
            std::cout << "min jerk: " << final_otg_params_ [i].min_jerk << "\n";
            std::cout << "----------------------------------------------------\n";
        }

    }

}

/**
 * @brief recomputes the trajectory and constraints based on the current target and constraints.
 *
 */
void Chorus::MultiDofOtg::recompute_( ) {
    computeTrajectoryDuration_( );
    computeConstraintsFromTrajDuration_( );


}

/**
 * @brief checks for the updates in the constraints
 *
 * @param constraints
 */
void Chorus::MultiDofOtg::checkForConstraintsUpdate_( const Chorus::OTGConstraints& constraints ) {
    if ( constraints_.sampling_rate != constraints.sampling_rate ||
        constraints_.max_velocity != constraints.max_velocity ||
        constraints_.max_acceleration != constraints.max_acceleration ||
        constraints_.max_jerk != constraints.max_jerk ||
        constraints_.min_velocity != constraints.min_velocity ||
        constraints_.min_acceleration != constraints.min_acceleration ||
        constraints_.min_jerk != constraints.min_jerk ) {

        constraints_ = constraints;
        is_constraints_updated_ = true;
    }

}


/**
 * @brief checks for the updates in the target position
 *
 * @param target
 */
void Chorus::MultiDofOtg::checkForTargetUpdate_( const Chorus::OTGTargetPosition& target ) {
    for ( size_t i = 0; i < dof_; i++ )
    {
        if ( target_position_ [i] != target [i] ) {
            // std::cout << "target position for dof: " << i << " has changed\n";
            is_target_updated_ = true;
            target_position_ [i] = target [i];
        }
    }

}

/**
 * @brief fills the parameters to be used by the OTG for each dof.
 *
 */
void Chorus::MultiDofOtg::fillParams_( ) {
    for ( size_t i = 0; i < dof_; i++ )
    {
        params_ [i].sampling_rate = constraints_.sampling_rate;
        params_ [i].max_velocity = constraints_.max_velocity;
        params_ [i].max_acceleration = constraints_.max_acceleration;
        params_ [i].max_jerk = constraints_.max_jerk;
        params_ [i].min_velocity = constraints_.min_velocity;
        params_ [i].min_acceleration = constraints_.min_acceleration;
        params_ [i].min_jerk = constraints_.min_jerk;
        params_ [i].initial_position = system_states_.initial_position [i];
        params_ [i].target_position = target_position_ [i];

    }

}