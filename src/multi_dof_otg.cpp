#include "multi_dof_otg.h"


OnlineTraj::MultiDofOtg::MultiDofOtg( ) {
    // Constructor initializes the number of degrees of freedom to zero
}

OnlineTraj::MultiDofOtg::~MultiDofOtg( ) {
    // Destructor does nothing for now
}

void OnlineTraj::MultiDofOtg::setDof( const int dof ) {
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
}

// void OnlineTraj::MultiDofOtg::setTarget( const MultiDofOTGParams& params ) {
//     if ( !checkTargetupdate_( params ) ) {
//         params_ = params;

//         std::cout << "target parameters have changed, recomputing trajectory...\n";
//         recompute_( );
//         for ( int i = 0; i < dof_; i++ ) {
//             otg_ [i].setTarget( final_otg_params_ [i] );
//         }

//     }


// }


void OnlineTraj::MultiDofOtg::update( const OnlineTraj::OTGConstraints& constraints,
    const OnlineTraj::OTGTargetPosition& target, OnlineTraj::SystemStates& states ) {

    // updateConstraits_( constraints );
    // updateTargetPosition_( target );
    setSystemStates_( states );

    checkForConstraintsUpdate_( constraints );
    checkForTargetUpdate_( target );

    if ( is_constraints_updated_ ) {
        constraints_ = constraints;
        std::cout << "here\n\n";
        should_recompute_ = true;
        is_constraints_updated_ = false;
    }

    if ( is_target_updated_ ) {
        target_position_ = target;
        std::cout << "herer2\n\n";
        should_recompute_ = true;
        is_target_updated_ = false;
    }

    fillParams_( );
    if ( should_recompute_ ) {
        recompute_( );
        should_recompute_ = false;
    }

    // set the target for all OTGs
    for ( int i = 0; i < dof_; i++ ) {
        otg_ [i].setTarget( final_otg_params_ [i] );
    }

}




void OnlineTraj::MultiDofOtg::getOutput( MultiDofOTGOutput& output ) {
    for ( int i = 0; i < dof_; i++ ) {
        otg_ [i].getTrajectory( output_ [i] );
    }
    output = output_;
}

bool OnlineTraj::MultiDofOtg::checkTargetupdate_( const MultiDofOTGParams& params ) {
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

double OnlineTraj::MultiDofOtg::computeTrajectoryDuration_( ) {
    findMaxDisplacement_( );
    computeTJStar_( );
    computeJerkDurations_( );
    computeConstantVelocityDurations_( );
    // final_time_ = Ta_ + Tv_ + Td_;
    std::cout << "final time: " << final_time_ << "\n";

    return final_time_;
}

void OnlineTraj::MultiDofOtg::findMaxDisplacement_( ) {

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

    std::cout << maxDistIndex_ << "\n";

    Vmax_ = params_ [maxDistIndex_].max_velocity;
    Vmin_ = params_ [maxDistIndex_].min_velocity;
    Amax_ = params_ [maxDistIndex_].max_acceleration;
    Amin_ = params_ [maxDistIndex_].min_acceleration;
    Jmax_ = params_ [maxDistIndex_].max_jerk;
    Jmin_ = params_ [maxDistIndex_].min_jerk;



}


void  OnlineTraj::MultiDofOtg::computeTJStar_( ) {
    TJ_star_ = ( Amax_ / Jmax_ );

}
void OnlineTraj::MultiDofOtg::computeJerkDurations_( ) {
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

void OnlineTraj::MultiDofOtg::computeConstantVelocityDurations_( ) {


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


void OnlineTraj::MultiDofOtg::computeConstraintsFromTrajDuration_( ) {
    double alpha = 0.4, beta = 0.2;   //? do not change these values unless you know what they do!!.

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
            Vmax_ = ( diff_vec_ [i] ) / ( ( 1 - alpha ) * final_time_ );
            Amax_ =
                diff_vec_ [i] / ( alpha * ( 1 - alpha ) * ( 1 - beta ) * pow( final_time_, 2 ) );
            Jmax_ = diff_vec_ [i] / ( pow( alpha, 2 ) * beta * ( 1 - alpha ) * ( 1 - beta ) * pow( final_time_, 3 ) );
            Vlim_ = Vmax_;
            Alim_a_ = Amax_;
            Alim_d_ = -Alim_a_;
            Jmin_ = -Jmax_;
            Ta_ = alpha * final_time_;
            Td_ = Ta_;
            Tj1_ = Tj2_ = beta * Ta_;
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
void OnlineTraj::MultiDofOtg::recompute_( ) {
    computeTrajectoryDuration_( );
    computeConstraintsFromTrajDuration_( );


}

void OnlineTraj::MultiDofOtg::checkForConstraintsUpdate_( const OnlineTraj::OTGConstraints& constraints ) {
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

void OnlineTraj::MultiDofOtg::checkForTargetUpdate_( const OnlineTraj::OTGTargetPosition& target ) {
    for ( size_t i = 0; i < dof_; i++ )
    {
        if ( target_position_ [i] != target [i] ) {
            std::cout << "target position for dof: " << i << " has changed\n";
            is_target_updated_ = true;
            target_position_ [i] = target [i];
        }
    }

}

void OnlineTraj::MultiDofOtg::fillParams_( ) {
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