/**
 * @file example_multi_dof_constraints_update.cpp
 * @author Aditya Singh (aditya.in753@gmail.com)
 * @brief This example will teach how to run a 6 dof system
 * @version 0.1
 * @date 2025-06-29
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "chorus.h"
#include "matplotlibcpp.h"

#define plt matplotlibcpp

 //! NOTE the constraints switching logic needs to be based on certain conditions that can be totally user defined
int main( ) {
    // setting up Chorus ,these are mandatory steps!
    Chorus::MultiDofOtg otg;
    Chorus::MultiDofOTGOutput output;
    Chorus::OTGConstraints constraints;
    Chorus::OTGTargetPosition target;
    Chorus::SystemStates states;
    Chorus::MultiDofOTGControllerGains gains;
    int dof = 6;
    otg.setDof( dof );

    // do not forget to resize
    target.resize( dof );
    gains.resize( dof );

    // setting up the gains ,0 means no controller
    for ( size_t i = 0; i < dof; i++ )
    {
        gains [i].kp = 0;
        gains [i].kd = 0;
        gains [i].ki = 0;

        // integeral clamp limits
        gains [i].upper_limit = 0;
        gains [i].lower_limit = 0;
    }
    otg.setGains( gains );

    // setting up the initial contraints 
    constraints.sampling_rate = 0.001;
    constraints.max_velocity = 2.0;
    constraints.min_velocity = -2.0;
    constraints.max_acceleration = 2.0;
    constraints.min_acceleration = -2.0;
    constraints.max_jerk = 4.0;
    constraints.min_jerk = -4.0;

    // setting up the initial state and target, on a real system initial state should be the current state of system based on feedback
    states.initial_position = { 0,0,0,0,0,0 };
    target = { 0,0,0,0,0,0 };

    std::vector<double>pos, vel, acc, jerk;
    std::vector<double>pos1, vel1, acc1, jerk1;
    std::vector<double>pos2, vel2, acc2, jerk2;
    std::vector<double>pos3, vel3, acc3, jerk3;
    std::vector<double>pos4, vel4, acc4, jerk4;
    std::vector<double>pos5, vel5, acc5, jerk5;

    double t = 0;

    while ( t < 49.1 ) {
        // go to the target with the constraints sets above
        if ( t > 0 && t < 10 ) {
            target = { 5,2,-5,-3,1,4 };
        }
        // updating target
        else if ( t > 10 && t < 15 ) {
            target = { 0,0 ,0,0,0,0 };

        }
        // updating target
        else if ( t > 15 && t < 25 ) {
            target = { -5,-2,0,6,-1,0 };

        }
        // updating target
        else if ( t > 25 && t < 45 ) {
            target = { 4,4,4,4,4,4 };
        }
        // updating target
        else {
            target = { 0,0,0,0,0,0 };
        }

        // getting the output for all dof's
        otg.getOutput( output );

        // updating the current state 
        states.initial_position = { output [0].position,output [1].position,output[2].position,output[3].position,output[4].position,output[5].position };

        // updating the system with desired data
        otg.update( constraints, target, states );

        // time updation
        t += constraints.sampling_rate;
        // std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) ); //? use on a real system

        // plotting related  stuff
        pos.emplace_back( output [0].position );
        vel.emplace_back( output [0].velocity );
        // acc.emplace_back( output [0].acceleration );
        pos1.emplace_back( output [1].position );
        vel1.emplace_back( output [1].velocity );
        // acc1.emplace_back( output [1].acceleration );
        pos2.emplace_back( output [2].position );
        vel2.emplace_back( output [2].velocity );
        // acc2.emplace_back( output [2].acceleration );
        pos3.emplace_back( output [3].position );
        vel3.emplace_back( output [3].velocity );
        // acc3.emplace_back( output [3].acceleration );
        pos4.emplace_back( output [4].position );
        vel4.emplace_back( output [4].velocity );
        // acc4.emplace_back( output [4].acceleration );
        pos5.emplace_back( output [5].position );
        vel5.emplace_back( output [5].velocity );
        // acc5.emplace_back( output [5].acceleration );

    }

    plt::plot( pos );
    plt::plot( vel );
    // plt::plot( acc );
    plt::plot( pos1 );
    plt::plot( vel1 );
    // plt::plot( acc1 );
    plt::plot( pos2 );
    plt::plot( vel2 );
    plt::plot( pos3 );
    plt::plot( vel3 );
    plt::plot( pos4 );
    plt::plot( vel4 );
    plt::plot( pos5 );
    plt::plot( vel5 );
    plt::grid( true );
    plt::show( );


    return 0;
}