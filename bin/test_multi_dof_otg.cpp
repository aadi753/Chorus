#include "multi_dof_otg.h"
#include "matplotlibcpp.h"

#define plt  matplotlibcpp 
#define DOF_ 3

int main( ) {
    // OnlineTraj::MultiDofOtg multi_dof_otg;
    // multi_dof_otg.setDof( DOF_ );

    // OnlineTraj::MultiDofOTGParams params;
    // params.resize( DOF_ );
    // // Set some example parameters for each degree of freedom
    // for ( int i = 0; i < DOF_; ++i ) {
    //     params [i].sampling_rate = 0.001; // 1 kHz sampling rate
    //     // params [i].initial_position = 0.0;
    //     // params [i].target_position = 0.01;
    //     params [i].max_velocity = 5.0;
    //     params [i].min_velocity = -5.0;
    //     params [i].max_acceleration = 3.4;
    //     params [i].min_acceleration = -3.4;
    //     params [i].max_jerk = 4.0;
    //     params [i].min_jerk = -4.0;
    // }

    // multi_dof_otg.setTarget( params );
    // OnlineTraj::MultiDofOTGOutput output;
    // double t = 0;
    // int joint_id = 0;
    // std::vector<double>pos, vel, acc, jerk;
    // std::vector<double>pos1, vel1, acc1, jerk1;
    // std::vector<double>pos2, vel2, acc2, jerk2;
    // bool flag = false;
    // while ( t < 28 ) {
    //     if ( t > 0.5 && t < 4 && !flag ) {
    //         //     // params [joint_id].sampling_rate = 0.001; // 1 kHz sampling rate
    //         params [joint_id].initial_position = output [joint_id].position;
    //         params [joint_id].target_position = -10.0;
    //         // params [joint_id].target_velocity = 2;
    //         // params [joint_id].target_acceleration = 2;

    //         params [joint_id + 1].initial_position = output [joint_id + 1].position;
    //         params [joint_id + 1].target_position = 8.0;

    //         params [joint_id + 2].initial_position = output [joint_id + 2].position;
    //         params [joint_id + 2].target_position = 12.0;
    //         // params [joint_id + 1].target_velocity = 2;
    //         // params [joint_id + 1].target_acceleration = 2;

    //         // params [joint_id + 1].max_velocity = 0.9;
    //         // params [joint_id + 1].min_velocity = -0.9;
    //         // params [joint_id + 1].max_acceleration = 1.0;
    //         // params [joint_id + 1].min_acceleration = -1.0;
    //         // params [joint_id + 1].max_jerk = 1;
    //         // params [joint_id + 1].min_jerk = -1;

    //         multi_dof_otg.setTarget( params );
    //         // t = 0;
    //         flag = true;
    //     }
    //     if ( t > 6 && flag ) {
    //         flag = false;
    //         params [joint_id].initial_position = output [joint_id].position;
    //         params [joint_id].target_position = -8.0;
    //         params [joint_id].max_velocity = 3.5;
    //         params [joint_id].min_velocity = -3.5;
    //         params [joint_id].max_acceleration = 7;
    //         params [joint_id].min_acceleration = -7;
    //         params [joint_id].max_jerk = 14;
    //         params [joint_id].min_jerk = -14;
    //         // params [joint_id].target_velocity = 2;
    //         // params [joint_id].target_acceleration = 2;

    //         params [joint_id + 1].initial_position = output [joint_id + 1].position;
    //         params [joint_id + 1].target_position = 2.0;

    //         params [joint_id + 2].initial_position = output [joint_id + 2].position;
    //         params [joint_id + 2].target_position = 2.0;
    //         // flag = false;
    //         multi_dof_otg.setTarget( params );
    //     }

    //     if ( t > 21 && !flag ) {
    //         flag = false;
    //         params [joint_id].initial_position = output [joint_id].position;
    //         params [joint_id].target_position = 0.0;
    //         params [joint_id].max_velocity = 1.5;
    //         params [joint_id].min_velocity = -1.5;
    //         params [joint_id].max_acceleration = 3;
    //         params [joint_id].min_acceleration = -3;
    //         params [joint_id].max_jerk = 3;
    //         params [joint_id].min_jerk = -3;
    //         // params [joint_id].target_velocity = 2;
    //         // params [joint_id].target_acceleration = 2;

    //         params [joint_id + 1].initial_position = output [joint_id + 1].position;
    //         params [joint_id + 1].target_position = 0.0;

    //         params [joint_id + 2].initial_position = output [joint_id + 2].position;
    //         params [joint_id + 2].target_position = 0.0;
    //         // flag = false;
    //         multi_dof_otg.setTarget( params );
    //     }
    //     multi_dof_otg.getOutput( output );
    //     int offset = 2;
    //     pos.emplace_back( output [joint_id + offset].position );
    //     vel.emplace_back( output [joint_id + offset].velocity );
    //     acc.emplace_back( output [joint_id + offset].acceleration );
    //     jerk.emplace_back( output [joint_id + offset].jerk );

    //     pos1.emplace_back( output [joint_id].position );
    //     vel1.emplace_back( output [joint_id].velocity );
    //     acc1.emplace_back( output [joint_id].acceleration );
    //     jerk1.emplace_back( output [joint_id].jerk );

    //     pos2.emplace_back( output [joint_id + 1].position );
    //     vel2.emplace_back( output [joint_id + 1].velocity );
    //     acc2.emplace_back( output [joint_id + 1].acceleration );
    //     jerk2.emplace_back( output [joint_id + 1].jerk );
    //     t += ( params [joint_id].sampling_rate );

    //     // std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
    // }
    // plt::plot( pos );
    // plt::plot( vel );
    // plt::plot( acc );
    // plt::plot( pos1 );
    // plt::plot( vel1 );
    // plt::plot( acc1 );
    // plt::plot( pos2 );
    // plt::plot( vel2 );
    // plt::plot( acc2 );

    // // plt::plot( jerk );
    // plt::show( );

    // return 0;
}