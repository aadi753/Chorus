#include "otg.h"
#include <chrono>
#include <vector>
#include<matplotlibcpp.h>
#include <thread>
#include <Eigen/Dense>
#include <Eigen/Core>

#define plt matplotlibcpp

int main( ) {

    Chorus::OTG otg;
    Chorus::OTGParams params;
    Chorus::OTGOutput output;
    params.sampling_rate = 0.001; // 1 ms
    params.max_velocity = 2.0;
    params.max_acceleration = 2.0;
    params.max_jerk = 5.0;
    params.min_velocity = -3.0;
    params.min_acceleration = -2.0;
    // params.

    auto start = std::chrono::high_resolution_clock::now( );

    double t_elap = 0;
    otg.setTarget( params );

    std::vector<double> pos, vel, acc, jerk, target;
    Eigen::VectorXd t;
    t = t.LinSpaced( 20000, 0, 20 );

    for ( int i = 0; i < t.size( ); i++ ) {
        // auto ctime = std::chrono::high_resolution_clock::now( );
        // t_elap += static_cast< double >( std::chrono::duration<double, std::micro>( ctime - start ).count( ) ) * ( 1e-6 );
        t_elap = t [i];
        if ( t_elap >= 1 && t_elap < 7 && params.target_position != 8.0 ) {
            params.target_position += 8.0;
            otg.setTarget( params );
        }
        if ( t_elap >= 7 && t_elap < 9 && params.target_position != 10.0 ) {
            params.target_position += 2.0;
            // params.max_velocity = 4.0;
            // params.min_velocity = -4.0;
            // params.max_acceleration = 5.0;
            // params.min_acceleration = -5.0;
            otg.setTarget( params );
        }
        if ( t_elap >= 10 && t_elap < 12 && params.target_position != -2.0 ) {
            params.target_position += -12.0;
            // params.max_velocity = 2.0;
            // params.min_velocity = -1.5;
            // params.max_acceleration = 2.0;
            // params.min_acceleration = -2.0;
            otg.setTarget( params );
        }
        if ( t_elap >= 12 && t_elap <= 16 ) {
            // params.target_position += -12.0;
            // params.max_velocity = 2.0;
            params.min_velocity = -1.5;
            // params.max_acceleration = 1.5;
            // params.min_acceleration = -2.0;
            otg.setTarget( params );
        }
        if ( t_elap >= 16 && t_elap <= 20 ) {
            // params.target_position += -12.0;
            // params.max_velocity = 2.0;
            // params.min_velocity = -1.5;
            params.max_acceleration = 1.5;
            // params.min_acceleration = -2.0;
            otg.setTarget( params );
        }
        if ( t_elap < 1 ) {
            params.target_position = 0.0;
            otg.setTarget( params );
        }
        // std::cout << t_elap << "\n";
        otg.getTrajectory( output );
        // otg.passToInput( output );

        target.emplace_back( params.target_position );
        pos.emplace_back( output.position );
        vel.emplace_back( output.velocity );
        acc.emplace_back( output.acceleration );
        jerk.emplace_back( output.jerk );

        // std::cout << output.position << "\n";

        // std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) ); // simulate 1 ms delay
        // start = ctime;

    }
    std::cout << "exiting\n";
    // Plotting the results
    plt::plot( target );
    plt::plot( pos );
    plt::plot( vel );
    plt::plot( acc );
    // plt::plot( jerk );
    plt::grid( true );
    plt::show( );
    return 0;
}