/**
 * @file chorus_params.h
 * @author Aditya Singh (aditya.in753@gmail.com)
 * @brie Header file for the Online Trajectory Generator (OTG) parameters and output structures.
 * @version 0.1
 * @date 2025-06-27
 *
 *
 * Chorus - Multi-DOF Online Trajectory Generator
 * Copyright (C) 2025 Aditya Singh
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
* /
*/

#ifndef CHORUS_PARAMAS_H_
#define CHORUS_PARAMAS_H_

#include <vector>

// this may make the computations slow as couts are time taking
#define DEBUG 0



namespace Chorus {

    /**
     * @brief Parameters for single-DOF OTG.
     */
    struct OTGParams {
        double sampling_rate = 0;         ///< Sampling rate (s)
        double max_velocity = 0;          ///< Maximum velocity
        double max_acceleration = 0;      ///< Maximum acceleration
        double max_jerk = 0;              ///< Maximum jerk
        double min_velocity = 0;          ///< Minimum velocity
        double min_acceleration = 0;      ///< Minimum acceleration
        double min_jerk = 0;              ///< Minimum jerk
        double initial_position = 0;      ///< Initial position
        double initial_velocity = 0;      ///< Initial velocity
        double target_position = 0;       ///< Target position
        double target_velocity = 0;       ///< Target velocity
        double target_acceleration = 0;   ///< Target acceleration
    };

    /**
     * @brief Output structure for single-DOF OTG.
     */
    struct OTGOutput {
        double position = 0;      ///< Current position
        double velocity = 0;      ///< Current velocity
        double acceleration = 0;  ///< Current acceleration
        double jerk = 0;          ///< Current jerk (control variable)
    };

    /**
     * @brief Constraints for OTG (shared across all DOFs).
     */
    struct OTGConstraints {
        double sampling_rate = 0; // sampling rate 
        double max_velocity = 0; // max velocity for trajectory
        double max_acceleration = 0; // max acceleration for trajectory
        double max_jerk = 0; // max jerk for trajectory
        double min_velocity = 0; // min velocity for trajectory
        double min_acceleration = 0; // min acceleration for trajectory
        double min_jerk = 0; // min jerk for trajectory
    };

    /**
     * @brief System state vectors for all DOFs.
     */
     struct SystemStates {
        std::vector<double> initial_position;      // Initial positions for all DOFs
        std::vector<double> initial_velocity;      // Initial velocities for all DOFs
        std::vector<double> initial_acceleration;  // Initial accelerations for all DOFs
    };

    /**
     * @brief Controller gains for a single DOF.
     */
    struct ControllerGains {
        double kp = 0;           ///< Proportional gain
        double kd = 0;           ///< Derivative gain
        double ki = 0;           ///< Integral gain
        double upper_limit = 0;  ///< Upper limit for control output
        double lower_limit = 0;  ///< Lower limit for control output
    };
    typedef std::vector<double> OTGTargetPosition; ///< Target positions for all DOFs



    typedef std::vector<OTGParams> MultiDofOTGParams; // OTG parameters for all DOFs
    typedef std::vector<OTGOutput> MultiDofOTGOutput; // OTG outputs for all DOFs
    typedef std::vector<ControllerGains> MultiDofOTGControllerGains; // Controller gains for all DOFs
};
#endif /* CHORUS_PARAMS_H_ */
