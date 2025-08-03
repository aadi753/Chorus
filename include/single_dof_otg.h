/**
 * @file single_dof_otg.h
 * @author Aditya Singh (aditya.in753@gmail.com)
 * @brief  Header file for the Online Trajectory Generator (OTG). for single degree of freedom.
 * @version 0.1
 * @date 2025-06-23
 *
    Chorus - Multi-DOF Online Trajectory Generator
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
 *
 */

#ifndef SINGLE_DOF_OTG_H_
#define SINGLE_DOF_OTG_H_

#include <iostream>
#include <math.h>
#include "chorus_params.h"
#include "config.h"
 namespace Chorus{

    /**
     * @brief Class for Online Trajectory Generation (OTG) for a single degree of freedom.
     *
     * This class implements a C3 nonlinear filter for trajectory generation,
     * supporting jerk, acceleration, velocity and position constraints.
     *
     * The role of this class is to compute the optimal trajectory based on target parameters and constraints,
        it generates a constant jerk which is then used to compute the acceleration, velocity and position of the system.
     */
    class OTG {
    
        public:
        /**
         * @brief Default constructor.
         */
        OTG( ) { }
        /**
         * @brief Destructor.
         */
        ~OTG( ) { }

        /**
         * @brief Set the target parameters for trajectory generation.
         * @param params The OTGParams struct containing target and constraint values.
         */
        void setTarget( const Chorus::OTGParams& params ) {
            params_ = params;
            // if ( flag ) {
                // prev_pos_ = params_.initial_position;
                // prev_vel_ = params_.initial_velocity;
            //     flag = false;
            // }
        }

        /**
         * @brief Compute the next trajectory point and update the output.
         * @param output Reference to OTGOutput struct to be filled with the result.
         */
        void getTrajectory( Chorus::OTGOutput& output ) {
            nonLinearFilterC3_( );
            output = output_;
        }

        /**
         * @brief Pass the output state to the input for the next iteration.
         * @param output The OTGOutput struct to use as the new input state.
         */
        void passToInput( Chorus::OTGOutput& output );
    private:
        /**
         * @brief Compute normalized errors for position, velocity, and acceleration.
         */
        void computeNormalizedError_( );
        /**
         * @brief Update motion constraints based on target parameters.
         */
        void updateMotionConstraints_( );
        /**
         * @brief Apply the C3 nonlinear filter to compute the control variable.
         * @return True if successful, false otherwise.
         */
        bool nonLinearFilterC3_( );
        /**
         * @brief Compute the control variable uc.
         * @return True if successful.
         */
        bool computeUc_( );
        /**
         * @brief Compute the velocity control variable for a given velocity constraint.
         * @param vel The velocity constraint.
         * @return The computed control variable.
         */
        double computeUv_( double& vel );
        /**
         * @brief Compute the final control variable uk.
         * @return True if successful.
         */
        bool computeUk_( );
        /**
         * @brief Compute the control variable for a given velocity (ucv).
         * @param vel The velocity constraint.
         * @return The computed ucv value.
         */
        double computeUcv_( double& vel );
        /**
         * @brief Compute the delta value for a given velocity.
         * @param vel The velocity constraint.
         * @return The computed delta_v value.
         */
        double computeDeltaV_( double& vel );
        /**
         * @brief Compute the control variable for a given acceleration constraint.
         * @param acc The acceleration constraint.
         * @return The computed ua value.
         */
        double computeUa_( double& acc );
        /**
         * @brief Get the sign of delta.
         * @param delta The delta value.
         * @return 1 if positive, -1 if negative, 0 if zero.
         */
        int getSignOfDelta_( const double& delta );
        /**
         * @brief Integrate the control variable to update position, velocity, and acceleration.
         */
        void integrateControlVariable_( );
        /**
         * @brief Compute the sigma parameter for the C3 filter.
         * @return The computed sigma value.
         */
        double computeSigma_( );
        /**
         * @brief Compute the mu_positive parameter for the C3 filter.
         * @return The computed mu_positive value.
         */
        double computeMuPositive_( );
        /**
         * @brief Compute the mu_negative parameter for the C3 filter.
         * @return The computed mu_negative value.
         */
        double computeMuNegative_( );
        /**
         * @brief Compute the summation parameter for the C3 filter.
         * @return The computed summation value.
         */
        double computeSummation_( );
        /**
         * @brief Get the sign of the summation.
         * @param summation The summation value.
         * @return 1 if positive, -1 if negative, 0 if zero.
         */
        int getSignOfSummation_( const double& summation );
        /**
         * @brief Get the sign of a value.
         * @param value The value to check.
         * @return 1 if positive, -1 if negative, 0 if zero.
         */
        int getSign( double value );

        Chorus::OTGParams params_;
        Chorus::OTGOutput output_;

        // normalized error variables
        double error_pos_ = 0;
        double error_vel_ = 0;
        double error_acc_ = 0;

        // motion constraints , these are not the constraints for the actuator but the constrainst for the trajectory
        // and ofcourse these should be less than the actuator constraints
        double max_velocity_ = 0;
        double min_acceleration_ = 0;
        double min_velocity_ = 0;
        double max_acceleration_ = 0;

        double prev_acc_ = 0;
        double prev_vel_ = 0;
        double prev_pos_ = 0;

        bool flag = true; // to check if the initial position is set
        double U_ = 0; // max value of control variable uk
        double uk_ = 0; // control variable
        double uk_prev_ = 0; // previous control variable
        int sign_ = 0;
        int sign_summation_ = 0;
        double uc_ = 0;
        double delta_ = 0; // C3 nonlinear filter parameter
        double sigma_ = 0; // C3 nonlinear filter parameter
        double mu_positive_ = 0; // C3 nonlinear filter parameter
        double mu_negative_ = 0; // C3 nonlinear filter parameter
        double summation_ = 0; // C3 nonlinear filter parameter

        double uv_ = 0; //Cv non linear filter parameter
        double ucv_ = 0; //Cv non linear filter parameter
        double delta_v_ = 0; //Cv non linear filter parameter
        double ua_ = 0; //Cv non linear filter parameter

    };
};

#endif /* SINGLE_DOF_OTG_H_ */
