#ifndef MULTI_DOF_OTG_H_
#define MULTI_DOF_OTG_H_

#include "otg.h"
#include <bits/stdc++.h>

namespace OnlineTraj {

    /**
     * @brief Multi-DOF Online Trajectory Generator.
     *
     * This class manages multiple single-DOF OTGs to generate synchronized trajectories for multi-DOF systems.
     */
    class MultiDofOtg {
    public:
        /**
         * @brief Constructor.
         */
        MultiDofOtg( );
        /**
         * @brief Destructor.
         */
        ~MultiDofOtg( );

        /**
         * @brief Set the number of degrees of freedom.
         * @param dof Number of DOFs.
         */
        void setDof( const int dof );
        /**
         * @brief Get the current output for all DOFs.
         * @param output Reference to output vector to be filled.
         */
        void getOutput( MultiDofOTGOutput& output );

        /**
         * @brief Update the trajectory generator with new constraints, targets, and system states.
         * @param constraints The constraints for all DOFs.
         * @param target The target positions for all DOFs.
         * @param states The current system states.
         */
        void update( const OnlineTraj::OTGConstraints& constraints, const OnlineTraj::OTGTargetPosition& target, OnlineTraj::SystemStates& states );

        /**
         * @brief Set the controller gains for all DOFs.
         * @param gains The controller gains.
         */
        void setGains( OnlineTraj::MultiDofOTGControllerGains& gains );

    private:
        /**
         * @brief Check if the target parameters have changed.
         * @param params The new target parameters.
         * @return True if unchanged, false if changed.
         */
        bool checkTargetupdate_( const MultiDofOTGParams& params );
        /**
         * @brief Compute the total trajectory duration for synchronization.
         * @return The computed duration.
         */
        double computeTrajectoryDuration_( );
        /**
         * @brief Find the DOF with the maximum displacement.
         */
        void findMaxDisplacement_( );
        /**
         * @brief Compute constraints for all DOFs based on the synchronized trajectory duration.
         */
        void computeConstraintsFromTrajDuration_( );
        /**
         * @brief Recompute the trajectory and constraints.
         */
        void recompute_( );
        /**
         * @brief Check if the constraints have changed.
         * @param constraints The new constraints.
         */
        void checkForConstraintsUpdate_( const OnlineTraj::OTGConstraints& constraints );
        /**
         * @brief Check if the target positions have changed.
         * @param target_position The new target positions.
         */
        void checkForTargetUpdate_( const OnlineTraj::OTGTargetPosition& target_position );
        /**
         * @brief Fill the OTG parameters for each DOF from constraints and targets.
         */
        void fillParams_( );
        /**
         * @brief Compute the error for each DOF (for control).
         */
        void computeError_( );

        /**
         * @brief Set the system states for all DOFs.
         * @param states The new system states.
         */
        void setSystemStates_( const OnlineTraj::SystemStates& states ) {
            system_states_ = states;
        }
        /**
         * @brief Compute the TJStar_ internal variable.
         */
        void  computeTJStar_( );
        /**
         * @brief Compute the jerk segment durations.
         */
        void computeJerkDurations_( );
        /**
         * @brief Compute the constant velocity segment durations.
         */
        void computeConstantVelocityDurations_( );
        /**
         * @brief Reset the integral error (for control).
         */
        void resetIntegeral_( );
        /**
         * @brief Clamp the integral error (for control).
         */
        void clampIntegeralError_( );

        // position error 
        std::vector<double>position_error_;

        // integeral error for control
        std::vector<double>integeral_error_;;

        // controller gains for all DOFs
        OnlineTraj::MultiDofOTGControllerGains gains_;

        //params for all dofs
        MultiDofOTGParams params_;

        // final otg params for all dofs after recomputation
        MultiDofOTGParams final_otg_params_;


        // difference vector
        std::vector<double>diff_vec_;

        //constraints for all dofs
        OnlineTraj::OTGConstraints constraints_;

        // target position for all dofs
        OnlineTraj::OTGTargetPosition target_position_;

        // system states for all dofs
        OnlineTraj::SystemStates system_states_;

        // max displacement joint index and value
        int max_displacement_idx_;
        double max_displacement_;

        // number of dofs
        int dof_;

        // output of all otgs
        MultiDofOTGOutput output_;

        //vector of single DOF OTGs
        std::vector<OnlineTraj::OTG> otg_;

        // flag to check if the target parameters have changed
        bool is_target_updated_ = false;

        // flag to check if the constraints have changed
        bool is_constraints_updated_ = false;

        // flag to check if recomputation is needed
        bool should_recompute_ = false;

        // double s curve parameters
        double Ta_;          // time for acceleration phase
        double Tv_;          // time for constant velociy phase
        double Td_;          // time for deceleation phase
        double final_time_;   // total time of trajectory
        double Tj1_;    // time interval in which jerk is constant during acceleration
        double Tj2_;    // time interval in which jerk is costant during deceleration
        double Vmax_;   // max Velocity
        double Vmin_;   // min velocity
        double Amax_;   // max acceleration
        double Amin_;   // min acceleration
        double Jmax_;   // max Jerk
        double Jmin_;   // min jerk
        double Vlim_;   // velocity limit
        double Alim_a_;   // acceleration limit for acceleration phase
        double Alim_d_;   // acceleration limit for deceleration phase
        double Y_;       // gamma for converging to new values of internal variables
        double TJ_star_;      // gamma (ranges from 0-1)
        int maxDistIndex_;  // index of max displacement joint
        double displacement_;  // max displacement
        std::vector<double>V0_;
        std::vector<double>V1_;  // final velocity of the joint at the end of trajectory
    };
};

#endif /* MULTI_DOF_OTG_H_ */
