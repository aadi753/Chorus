#ifndef MULTI_DOF_OTG_H_
#define MULTI_DOF_OTG_H_


#include "otg.h"

#include <bits/stdc++.h>


namespace OnlineTraj {

    class MultiDofOtg {
    public:
        MultiDofOtg( );
        ~MultiDofOtg( );

        void setDof( const int dof );
        void getOutput( MultiDofOTGOutput& output );

        void update( const OnlineTraj::OTGConstraints& constraints, const OnlineTraj::OTGTargetPosition& target, OnlineTraj::SystemStates& states );


    private:
        bool checkTargetupdate_( const MultiDofOTGParams& params );
        double computeTrajectoryDuration_( );
        void findMaxDisplacement_( );
        void computeConstraintsFromTrajDuration_( );
        void recompute_( );
        void checkForConstraintsUpdate_( const OnlineTraj::OTGConstraints& constraints );
        void checkForTargetUpdate_( const OnlineTraj::OTGTargetPosition& target_position );
        void fillParams_( );


        void setSystemStates_( const OnlineTraj::SystemStates& states ) {
            system_states_ = states;
        }
        /**
         * @brief computes the internal variable TJStar_
         *
         * @return true
         * @return false
         */
        void  computeTJStar_( );

        /**
         * @brief computes the Jerk segement durations
         *
         * @return true
         * @return false
         */
        void computeJerkDurations_( );
        void computeConstantVelocityDurations_( );

        MultiDofOTGParams params_;
        MultiDofOTGParams final_otg_params_;

        std::vector<double>diff_vec_;
        OnlineTraj::OTGConstraints constraints_;
        OnlineTraj::OTGTargetPosition target_position_;
        OnlineTraj::SystemStates system_states_;
        int max_displacement_idx_;
        double max_displacement_;
        int dof_;
        MultiDofOTGOutput output_;
        std::vector<OnlineTraj::OTG> otg_;

        bool is_target_updated_ = false;
        bool is_constraints_updated_ = false;
        bool should_recompute_ = false;
        // s curve parameters
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
