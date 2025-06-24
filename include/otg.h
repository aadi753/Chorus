#ifndef OTG_H_
#define OTG_H_

#include <iostream>
#include <math.h>
#include "otg_params.h"

namespace OnlineTraj {
    class OTG {
    public:
        OTG( ) { }
        ~OTG( ) { }

        void setTarget( const OnlineTraj::OTGParams& params ) {
            params_ = params;

            prev_pos_ = params_.initial_position;
        }

        void getTrajectory( OnlineTraj::OTGOutput& output ) {
            nonLinearFilterC3_( );
            output = output_;
        }

        void passToInput( OnlineTraj::OTGOutput& output );
    private:
        void computeNormalizedError_( );
        void updateMotionConstraints_( );
        bool nonLinearFilterC3_( );
        bool computeUc_( );
        double computeUv_( double& vel );
        bool computeUk_( );
        double computeUcv_( double& vel );
        double computeDeltaV_( double& vel );
        double computeUa_( double& acc );
        int getSignOfDelta_( const double& delta );
        void integrateControlVariable_( );
        double computeSigma_( );
        double computeMuPositive_( );
        double computeMuNegative_( );
        double computeSummation_( );
        int getSignOfSummation_( const double& summation );
        int getSign( double value );

        OnlineTraj::OTGParams params_;
        OnlineTraj::OTGOutput output_;

        // normalized error variables
        double error_pos_ = 0;
        double error_vel_ = 0;
        double error_acc_ = 0;

        // motion constraints
        double max_velocity_ = 0;
        double min_acceleration_ = 0;
        double min_velocity_ = 0;
        double max_acceleration_ = 0;

        double prev_acc_ = 0;
        double prev_vel_ = 0;
        double prev_pos_ = 0;

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



#endif /* OTG_H_ */
