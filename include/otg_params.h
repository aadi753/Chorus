#ifndef OTG_PARAMS_H_
#define OTG_PARAMS_H_

#include <vector>


#define DOF 1


namespace OnlineTraj {

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
        double sampling_rate = 0;
        double max_velocity = 0;
        double max_acceleration = 0;
        double max_jerk = 0;
        double min_velocity = 0;
        double min_acceleration = 0;
        double min_jerk = 0;
    };

    /**
     * @brief System state vectors for all DOFs.
     */
    struct SystemStates {
        std::vector<double> initial_position;      ///< Initial positions for all DOFs
        std::vector<double> initial_velocity;      ///< Initial velocities for all DOFs
        std::vector<double> initial_acceleration;  ///< Initial accelerations for all DOFs
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



    typedef std::vector<OTGParams> MultiDofOTGParams; ///< OTG parameters for all DOFs
    typedef std::vector<OTGOutput> MultiDofOTGOutput; ///< OTG outputs for all DOFs
    typedef std::vector<ControllerGains> MultiDofOTGControllerGains; ///< Controller gains for all DOFs
};
#endif /* OTG_PARAMS_H_ */
