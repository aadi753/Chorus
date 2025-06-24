#ifndef OTG_PARAMS_H_
#define OTG_PARAMS_H_

#include <vector>


#define DOF 1


namespace OnlineTraj {

    struct OTGParams {
        double sampling_rate = 0;
        double max_velocity = 0;
        double max_acceleration = 0;
        double max_jerk = 0;
        double min_velocity = 0;
        double min_acceleration = 0;
        double min_jerk = 0;
        double initial_position = 0;
        double target_position = 0;
        double target_velocity = 0;
        double target_acceleration = 0;

    };

    struct OTGOutput {
        double position = 0;
        double velocity = 0;
        double acceleration = 0;
        double jerk = 0;

    };

    struct OTGConstraints {
        double max_velocity = 0;
        double max_acceleration = 0;
        double max_jerk = 0;
        double min_velocity = 0;
        double min_acceleration = 0;
        double min_jerk = 0;
    };
    struct OTGTarget {
        std::vector<double> initial_position;
        std::vector<double> target_position;
    };




    typedef std::vector<OTGParams> MultiDofOTGParams;
    typedef std::vector<OTGOutput> MultiDofOTGOutput;
};
#endif /* OTG_PARAMS_H_ */
