/**
 * @file test_multi_dof_sync_verification.cpp
 * @author Aditya Singh
 * @brief Automated verification test suite for Multi-DOF Online Trajectory Generator time synchronization.
 * @version 0.1
 * @date 2026-09-24
 *
 * Chorus - Multi-DOF Online Trajectory Generator
 * Copyright (C) 2025-2026 Aditya Singh
 */

#include "chorus.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <cassert>
#include <string>

struct TestCaseConfig {
    std::string name;
    double max_velocity;
    double max_acceleration;
    double max_jerk;
    std::vector<double> targets;
    std::vector<double> initial_positions;
    double max_allowed_spread_sec;
};

bool run_sync_test(const TestCaseConfig& test_config) {
    int dof = test_config.targets.size();
    Chorus::MultiDofOtg otg;
    otg.setDof(dof);

    Chorus::MultiDofOTGControllerGains gains(dof);
    for (int i = 0; i < dof; i++) {
        gains[i].kp = 0;
        gains[i].kd = 0;
        gains[i].ki = 0;
        gains[i].upper_limit = 0;
        gains[i].lower_limit = 0;
    }
    otg.setGains(gains);

    Chorus::OTGConstraints constraints;
    constraints.sampling_rate = 0.001; // 1 ms
    constraints.max_velocity = test_config.max_velocity;
    constraints.min_velocity = -test_config.max_velocity;
    constraints.max_acceleration = test_config.max_acceleration;
    constraints.min_acceleration = -test_config.max_acceleration;
    constraints.max_jerk = test_config.max_jerk;
    constraints.min_jerk = -test_config.max_jerk;

    Chorus::SystemStates states;
    states.initial_position = test_config.initial_positions;
    states.initial_velocity = std::vector<double>(dof, 0.0);
    states.initial_acceleration = std::vector<double>(dof, 0.0);

    Chorus::OTGTargetPosition target = test_config.targets;
    otg.update(constraints, target, states);

    Chorus::MultiDofOTGOutput output;
    double time_elapsed = 0;
    const double dt = constraints.sampling_rate;
    std::vector<double> reach_time(dof, -1.0);
    std::vector<bool> reached(dof, false);

    bool constraint_violated = false;
    double max_observed_vel = 0.0;
    double max_observed_acc = 0.0;
    const double max_sim_time = 100.0;

    double max_displacement = 0.0;
    for (int i = 0; i < dof; i++) {
        max_displacement = std::max(max_displacement, std::abs(test_config.targets[i] - test_config.initial_positions[i]));
    }

    while (time_elapsed < max_sim_time) {
        otg.getOutput(output);

        for (int i = 0; i < dof; i++) {
            const double displacement = std::abs(test_config.targets[i] - test_config.initial_positions[i]);
            const double pos_error = std::abs(output[i].position - target[i]);
            const double vel = std::abs(output[i].velocity);
            const double acc = std::abs(output[i].acceleration);

            max_observed_vel = std::max(max_observed_vel, vel);
            max_observed_acc = std::max(max_observed_acc, acc);

            // Constraint violation check (allowing 5% margin for discrete numerical integration)
            if (vel > test_config.max_velocity * 1.05 || acc > test_config.max_acceleration * 1.05) {
                constraint_violated = true;
            }

            if (!reached[i]) {
                if (displacement < 1e-9) {
                    reach_time[i] = 0.0;
                    reached[i] = true;
                } else {
                    const double scale = displacement / max_displacement;
                    const double joint_max_vel = scale * test_config.max_velocity;
                    const double pos_tolerance = std::max(1e-5, 0.002 * displacement);
                    const double vel_tolerance = std::max(1e-5, 0.02 * joint_max_vel);

                    if (pos_error <= pos_tolerance && vel <= vel_tolerance && time_elapsed > 0.05) {
                        reach_time[i] = time_elapsed;
                        reached[i] = true;
                    }
                }
            }
        }

        states.initial_position.resize(dof);
        for (int i = 0; i < dof; i++) {
            states.initial_position[i] = output[i].position;
        }
        otg.update(constraints, target, states);

        bool all_done = true;
        for (int i = 0; i < dof; i++) {
            if (!reached[i]) all_done = false;
        }
        if (all_done && time_elapsed > 0.1) break;

        time_elapsed += dt;
    }

    double min_reach_time = 1e9;
    double max_reach_time = -1.0;
    bool all_moving_reached = true;

    std::cout << "=================================================================" << std::endl;
    std::cout << " TEST CASE: " << test_config.name << std::endl;
    std::cout << " Constraints: Vmax=" << test_config.max_velocity 
              << ", Amax=" << test_config.max_acceleration 
              << ", Jmax=" << test_config.max_jerk << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    for (int i = 0; i < dof; i++) {
        const double displacement = std::abs(test_config.targets[i] - test_config.initial_positions[i]);
        std::cout << "  DOF " << i << " | Displacement: " << std::setw(8) << std::fixed << std::setprecision(4) << displacement
                  << " | Final Pos: " << std::setw(8) << output[i].position
                  << " | Reach Time: ";
        if (reached[i]) {
            std::cout << std::setw(7) << reach_time[i] << " s";
            if (displacement > 1e-9) {
                min_reach_time = std::min(min_reach_time, reach_time[i]);
                max_reach_time = std::max(max_reach_time, reach_time[i]);
            }
        } else {
            std::cout << "NOT REACHED (TIMEOUT)";
            all_moving_reached = false;
        }
        std::cout << std::endl;
    }

    const double time_spread = (max_reach_time >= 0 && min_reach_time < 1e8) ? (max_reach_time - min_reach_time) : 999.0;
    const bool spread_passed = time_spread <= test_config.max_allowed_spread_sec;
    const bool test_passed = all_moving_reached && !constraint_violated && spread_passed;

    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "  Time Spread: " << std::fixed << std::setprecision(4) << time_spread << " s (Max Allowed: " << test_config.max_allowed_spread_sec << " s)" << std::endl;
    std::cout << "  Max Observed Vel: " << max_observed_vel << " | Max Observed Acc: " << max_observed_acc << std::endl;
    std::cout << "  Constraint Compliance: " << (constraint_violated ? "VIOLATED [FAIL]" : "SATISFIED [PASS]") << std::endl;
    std::cout << "  Test Result: " << (test_passed ? "PASSED [OK]" : "FAILED [ERROR]") << std::endl;
    std::cout << "=================================================================\n" << std::endl;

    return test_passed;
}

// Test runtime continuous updates
bool test_runtime_continuous_updates() {
    std::cout << "=================================================================" << std::endl;
    std::cout << " TEST CASE: Dynamic Multi-Waypoint Target Updates" << std::endl;
    std::cout << "=================================================================" << std::endl;

    int dof = 4;
    Chorus::MultiDofOtg otg;
    otg.setDof(dof);

    Chorus::MultiDofOTGControllerGains gains(dof);
    otg.setGains(gains);

    Chorus::OTGConstraints constraints;
    constraints.sampling_rate = 0.001;
    constraints.max_velocity = 2.0;
    constraints.min_velocity = -2.0;
    constraints.max_acceleration = 3.0;
    constraints.min_acceleration = -3.0;
    constraints.max_jerk = 5.0;
    constraints.min_jerk = -5.0;

    Chorus::SystemStates states;
    states.initial_position = {0, 0, 0, 0};
    states.initial_velocity = {0, 0, 0, 0};
    states.initial_acceleration = {0, 0, 0, 0};

    Chorus::OTGTargetPosition target = {0, 0, 0, 0};
    otg.update(constraints, target, states);

    Chorus::MultiDofOTGOutput output;
    double t = 0;
    bool passed = true;

    while (t < 28.0) {
        if (t >= 0.0 && t < 6.0) {
            target = {5.0, 1.0, -3.0, 0.05};
        } else if (t >= 6.0 && t < 12.0) {
            target = {0.0, 0.0, 0.0, 0.0};
        } else if (t >= 12.0 && t < 20.0) {
            target = {-8.0, 4.0, -0.01, 6.0};
        } else {
            target = {2.0, 2.0, 2.0, 2.0};
        }

        otg.getOutput(output);

        for (int i = 0; i < dof; i++) {
            if (std::abs(output[i].velocity) > constraints.max_velocity * 1.05 ||
                std::abs(output[i].acceleration) > constraints.max_acceleration * 1.05) {
                passed = false;
            }
        }

        for (int i = 0; i < dof; i++) {
            states.initial_position[i] = output[i].position;
        }
        otg.update(constraints, target, states);

        t += constraints.sampling_rate;
    }

    // Check final target reached at t = 28s
    for (int i = 0; i < dof; i++) {
        if (std::abs(output[i].position - target[i]) > 0.01 || std::abs(output[i].velocity) > 0.01) {
            passed = false;
        }
    }

    std::cout << "  Dynamic Waypoints Result: " << (passed ? "PASSED [OK]" : "FAILED [ERROR]") << "\n" << std::endl;
    return passed;
}

int main() {
    std::cout << "\n#################################################################" << std::endl;
    std::cout << "       CHORUS MULTI-DOF TIME SYNCHRONIZATION TEST SUITE          " << std::endl;
    std::cout << "#################################################################\n" << std::endl;

    std::vector<TestCaseConfig> test_cases = {
        {
            "1. Standard 6-DOF Multi-Joint Motion",
            2.0, 2.0, 4.0,
            {5.0, 2.0, -5.0, -3.0, 1.0, 4.0},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            0.15
        },
        {
            "2. Extreme Displacement Difference (50.0 vs 0.001 rad)",
            3.0, 2.0, 5.0,
            {50.0, 0.001, 0.01, 0.1, 1.0, 10.0},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            0.35
        },
        {
            "3. Extreme Constraints: Low Acceleration (0.2), High Velocity (10.0), Jerk (0.5)",
            10.0, 0.2, 0.5,
            {15.0, 0.05, 0.5, 2.0, 8.0, 0.1},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            0.20
        },
        {
            "4. Extreme Constraints: High Acceleration (20.0), Low Velocity (0.5), High Jerk (100.0)",
            0.5, 20.0, 100.0,
            {5.0, 0.01, 0.1, 0.5, 2.0, 0.05},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            0.20
        },
        {
            "5. Short Stroke Motion (No Constant Velocity Cruising Phase)",
            2.0, 2.0, 4.0,
            {0.1, 0.01, 0.005, 0.08, 0.02, 0.05},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            0.25
        },
        {
            "6. Sub-Millimeter Micro-Movements",
            2.0, 2.0, 4.0,
            {0.005, 0.0001, 0.001, 0.002, 0.004, 0.0005},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            0.25
        },
        {
            "7. Mixed Signs & Bidirectional Displacements",
            2.0, 3.0, 5.0,
            {10.0, -0.02, 3.0, -7.0, 0.1, -0.005},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            0.20
        },
        {
            "8. Stationary Joint Inclusion (Zero-Displacement DOFs)",
            2.0, 2.0, 4.0,
            {10.0, 0.0, 5.0, -2.0, 0.0, 1.0},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            0.20
        }
    };

    int total_tests = 0;
    int passed_tests = 0;

    for (const auto& tc : test_cases) {
        total_tests++;
        if (run_sync_test(tc)) {
            passed_tests++;
        }
    }

    total_tests++;
    if (test_runtime_continuous_updates()) {
        passed_tests++;
    }

    std::cout << "\n=================================================================" << std::endl;
    std::cout << " FINAL SUMMARY: " << passed_tests << " / " << total_tests << " TEST SUITES PASSED" << std::endl;
    std::cout << "=================================================================\n" << std::endl;

    if (passed_tests == total_tests) {
        std::cout << ">>> ALL TIME SYNCHRONIZATION TESTS PASSED SUCCESSFULLY! <<<\n" << std::endl;
        return 0;
    } else {
        std::cerr << ">>> SOME TESTS FAILED! <<<\n" << std::endl;
        return 1;
    }
}
