# 🎼 Chorus

<img alt="Static Badge" src="https://img.shields.io/badge/build-passing-red?logo=github"> <img alt="Static Badge" src="https://img.shields.io/badge/branch-main-pink?logo=github"> <img alt="Static Badge" src="https://img.shields.io/badge/license-GPL--3.0--license-green?style=flat"> <img alt="Static Badge" src="https://img.shields.io/badge/trajectory--generation-blue?style=flat"> <img alt="Static Badge" src="https://img.shields.io/badge/online--planning-yellow?style=flat"> <img alt="Static Badge" src="https://img.shields.io/badge/robotics-purple?style=flat"> <img alt="Static Badge" src="https://img.shields.io/badge/C%2B%2B-white?style=flat">

**Chorus** is a high-performance **Multi-DOF RealTime Online Trajectory Generator** designed for robotic applications that demand real-time, smooth, and synchronized motion planning — all while respecting system constraints. With built-in jerk limiting and trajectory tracking, Chorus delivers optimal motion profiles with ultra-low latency.

---

## ✨ Features

- 🧠 **Real-Time Online Generation**  
  Generates smooth, optimal trajectories on the fly for any number of degrees of freedom.

- ⏱️ **Fast Execution**  
  Optimized for real-time performance — computation time is **under 1 milliseconds to compute the output in each control cycle**.

- 🔄 **Dynamic Target Updates**  
  Targets and constraints can be updated **during motion** without stopping the system.

- 📉 **Jerk-Limited Profiles**  
  Ensures continuous motion by limiting jerk, making trajectories smoother and safer for hardware.

- 🛠️ **Constraint-Aware Planning**  
  Respects user-defined limits on position, velocity, acceleration, and jerk.

- 🎯 **Synchronized Multi-DOF Motion**  
  All degrees of freedom are synchronized for coordinated and collision-free movement.

- 🚀 **Integrated Tracking Controller**  
  Includes a responsive trajectory tracking controller for **high tracking accuracy**.

---

## **Before diving in let's see what Chorus can do for you:**
### Sample profile for a 1 DOF system

 ![Chorus Architecture](images/single_dof_test.png)

- Blue is target position
- Orange is output position
- Green is output velocity
- Red is output acceleration

- Jerk output **(Chorus outputs a constant jerk)**

  ![Chorus Architecture](images/Figure_2.png)

- In the first image a visible noise is visible in acceleration but it is taken care of by Chorus therefore the velocity and position profiles are smooth.

## Okay! one DOF is not too fun , let's see what Chorus can do for you with 2 DOF

- **2 DOF system**
- Target position are updated, constraints are kept same for the whole duration
 ![Chorus Architecture](images/Figure_1_target_updated.png)

- Blue and red curves show the position outputs for each dof.
- Orange and purple curves show the velocity outputs for each dof.
- Green and brown show the acceleration outputs for each dof.
---
## NOTE: READ THE FULL README AND READ ALL THE COMMENTS IN THE EXAMPLES FOR THE BEST UNDERSTANDING OF HOW TO DO THINGS 
---
### Sample of how the targets were updated and what constraints were used.
- **Refer to the coded examples in bin folder to get proper understanding of how to do things this is just sample snippet.**
```.cpp
 // setting up the initial contraints
    constraints.sampling_rate = 0.001;
    constraints.max_velocity = 2.0; // rad/s
    constraints.min_velocity = -2.0;
    constraints.max_acceleration = 2.0; // rad/s^2
    constraints.min_acceleration = -2.0;
    constraints.max_jerk = 4.0; // rad/s^3
    constraints.min_jerk = -4.0;



// go to the target with the constraints sets above
        if ( t > 0 && t < 10 ) {
            target = { 5,2 };
        }
        // setting new target
        else if ( t > 10 && t < 15 ) {
            target = { 0,0 };

        }
        // setting new target
        else if ( t > 15 && t < 25 ) {
            target = { -7,-4 };


        }
        // setting new target
        else if ( t > 25 && t < 45 ) {
            target = { 1,1 };
        }
        // setting new target
        else {
            target = { 0,0 };
        }

```

### Okay ,so what we got?

1. Smooth motion ✅🚀
2. Constraints respected ✅🚀
3. Targets updates handled properly ✅🚀
4. Synchronized motion ✅🚀
---

### Great! Now let's see what we can do next

### How about changing the Constraints this time ??

### Why not, Chorus can handle that too 🚀

  ![Chorus Architecture](images/Figure_3_constraints_updated.png)

- Blue and red curves show the position outputs for each dof.
- Orange and purple curves show the velocity outputs for each dof.
- Green and brown show the acceleration outputs for each dof.

- **NOTE: When the constraints are changed in a way such that the velocity is coming from a higher value to lower value then make sure to put the right values in contraints otherwise it may cause a little bit of overshoot or oscillation and sometimes the sync can be a little off so make sure the Constraints values are set accordingly.**

**Its pure game of right Constraints values.**

### Let's see how we can do that
- **Refer to the coded examples in bin folder to get proper understanding of how to do things this is just sample snippet.**
```.cpp
// setting up the initial contraints
   constraints.sampling_rate = 0.001;
   constraints.max_velocity = 2.0; // rad/s
   constraints.min_velocity = -2.0;
   constraints.max_acceleration = 2.0; // rad/s^2
   constraints.min_acceleration = -2.0;
   constraints.max_jerk = 4.0; // rad/s^3
   constraints.min_jerk = -4.0;


 // go to the target with the constraints sets above
       if ( t > 0 && t < 6 ) {
           target = { 5,2 };

       }
       // setting new target and modifying the constraints
       else if ( t > 6 && t < 10 ) {
           target = { 0,0 };
           constraints.sampling_rate = 0.001;
           constraints.max_velocity = 1.0;
           constraints.min_velocity = -1.0;
           constraints.max_acceleration = 2.0;
           constraints.min_acceleration = -2.0;
           constraints.max_jerk = 4.0;
           constraints.min_jerk = -4.0;
       }

       // setting new target and modifying the constraints
       else if ( t > 10 && t < 25 ) {
           target = { -7,-4 };
           constraints.sampling_rate = 0.001;
           constraints.max_velocity = 1.5;
           constraints.min_velocity = -1.5;
           constraints.max_acceleration = 2.7;
           constraints.min_acceleration = -2.7;
           constraints.max_jerk = 3.0;
           constraints.min_jerk = -3.0;

       }

       // setting new target and modifying the constraints
       else if ( t > 25 && t < 45 ) {
           target = { 1,1 };
           constraints.sampling_rate = 0.001;
           constraints.max_velocity = 1.0;
           constraints.min_velocity = -1.0;
           constraints.max_acceleration = 1.0;
           constraints.min_acceleration = -1.0;
           constraints.max_jerk = 4.0;
           constraints.min_jerk = -4.0;
       }
       else {
           target = { 0,0 };
       }

```

### Okay ,so what we got?

1. Smooth motion ✅🚀
2. Constraints respected ✅🚀
3. Targets updates handled properly ✅🚀
4. Synchronized motion ✅🚀
---

### Woahhhh!🚀 What's next?

### Show us what more you got Chorus!

### What about changing the target and constraints together? 🤔

- **Haah! Too easy**
- **Here we goooo**
  ![Chorus Architecture](images/Figure_2_constraints_and_target_updated.png)
- Blue and red curves show the position outputs for each dof.
- Orange and purple curves show the velocity outputs for each dof.
- Green and brown show the acceleration outputs for each dof.

### Let's see how we can do that
- **Refer to the coded examples in bin folder to get proper understanding of how to do things this is just sample snippet.**
```.cpp
// setting up the initial contraints
   constraints.sampling_rate = 0.001;
   constraints.max_velocity = 2.0; // rad/s
   constraints.min_velocity = -2.0;
   constraints.max_acceleration = 2.0; // rad/s^2
   constraints.min_acceleration = -2.0;
   constraints.max_jerk = 4.0; // rad/s^3
   constraints.min_jerk = -4.0;


 // go to the target with the constraints sets above
        if ( t > 0 && t < 10 ) {
            target = { 5,2 };
        }
        // setting new target
        else if ( t > 10 && t < 15 ) {
            target = { 0,0 };

        }
        // setting new target and modifying the constraints
        else if ( t > 15 && t < 25 ) {
            target = { -7,-4 };
            constraints.max_velocity = 1.0;
            constraints.min_velocity = -1.0;
            constraints.max_acceleration = 0.9;
            constraints.min_acceleration = -0.9;
            constraints.max_jerk = 1.5;
            constraints.min_jerk = -1.5;

        }
        // setting new target
        else if ( t > 25 && t < 45 ) {
            target = { 1,1 };
        }
        // setting new target
        else {
            target = { 0,0 };
        }
```

### Okay ,so what we got?

1. Smooth motion ✅🚀
2. Constraints respected ✅🚀
3. Targets updates handled properly ✅🚀
4. Synchronized motion ✅🚀
---

### Impressive!🤩 What about 6 DOF's??

### Why not... just Do it and let's see what happens

- **And Chorus can do that too for you!** 🎶
  ![Chorus Architecture](images/Figure_4_6_dof_sync_position_plot.png)
  - Position plots for 6 DOF's

  ![Chorus Architecture](images/Figure_5_6_dof_sync_velocity_plot.png)

- Velocity plots for 6 DOF's

### Okay ,so what we got?

1. Smooth motion ✅🚀
2. Constraints respected ✅🚀
3. Targets updates handled properly ✅🚀
4. Synchronized motion ✅🚀

## Let's have a quick look at the controller output when the controller is used

  ![Chorus Architecture](images/fig1.png)

- Controller output for 2 dof
- The velocity plots are the controller outputs (purple and orange curves)
- The controller works on the position error between the output position and the position feedback given to Chorus 
- The controller in Chorus is working as a small corrector in every iteration.
- Controller output = (output.velocity + correction term based on position error).
- So basically it is feedforward velocity + some correction term

- **NOTE: the state feedback to the controller was provided by a noise generator, so the controller was given the feedback as output + noise ( noise was within specified bounds) in order to mimic the behavior on hardware.** 
---

## 🛠️ Getting Started

### ✅ Basic Setup Example (Crucial Initialization)

- Here we will see what are the requried objects that we need to make and what functions to call in order to use Chorus

```.cpp
// setting up Chorus — these are mandatory steps!
Chorus::MultiDofOtg otg; // wrapper class over single dof otg
Chorus::MultiDofOTGOutput output; // output structure for multi dof otg
Chorus::OTGConstraints constraints; //constraints for all the dof (shared among all dof's)
Chorus::OTGTargetPosition target; // target position for all dof's
Chorus::SystemStates states; // system states as feedback
Chorus::MultiDofOTGControllerGains gains; // controller gains
int dof = 2; // number of dof in system

// Set DOF and resize
otg.setDof(dof); // sets the number of dof in the MultiDofOtg object
target.resize(dof); // mandatory resize
gains.resize(dof); // mandatory resize

// Controller gains (0 disables controller)
for (size_t i = 0; i < dof; i++) {
    gains[i].kp = 0;
    gains[i].kd = 0;
    gains[i].ki = 0;
    gains[i].upper_limit = 0;
    gains[i].lower_limit = 0;
}
otg.setGains(gains); // sets the gains for the controller

// Define motion constraints
constraints.sampling_rate = 0.001; // 1 kHz
constraints.max_velocity = 2.0; // rad/s
constraints.min_velocity = -2.0;
constraints.max_acceleration = 2.0; // rad/s^2
constraints.min_acceleration = -2.0;
constraints.max_jerk = 4.0; // rad/s^3
constraints.min_jerk = -4.0;

// Set initial state and target
states.initial_position = {0, 0}; // initial state of the system based on feedback from the system
target = {0, 0}; // target position for system

// First output call
otg.getOutput(output);

// Update current state of the system based on the feedback from the system (replace the terms in the initial_position vector by the values from the feedback from the system).
states.initial_position = {output[0].position, output[1].position};

// Update system with constraints, target, and state
otg.update(constraints, target, states); // udpates the required data for Chorus , should be called in loop in every iteration

```

- ## So now we know what are the required initializations required in order to use Chorus ,refer to the coded examples in the bin folder to see how to use Chorus in your project.

---

## Deep Dive into the Chorus objects and structures

### - These are the all the data sturctures that you will need in order to setup your system.

- **Chorus::MultiDofOtg otg**

```.cpp
Chorus::MultiDofOtg otg;
```

- This is a wrapper class over the single dof OTG object this class is used to handle multiple DOF's ,it has the controller and the responsiblity to maintain Synchronization among the set DOF's in the system

- **Chorus::MultiDofOTGOutput output**

```.cpp
Chorus::MultiDofOTGOutput output;

// each index of output will have these fields
 struct OTGOutput {
        double position = 0;      ///< Current position
        double velocity = 0;      ///< Current velocity
        double acceleration = 0;  ///< Current acceleration
        double jerk = 0;          ///< Current jerk (control variable)
    };
```

- This is the vector of output structure that will have the output of position ,velocity ,acceleration ,jerk when the getTrajectory method is called with this sturcture pass to it.

- **Chorus::OTGConstraints constraints**

```.cpp
Chorus::OTGConstraints constraints;

struct OTGConstraints {
        double sampling_rate = 0;
        double max_velocity = 0;
        double max_acceleration = 0;
        double max_jerk = 0;
        double min_velocity = 0;
        double min_acceleration = 0;
        double min_jerk = 0;
    };
```

- This is the structure that hold the constraints that are shared among all the DOF's in the system.
- NOTE: the constraints should not be more than your systems max limits and these values are in rad/s for velocity and rad/s^2 for acceleration and rad/s^3 for jerk.

- **These constraints are not you system constrainst but are the trajectory constraints that you want for the motion so use them accordingly.**
- **Ofcourse the values should not be more than what your system can handle.**

- **Chorus::OTGTargetPosition target**

```.cpp
Chorus::OTGTargetPosition target;
```

- This is a vector that will hold the target positions

- **Chorus::SystemStates states**

```.cpp
Chorus::SystemStates states;

struct SystemStates {
        std::vector<double> initial_position;      ///< Initial positions for all DOFs
        std::vector<double> initial_velocity;      ///< Initial velocities for all DOFs
        std::vector<double> initial_acceleration;  ///< Initial accelerations for all DOFs
    };
```

- This is a vector of SystemStates structure and hold the current states of the system.
- For now use the initial_position term to be sent in feedback ,velocity and acceleration feedbacks are not being used by Chorus as of not ,they are here for just in case :).
- Each index of this vector will have the states of the corresponding DOF.

- **NOTE: State feedback should be sent to Chorus in every control cycle.**

- **Chorus::MultiDofOTGControllerGains**

```.cpp
Chorus::MultiDofOTGControllerGains gains;

// each index of gains will have this structure
 struct ControllerGains {
        double kp = 0;           ///< Proportional gain
        double kd = 0;           ///< Derivative gain
        double ki = 0;           ///< Integral gain
        double upper_limit = 0;  ///< Upper limit for control output
        double lower_limit = 0;  ///< Lower limit for control output
    };
```

- This vector of structure holds the gains for the controller.

---

## 🧩 Use Cases

- Industrial robot arms
- CNC machines
- Cobots in force-sensitive tasks
- Autonomous mobile robots
- Custom robotic platforms needing real-time planning

---

## Prerequisites

- C++14 or newer
- Eigen3
```bash
sudo apt-get install libeigen3-dev
```
- python3
- numpy
```bash
pip3 install numpy
```
- CMake ≥ 3.10

### Build Instructions

```bash
git clone https://github.com/aadi753/Chorus.git
cd Chorus
mkdir build && cd build
cmake ..
make -j4
```
# HAPPY TRAJECTORY PLANNING ✔😎
