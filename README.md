# Online-Trajectory-Planning-Manipulators

## Project Description

This submission consists of online trajectory planning algorithms for 6-axis manipulators using **double S velocity profile**.

[**Under Improvement and Translation, since the project is done in China**]

## Files Description
    .
    ├── Traj_Planning
    |   ├── OnlinePlanning.m        # Online Double S planning function
    |   ├── robotInit.m             # Initialize robot model
    |   ├── OnlinePlanning_test.m   # Example for OnlinePlanning on manipulators
    |   ├── LineTracking.m          # Example for line tracking     
    |   ├── CircleTracking.m        # Example for circle tracking
    |
    ├── .gitignore
    └── README.md
- `OnlinePlanning.m` - Function of online trajecotry planning based on Double S Velocity Profile to reach the goal with continuous acceleration in discrete time
- `robotInit.m` - Function of Initialize the 6-axis robot arm model using modified DH parameters on Robotics Toolbox
- `OnlinePlanning_test.m` - Example of using online planning function to calculate the state planning for next interval within joint configuration
- `LineTracking.m` - Example of using PID following to reach servo points (Line Trajcotry) with joint constraints (accleration is discontinuous)
- `CircleTracking.m` - Example of using PID following to reach servo points (Circle Trajcotry) with joint constraints (accleration is discontinuous)

## Features

### Limitations
- Planning Algorithms only consider position inputs and ignore orientation inputs