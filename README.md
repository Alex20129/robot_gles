# Robot Qt + OpenGL

An interactive 6-DOF robot simulator based on Qt, featuring real-time OpenGL visualization.
The application provides manual joint control and animated movement to a target pose.

![alt tag](https://github.com/Alex20129/robot_gles/blob/main/robot_gles.png)

## Features

- 6-DOF serial manipulator model with configurable joint limits and link lengths
- Real-time 3D visualization using Qt OpenGL widgets
- Binary STL model loading for individual links and target marker
- Interactive camera (orbit + zoom)
- Manual joint angle control
- Inverse kinematics solver
- Animated movement along a given path

## Inverse Kinematics

For solving the inverse kinematics problem, a simple iterative algorithm is used.
In this project, the algorithm is referred to as 'ACD' - Adaptive Coordinate Descent.

Joint angles are optimized sequentially. The joint angle is adjusted by the correction value. If the error decreases, the new angle is kept for this joint. Otherwise the correction value is multiplied by a negative coefficient, which simultaneously reverses direction and reduces correction magnitude. The target pose is reached in two phases: Position (joints 0, 1, 2) and Orientation (joints 3, 4, 5).

As a local numerical solver, ACD is sensitive to initial conditions and converges only if two conditions are met:
1. A valid kinematic solution exists, i.e. the target is reachable.
2. A continuous trajectory from the current pose to the target pose exists within the joint space, i.e. the optimization path is not blocked by the joint limit and/or singular configuration.

It works for robots of a specific configuration: 5/6-axis robot with a spherical wrist. No study was conducted for other configurations.

## How to build

```bash
git clone git@github.com:Alex20129/robot_gles.git
mkdir robot_gles-build
cd robot_gles-build
cmake ../robot_gles
cmake --build .
```
