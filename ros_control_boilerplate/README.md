# ROS Control Boilerplate

Simple simulation interface and template for setting up a hardware interface for ros_control. The idea is you take this as a starting point for creating your hardware interfaces, and it is needed because [ros_control documentation](http://wiki.ros.org/ros_control) is sparse. This boilerplate demonstrates:

 - Creating a hardware_interface for multiple joints for use with ros_control
 - Position Trajectory Controller
 - Control of 2 joints of the simple robot "Inchworm" pictured below
 - Loading configurations with roslaunch and yaml files
 - Generating a random trajectory and sending it over an actionlib interface
 - Partial support of joint mode switching (needs to be improved)
 - Joint limits
 - Pass-through non-physics based robot simulator
 - Visualization in Rviz

<img src="https://picknik.ai/assets/images/logo.jpg" width="120">

This open source project was developed at [PickNik Robotics](https://picknik.ai/). Need professional ROS development and consulting? Contact us at projects@picknik.ai for a free consultation.

## Maintainers

Special thanks to the following maintainers of this repo:

 - Dave Coleman (@davetcoleman)
 - Andy Zelenak (@AndyZe)
 - John Morris (@zultron)
 - Robert Wilbrandt (@RobertWilbrandt)

## Status:

 * [![Build Status](https://travis-ci.org/PickNikRobotics/ros_control_boilerplate.svg?branch=melodic-devel)](https://travis-ci.org/PickNikRobotics/ros_control_boilerplate) Travis CI
 * [![Devel Job Status](http://build.ros.org/buildStatus/icon?job=Mdev__ros_control_boilerplate__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__ros_control_boilerplate__ubuntu_bionic_amd64/) Devel Job Status
 * [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__ros_control_boilerplate__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__ros_control_boilerplate__ubuntu_bionic_amd64__binary/) AMD64 Debian Job Status

<img src="https://raw.githubusercontent.com/davetcoleman/ros_control_boilerplate/jade-devel/resources/screenshot.png"/>


## Video Demo

See [YouTube](https://www.youtube.com/watch?v=Tpj2tx9uZ-o) for a very modest video demo.

## Install

This package depends on [gazebo_ros_demos](https://github.com/ros-simulation/gazebo_ros_demos) for its ``inchworm_description`` package, but you must add it to your catkin workspace by source:

    git clone https://github.com/ros-simulation/gazebo_ros_demos.git

Then, either install this package from source so you can develop off of it, or install from debian:

    sudo apt-get install ros-indigo-ros-control-boilerplate

## Run Simulation Demo

This package is setup to run the "Inchworm" two joint revolute-revolute robot demo. This "template package" is located in the ros_control_boilerplate as a subfolder that you can easily rename and reuse. To run its ros_control non-physics-based simulated hardware interface, run:

    roslaunch ros_control_boilerplate inchworm_simulation.launch

To visualize its published ``/tf`` coordinate transforms in Rviz run:

    roslaunch ros_control_boilerplate inchworm_visualize.launch

To send a random, dummy trajectory to execute, run:

    roslaunch ros_control_boilerplate inchworm_test_trajectory.launch

## Customize

To test this as a simulation interface for your robot, you can quickly rename the subfolder package into the name of your robot using the following commands:

```
function findreplace() {
    grep -lr -e "$1" * | xargs sed -i "s/$1/$2/g" ;
}

function findreplacefilename() {
    find . -depth -name "*$1*" -exec bash -c 'for f; do base=${f##*/}; mv -- "$f" "${f%/*}/${base//'$1'/'$2'}"; done' _ {} +
}

findreplacefilename inchworm myrobot
findreplace inchworm myrobot
findreplace Inchworm MyRobot
findreplace INCHWORM MYROBOT
```

Then add the necessary code to communicate with your robot via USB/serial/ethernet/etc in the file ``myrobot_hw_interface.cpp``.

## Setting an Initial Position, Using with MoveIt!

If you need your robot to startup at a particular position in simulation, or you would like to use this funcitonality to simulate your robot with MoveIt!, see the downstream package (it depends on this package) [moveit_sim_controller](https://github.com/davetcoleman/moveit_sim_controller)

## Other Helper Tools

### Recording to CSV

Write the commands from a trajectory controller to csv file

    rosrun ros_control_boilerplate controller_to_csv SAVE_TO_FILE_PATH CONTROLLER_STATE_TOPIC TIME_TO_RECORD

### Commanding from CSV

Read from csv file and execute on robot

    rosrun ros_control_boilerplate csv_to_controller READ_FROM_FILE_PATH CONTROLLER_STATE_TOPIC TIME_TO_RECORD

### Commanding from Keyboard

Joint-level teleop from a keyboard (TODO: remove had coded topic names)

    rosrun ros_control_boilerplate keyboard_teleop

## Limitations

 - Does not implement estops, transmissions, or other fancy new features of ros_control
 - Does not have any hard realtime code, this depends largely on your platform, kernel, OS, etc
 - Only position control is fully implemented, though some code is in place for velocity control

## Contribute

Please add features, make corrections, and address the limitations above, thanks!
