# Individual Submission
Name             | Udacity Account Email
---------------- | ---------------------
Heming Chen      | hemingchenhc (at) gmail.com

This repository was the Final Project I submitted to [Udacity Self-Driving Car Engineer Nano Degree Program](https://eu.udacity.com/course/self-driving-car-engineer-nanodegree--nd013). 

The repository for training of traffic light detection model can be found [here](https://github.com/hemingchen/CarND-Capstone-Traffic-Light-Detection.git).

## I. Description

### 1. waypoint_updater
Based on traffic light information from `tl_detector` node, this node adjusts the expected ego vehicle speed at next N waypoints from ego vehicle's current position onwards, and publishes such information to `twist_controller` node.

It subscribes to the following topics:
```
/current_pose
/current_velocity
/base_waypoints
/traffic_waypoint
/obstacle_waypoint
```

and publishes to the following topics:
```
/final_waypoints
/ego_veh_waypoint # The nearest waypoint in front of ego vehicle's current position, useful to other nodes.
```

It follows below procedures:

1. Calculate the nearest waypoint in front of ego vehicle's current position, based on ego vehicle's current coordinates published to `/current_pose` topic.
2. Extract N next waypoints and maintain their default speeds. If the `base_waypoints` forms a loop, it will loop back to the beginning of the route and still fill up N waypoints.
3. Determine the appropriate speed at each waypoint. If red light is in the scope of next N waypoints, it slows down the vehicle at maximum deceleration of `-10 m/s^2` and eventually stops the vehicle near the stop line of traffic light. If no red light is in the scope, maintain/accelerate to the default speed at all waypoints. To do it, `waypoint_updater` first calculates waypoint nearest to the stop line of the traffic light, where the ego vehicle should completely stop. It then backward calculates the latest waypoint where the ego vehicle should start to decelerate at maximum comfort deceleration of `-10m/s^2` - when the ego vehicle travels at maximum speed. In the end, it assigns proper speed to each waypoint along the deceleration trajectory.
4. Publish the next N waypoints with their speeds properly adjusted.

where:

- `waypoint_updater` runs at `10Hz`.
- `N = LOOKAHEAD_WPS = 20` was used.
- Helper functions in `waypoint_updater/waypoint_updater_helper.py` were used.

### 2. twist_controller
Based on the latest information published to `final_waypoints`, this node adjust vehicle speed in the next N waypoints.

It subscribes to the following topics:
```
/current_velocity
/twist_cmd
/vehicle/dbw_enabled
```

and publishes to the following topics:
```
/vehicle/steering_cmd
/vehicle/throttle_cmd
/vehicle/brake_cmd
```

It follows below procedures:

1. Pass the proposed twist command and current twist information to the `twist_controller`, which will then compute the throttle, brake and steer.
2. Use a PID Controller for linear speed control and a Yaw Controller for steering control, which are provided by Udacity template.
3. If `dbw_enabled` is true, publish the computed throttle, brake and steer angle to their respective topics. Otherwise, do not publish anything since the ego vehicle is in manual mode.

where

- `twist_controller` can run as high as 25Hz without significant frame rate drop in the simulator. However Udacity states that 50Hz is the optimal rate.
- `Kp=0.7`, `Ki=0.0007` and `Kd=0.1` was used by the PID Controller for throttle control.

### 3. tl_detector
This node detects whether the ego vehicle is approaching a red traffic light using camera data.

It follows the following procedures:

1. Based on the current location of ego vehicle, find out the next traffic light and its stop line location.
2. Read the image from camera and determine the traffic light state.
3. If the traffic light state is `UNKNOWN` or `GREEN`, do not intervene.
4. If the traffic light state is `RED`, publish the index of the waypoint nearest to the stop line of such traffic light.

where:
- It uses a deep neural network trained with Tensorflow Object Detection API, based on a pretrained Mobile Net model. This work is done in [a separate repository](https://github.com/hemingchen/CarND-Capstone-Traffic-Light-Detection.git). The inference is done in `light_classification/tl_classifier.py`.
- `tl_detector` can run as fast as 3Hz on my PC before causing significant frame drops in simulator.
- Traffic light visibility threshold `TL_VISIBLE_DIST = 100` meters.
- `TL_DETECTOR_RATE = 3` where a new detection is not announced until 3 successive same-type detection is made.
- Helper functions in `tl_detector/tl_detector_helper.py` were used.

## II. How to use
For development and simulation tests, I used Windows 10 host (for simulator) and Udacity VM guest (for ROS).

### 1. Clone this repo
```
git clone https://github.com/hemingchen/CarND-Capstone
```

### 2. Install Python dependencies
```
# From repo root
pip install -r requirements.txt
```

### 3. Make and launch ROS
```
# From repo root
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```

### 4. Launch simulator

Launch the Term 3 Simultor. After adjusting graphics settings properly, choose the Highway scenario. Then, check `Camera` and uncheck `Manual` on the upper left corner of the simulator screen. Car should be running by itself and stop at red lights.



















## Udacity Original Readme Below

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
