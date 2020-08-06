# odom_to_tf

## Overview

This package converts an odometry message published by gazebo to a TF message.

**Keywords:** gazebo, odometry, tf

### License

The source code is released under a [GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.en.html).

**Author: Tim Wiese<br />
Affiliation: [European Space Agency](https://www.esa.int/)<br />
Maintainer: Tim Wiese, tim.wiese@esa.int**

This package has been tested under [ROS2] Foxy Fitzroy and Ubuntu 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)

#### Building

To build from source, clone the latest version from this repository into your ros2 workspace and compile the package using

	cd ros2_ws/src
	git clone https://github.com/esa-prl/marta_launch
	cd ../
	colcon build

## Usage

Run the main node with:

	ros2 run odom_to_tf odom_to_tf_node --ros-args --params-file src/odom_to_tf/config/odom_to_tf.yaml

## Nodes

### odom_to_tf_node

This node contains all the logic.

#### Parameters

* **`gazebo_entity`** (string, default: "marta") 

    The gazebo entity for which the odometry message should be converted
    
* **`body_tf_name`** (string, default: "odom")

    The child frame of the TF message

#### Subscribed Topics

* **`/[gazebo_entity]/odom`**  ([nav_msg/Odometry])

    The odometry messages to convert

#### Published Topics

* **`/tf`** ([tf2_msgs/TFMessage])

    The TF messages converted from the odometry messages

## Bugs & Feature Requests

Please report bugs and request features using the github issue tracker.

[ROS2]: http://www.ros.org
[tf2_msgs/TFMessage]: http://docs.ros.org/noetic/api/tf2_msgs/html/msg/TFMessage.html
[nav_msg/Odometry]: http://docs.ros.org/noetic/api/nav_msgs/html/msg/Odometry.html
