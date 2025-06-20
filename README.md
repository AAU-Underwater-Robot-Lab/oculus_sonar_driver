# oculus_sonar_driver

[![Build Status](https://gitlab.drone.camhd.science/api/badges/apl-ocean-engineering/oculus_sonar_driver/status.svg)](https://gitlab.drone.camhd.science/apl-ocean-engineering/oculus_sonar_driver)

ROS node/nodelet that uses [liboculus](https://github.com/apl-ocean-engineering/liboculus) to interface with a [Blueprint Subsea Oculus imaging sonar](https://www.blueprintsubsea.com/oculus/index.php).
Currently tested on ROS1 Noetic.

At present we have only tested with the [Oculus M1200d](https://www.blueprintsubsea.com/pages/product.php?PN=BP01042) sonar.

## Contents

This package defines the `oculus_sonar/driver` nodelet that interfaces with the sonar
over ethernet and publishes [`acoustic_msgs::ProjectedSonarImage`](https://github.com/apl-ocean-engineering/hydrographic_msgs/blob/main/acoustic_msgs/msg/ProjectedSonarImage.msg) and
[`apl_msgs::RawData`](https://gitlab.com/apl-ocean-engineering/apl_msgs) messages.

The package also builds a conventional node `oculus_driver` which is a trivial wrapper around a
`oculus_sonar/driver` nodelet.

** Note that to visualize the data (as a ROS Image) you need to use the [rqt_sonar_image_view]([sonar_image_proc](https://github.com/apl-ocean-engineering/rqt_sonar_image_view) rqt plugin or launch a [sonar_image_proc/draw_sonar](https://github.com/apl-ocean-engineering/sonar_image_proc) node/nodelet.  The `default_ros.launch` launchfile will launch this node if `draw_sonar:=true` is passed to it.

## Installation

  1. Either:
     1. Use [vcstool](http://wiki.ros.org/vcstool):
        1. Clone this repo to `<catkin_ws>/src`
        1. `cd <catkin_src>`
        1. `vcs import --input oculus_sonar_driver/oculus_sonar_driver.repos`
     1. Or install dependencies manually. Clone [liboculus](https://github.com/apl-ocean-engineering/liboculus), [hydrographic_msgs](https://github.com/apl-ocean-engineering/hydrographic_msgs.git) and [g3log_ros](https://gitlab.com/apl-ocean-engineering/g3log_ros) to `<catkin_ws>/src`
  1. Run `catkin_make` or `catkin build` from ``<catkin_ws>``
  1. Run `source ./devel/setup.bash` from ``<catkin_ws>``

## Usage
`roslaunch oculus_sonar_driver default_ros.launch` will start the sonar driver.
See the comments in [`default_ros.launch`](launch/default_ros.launch) to set sonar parameters at startup.   Sonar params can also be set through dynamic reconfigure (below).

## dynamic_reconfigure

The sonar params can be modified on the fly using dynamic reconfigure.  The launchfile [`default_ros.launch`](launch/default_ros.launch) starts the nodes in a nodelet manager `/nodelet_manager`.  The dynamic reconfigure params can be queried as:

```
$ rosrun dynamic_reconfigure dynparam get /nodelet_manager
{'send_range_as_meters': True, 'send_gain': True, 'send_simple_return': True, 'gain_assistance': False, 'all_beams': True, 'num_beams': 1, 'gamma': 127, 'ping_rate': 0, 'data_size': 0, 'freq_mode': 2, 'range': 2.0, 'gain': 50.0, 'groups': {'id': 0, 'parent': 0, 'name': 'Default', 'type': '', 'state': True, 'groups': {}, 'parameters': {}, 'send_range_as_meters': True, 'send_gain': True, 'send_simple_return': True, 'gain_assistance': False, 'num_beams': 1, 'all_beams': True, 'range': 2.0, 'gain': 50.0, 'gamma': 127, 'ping_rate': 0, 'data_size': 0, 'freq_mode': 2}}
```

These params can also be set from the command line.  For example, to enable 32-bit mode:

```
$ rosrun dynamic_reconfigure dynparam set /nodelet_manager data_size 2
```

----
# A note about sonar resolution

(This is inferred from experimental data, it's not from Blueprint)

The sonar appears to have a fixed maximum number of range bins, approx 720.  It also has fixed quantized range resolutions:  ~2.8mm, ~5.6mm, etc.  So the number of range bins N present in data at a given range R in meters is:

```
N = R / (2^k * 2.8mm)  minimizing k s.t. N <= 720
```

This means, for example:

| Range (m) | Bins | Resolution |
|-----------|------|------------|
| 2.0 | 709 | 2.8mm |
| 2.02 | 720 | 2.8mm |
| 3 | 532 | 5.6mm |
| 4 | 709 | 5.6mm |
| 4.065 | 720 | 5.6mm |
| 5 | 443 | 11.2mm |
| 6 | 532 | 11.2mm |

etc.   This effect seems to be invariant of 256 v 512 beams, and 8/16/32 bit data.  Haven't tested in the lower frequency mode


----
# Related Packages

* [acoustic_msgs](https://github.com/apl-ocean-engineering/hydrographic_msgs/tree/main/acoustic_msgs) defines the ROS [ProjectedSonarImage](https://github.com/apl-ocean-engineering/hydrographic_msgs/blob/main/acoustic_msgs/msg/ProjectedSonarImage.msg) message type published by this node.
* [liboculus](https://github.com/apl-ocean-engineering/liboculus) is the underlying (non-ROS) library which parses handles the Oculus network protocol.  It also includes a Boost::Asio-based network client.
* [sonar_image_proc](https://github.com/apl-ocean-engineering/sonar_image_proc) contains code to postprocess sonar data, including drawing the sonar data to an OpenCV Mat (contains both ROS and non-ROS code).
* [rqt_sonar_image_view](https://github.com/apl-ocean-engineering/rqt_sonar_image_view) is an Rqt plugin for displaying sonar imagery (uses [sonar_image_proc](https://github.com/apl-ocean-engineering/sonar_image_proc))

# License

This repository is covered by the [BSD 3-Clause License](LICENSE).
