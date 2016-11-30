# Introducing the Human-Baxter Collaboration Framework

This repository contains [BRML](https://brml.org/brml/)'s Human-Baxter collaboration framework. 
It was implemented as part of the [EIT CPS for Smart Factories project](http://dfki.de/smartfactories/), in collaboration with the [Neural Information Processing Group](http://nipg.inf.elte.hu/) at ELTE, Budapest, Hungary and [DFKI](http://dfki.de/web) Saarbruecken, Germany.


## Overview

The Human-Baxter collaboration framework aims at being a modular, easy to modify and adapt, framework for collaborative experiments with human collaborators and a [Baxter research robot](http://www.rethinkrobotics.com/research-education/).

The distributed pick-and-place scenario integrates the Baxter robot, the Microsoft Kinect V2 sensor and deep neural networks to detect, pick up and place objects.
Three types of experiments are possible:
- picking and placing an object on a table,
- handing over an object to a human collaborator and
- taking over an object from a human collaborator.


## Dependencies and Requirements

Two possibilities to use the Human-Baxter collaboration framework exist.
You either need the open source [Baxter simulator](http://sdk.rethinkrobotics.com/wiki/Baxter_Simulator) or access to a [Baxter research robot](http://www.rethinkrobotics.com/research-education/).

The Kinect V2 can be interfaced either on Ubuntu/ROS via the [iai_kinect2](https://github.com/code-iai/iai_kinect2) package (no skeleton tracking) or via a web socket connection and the ELTE Kinect Windows tool running on a Windows machine.

The framework heavily builds upon the [Baxter SDK](https://github.com/RethinkRobotics) and depends on customized versions of [baxter_interface](https://github.com/BRML/baxter_interface.git) and [baxter_common](https://github.com/BRML/baxter_common.git) from [Rethink Robotics](http://www.rethinkrobotics.com/). 
For the simulation in Gazebo, the [depth_sensors](https://github.com/BRML/depth_sensors.git) package is utilized.
For image processing we rely on the [OpenCV](http://opencv.org/) library.


The framework has been tested with ROS Indigo on Ubuntu 14.04.
For the simulator running in [Gazebo](http://gazebosim.org/) a machine with (any) NVIDIA GPU has been proven useful.
For the object detection using [faster R-CNN](https://github.com/rbgirshick/py-faster-rcnn) a GPU with at least 2GB RAM is required.
For the object segmentation using [MNC](https://github.com/daijifeng001/MNC) a GPU with at least 7 GB RAM are required (not used in the pick-and-place scenario distributed with the framework).
We made good experiences with NVIDIA Quadro K2200 and NVIDIA TITAN X GPUs.


## License

We publish the Human-Baxter collaboration framework under a BSD license, hoping that it might be useful for others as well.
The license text can be found in the LICENSE file and can be obtained from the [Open Source Initiative](https://opensource.org/licenses/BSD-2-Clause).

If you find our Human-Baxter collaboration framework useful in your work, please consider citing it:
```
@misc{hbcf2016,
    author={Ludersdorfer, Marvin},
    title={{The Human-Baxter collaboration framework}},
    organization={{Biomimetic Robotics and Machine Learning laboratory}},
    address={{fortiss GmbH}},
    year={2015--2016},
    howpublished={\url{https://github.com/BRML/baxter\_pick\_and\_place}},
    note={Accessed November 30, 2016}
}
```

## Installation

The framework is implemented as several [ROS](http://www.ros.org/) packages that can be installed conveniently as described [here](install.md).


## Usage

How to run the distributed pick-and-place scenario is explained in detail [here](scripts/README.md).
