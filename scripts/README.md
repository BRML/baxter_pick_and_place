# Usage of the Human-Baxter Collaboration Framework

Let us introduce the usage of the Human-Baxter collaboration framework using the example of the distributed pick-and-place scenario.
For first-time usage, we need to
- download the pre-trained neural networks,
- perform the calibration routines and
- run the experiment.

Otherwise it is sufficient to 
- run the experiment.

Note that it might make sense to repeat some parts of the calibration every once in a while.


In the following, we assume that the Human-Baxter collaboration framework has been installed as described [here](../install.md) into a catkin workspace called `$WS_HBCF`.  


## Preliminaries


### Obtain pre-trained Caffe Model

To use R-FCN for object detection we first download the pre-trained weights for the Caffe model:
```bash
$ cd $WS_HBCF/data/ResNet-101
$ . fetch_rfcn.sh
```


### Calibrate Kinect V2 Sensor

It is advised that you calibrate your Kinect V2 once.
To do so, connect your sensor and run
```bash
$ cd $WS_HBCF/src/iai_kinect2
$ source /opt/ros/indigo/setup.bash
$ roslaunch kinect2_bridge kinect2_bridge.launch
```

Calibrate your sensor using the `kinect2_calibration`.
Follow the instructions [here](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration).
We obtained good results with the *chess5x7x0.03* pattern and the guidelines laid out in Wiedemeyer's second comment [here](https://github.com/code-iai/iai_kinect2/issues/311).

Add the calibration files to the `kinect2_bridge/data/<serialnumber>` folder.
See [here](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_bridge#first-steps) for further details.

Restart `kinect2_bridge` such that it uses the new calibration.


### Assemble the Hand Mount for the External Calibration
For the external calibration of the Kinect V2 sensor relative to the Baxter robot a hand-mounted calibration pattern is used.
The hand mount has been designed in [OpenSCAD](http://www.openscad.org/) to replace the fingers of the electrical gripper.
The original CAD file and the derived STL file can be found in `$WS_HBCF/models/mount`.

3D print the hand mount.
Additionally, you will need a flat piece of wood or acrylic glass of about 21 x 25 cm, a drill and some screws.

Print the [calibration pattern](http://docs.opencv.org/2.4.13/_downloads/acircles_pattern.png) and make sure that its dimensions are preserved by the printer.
When gluing the patten onto the mount (after it has been screwed to the hand of the robot), make sure that it is completely flat! 

![Image of back side of hand mount](../models/mount/mount-back.jpg | width=100)
![Image of front side of hand mount](../models/mount/mount-front.jpg | width=100)
![Image of mounted hand mount](../models/mount/mounted.jpg | width=100)
![Image of hand mount with calibration pattern](../models/mount/mount-pattern.jpg | width=100)



## Calibration

Copy 


## Run the Experiment

Using the Kinect sensor requires some preparation:
- If you are intending to use the Kinect V2 on Windows, make sure to plug it in and run the ELTE Kinect Windows tool on the Windows machine.
- If you want to use the Kinect V2 on Ubuntu using ROS, plug it in and run the kinect2_bridge by typing
```bash
$ cd $WS_HBCF
$ . baxter.sh
$ rosrun kinect2_bridge kinect2_bridge
```
- If you are using the simulated Kinect, the launch script (see below) will take care of the details.


To follow the progress of the experiment visual information is published to a ROS image topic.
To show the visualization, type (in two terminals with initialized SDK environment) `rosrun baxter_pick_and_place inspection.py` and `rosrun image_view image_view image:=/visualization/image`.


### On the Robot

To run the pick-and-place experiment, initialize your SDK environment and `rosrun` the experiment.
That is, do
```bash
$ cd $WS_HBCF
$ . baxter.sh
$ rosrun baxter_pick_and_place demonstration.py
```
In a new terminal, run the instruction module
```bash
$ cd $WS_HBCF
$ . baxter.sh
$ rosrun baxter_pick_and_place instruct.py
```
and follow the instructions in the terminal.


### In Simulation

To start up the simulation environment (Gazebo) and run the demonstration, 
initialize your SDK environment in simulation mode, `roslaunch` the simulator
and `rosrun` the experiment.
That is, in a terminal do
```bash
$ cd $WS_HBCF
$ . baxter.sh sim
$ roslaunch baxter_pick_and_place simulation.launch depth_external:=true
```
In another terminal do
```bash
$ cd $WS_HBCF
$ . baxter.sh sim
$ rosrun baxter_pick_and_place demonstration.py
```
And in a third terminal run the instruction module by typing
```bash
$ cd $WS_HBCF
$ . baxter.sh sim
$ rosrun baxter_pick_and_place instruct.py
```
Now follow the instructions in the third terminal.

Note: The `roslaunch` files have parameters to modify their behavior. Please
have a look at the files for more information.


### Experiment Convenience Launch File

There also is a launch file that collects the separate steps above into
one file.
To [start up the simulation environment and] run the experiment, do
```bash
$ cd $WS_HBCF
$ . baxter.sh [sim]
$ roslaunch baxter_pick_and_place demonstration.launch [gazebo:=true] [record_vis:=true]
```
where the optional `gazebo` flag indicates whether the simulation environment needs to be started and `record_vis` records the visualization topic into a rosbag file and stores it in /tmp/pnp-vis*.
In another terminal run the instruction module by typing
```bash
$ cd $WS_HBCF
$ . baxter.sh [sim]
$ rosrun baxter_pick_and_place instruct.py
```
