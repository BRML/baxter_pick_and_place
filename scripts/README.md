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


### Preliminaries

TBD


### Calibration

TBD


### Run the Experiment

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


#### On the Robot

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


#### In Simulation

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
