# baxter-pick-and-place
Pick &amp; place demonstration with the baxter research robot in collaboration 
with DFKI Saarbruecken.

## Description of demonstration
An eye tracker is used to select and trigger an object lying on a table in 
front of the baxter robot. A network service is used to classify the selected 
object and returns 5 object labels and their respective probabilities of being 
the object being asked for. 

The baxter robot looks for objects on the table, calls the network service on 
them in turn, selects the most probable object, picks it up and places it in a 
pre-defined location.

## How to install and use
The baxter pick and place demonstration is implemented as a ROS package.

### Install the package
The following steps are required to install the package:
1. If not already done, set up your baxter workstation as explained in the 
[baxter SDK wiki](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup).
1. `$ cd ~/ros_ws`
1. `$ . baxter.sh`
1. `$ cd ~/ros_ws/src`
1. `$ git clone https://github.com/BRML/baxter-pick-and-place ./baxter_pick_and_place`
1. `$ cd ~\ros_ws`
1. `$ catkin_make`
1. `$ catkin_make install`

### Run the demonstration
To run the demonstration go to
`$ cd ~/ros_ws`
`$ . baxter.sh`
`$ rosrun baxter_pick_and_place demonstration.py -n N`, where `N` is the 
number of objects the robot is supposed to pick up and place in a pre-defined 
location.
