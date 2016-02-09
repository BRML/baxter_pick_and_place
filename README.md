# baxter-pick-and-place
Pick &amp; place demonstration with the baxter research robot in collaboration 
with DFKI Saarbruecken.

## Description of demonstration
An eye tracker is used to select and trigger an object lying on a table in 
front of the baxter robot. A network service is used to classify the selected 
object and returns 5 object labels and their respective probabilities of being 
the object being asked for. 

The baxter robot looks for objects on the table, selects the most probable 
object, uses a Haar cascade classifier to detect it on the table, picks it up 
and places it in a bin it detected previously.

## How to install and use
The baxter pick and place demonstration is implemented as a ROS package.

### Install the package
The following steps are required to install the package:

1. If not already done, set up your baxter workstation as explained in the 
[baxter SDK wiki](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup).
1. Clone, build and install the ROS package:
```bash
$ cd ~/ros_ws
$ . baxter.sh
$ cd ~/ros_ws/src
$ git clone https://github.com/BRML/baxter-pick-and-place ./baxter_pick_and_place
$ cd ~\ros_ws
$ catkin_make
$ catkin_make install
```

### Run the demonstration
To run the demonstration, do
```bash
$ cd ~/ros_ws
$ . baxter.sh
$ rosrun baxter_pick_and_place demonstration.py -n N
```
`N` is the number of objects the robot is supposed to pick up and place in a 
bin.


## Train new classifiers
To train new classifiers, the `scripts/opencv_i_*.ipynb` notebooks can be 
used, where `i` is the step in the training pipeline.
The scripts should be very much self-explanatory:

#### 0
Prepare negative (background) samples.

#### 1
Mark-up positive (object) samples, i.e., label the positive samples.

#### 2
Use the marked-up positive samples from step 1 to create an augmented data set.
 
#### 3
Train a classifier using the augmented data set from step 2.

#### 4
For a quick reference, test the trained classifier from step 3 on the not 
augmented data.
