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
$ cd ~/ros_ws
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

### Select camera parameters
To help with the selection of the camera parameters for the baxter robot, a
reconfigurable script exists. To run it, do
```bash
$ cd ~/ros_ws
$ . baxter.sh
$ rosrun baxter_pick_and_place cam_config_server.py
```
This will have the baxter robot move its left limb into a pose hovering over
the table and display the left camera image.

To fire up the parameter reconfigurator, do (in another shell)
```bash
$ cd ~/ros_ws
$ . baxter.sh
$ rosrun rqt_gui rqt_gui -s reconfigure
```
and select the `camera*` server in the window that is opening.

## Train object detection algorithm

### Record (training/validation/test) set images
To run the image recorder, do
```bash
$ cd ~/ros_ws
$ . baxter.sh
$ rosrun baxter_pick_and_place record_image.py --limb L --dirname D
```
where `L=<left, right>` is the limb to record images from and `D` is the
directory to save the recorded images into, relative to the ROS package path
(here defaults to `~/ros_ws/src/baxter_pick_and_place/data`).

Once the program has started, the baxter robot moves the selected limb into a
pose hovering over the table. Follow the explanations in the console, i.e.,
press `r` to record an image (stored under a 12-digit random name in
`dirname`), or press `s` to stop image recording and exit the program.

### Prepare (training/validation/test) set images
To prepare the data for training the object detection algorithm, run the
`scripts/vae_0_dat.ipynb` ipython notebook.
The script splits the recorded images into training, validation and test sets,
and extracts image patches from them that are used for training the object
detection algorithm.

### Train object detection algorithm
To train the variational auto-encoder that we use as a cheap and quick-to-
train alternative to a more complex convolutional neural network, run the
`scripts/vae_1_train.ipynb` ipython notebook.
