# The Baxter pick-and-place demonstration
Pick &amp; place demonstration with the baxter research robot in collaboration 
with DFKI Saarbruecken.
The framework heavily builds upon the 
[Baxter SDK](https://github.com/RethinkRobotics) and depends on customized 
versions of [baxter_interface](https://github.com/lude-ma/baxter_interface.git) 
and [baxter_common](https://github.com/lude-ma/baxter_common.git) from Rethink 
Robotics.


## Description of demonstration
An eye tracker is used to select and trigger an object lying on a table in 
front of the baxter robot. A network service is used to classify the selected 
object and returns 5 object labels and their respective probabilities of being 
the object being asked for. 

The baxter robot looks for objects on the table, selects the most probable 
object and estimates its pose on the table using machine learning techniques, 
picks it up and places it in a bin it detected previously.


## Description of software
In this [ROS](http://www.ros.org/) package the previously described pick-and-
place demonstration with the
[Baxter research robot](http://www.rethinkrobotics.com/research-education/) 
is implemented.

The demonstration can be run both on the real robot as well as in the 
[Gazebo](http://gazebosim.org/)-powered 
[Baxter simulator](http://sdk.rethinkrobotics.com/wiki/Baxter_Simulator).


## How to install and use
The Baxter pick and place demonstration is implemented as a ROS package.
It requires a development workstation with 
[Ubuntu 14.04](http://releases.ubuntu.com/14.04/) and 
[ROS Indigo](http://wiki.ros.org/indigo) installed.

> Note: If you have Ubuntu, ROS and and the Baxter SDK dependencies already 
> installed, you only need to perform steps 3, 5 and 6 to clone, install and 
> setup the Baxter data acquisition framework!

### Step 1: Install Ubuntu
Follow the standard Ubuntu Installation Instructions for 14.04 (Desktop).

### Step 2: Install ROS Indigo
Configure your Ubuntu repositories to allow "restricted," "universe," and 
"multiverse."

#### Setup your sources.list
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
```

#### Setup your keys
```bash
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
```

#### Verify latest debians, install ROS Indigo Desktop Full and rosinstall
```bash
$ sudo apt-get update
$ sudo apt-get install ros-indigo-desktop-full
$ sudo rosdep init
$ rosdep update
$ sudo apt-get install python-rosinstall
```

### Step 3: Create ROS workspace
```bash
$ mkdir -p ~/ros_baxter_pnp_ws/src
$ source /opt/ros/indigo/setup.bash
$ cd ~/ros_baxter_pnp_ws
$ catkin_make
$ catkin_make install
```

### Step 4: Install Baxter SDK dependencies
```bash
$ sudo apt-get update
$ sudo apt-get install git-core python-argparse python-wstool python-vcstools python-rosdep ros-indigo-control-msgs ros-indigo-joystick-drivers
```

### Step 5: Install this package and its dependencies
Using the [wstool](http://wiki.ros.org/wstool) workspace tool, you will 
checkout all required Github repositories into your ROS workspace source 
directory.
```bash
$ cd ~/ros_baxter_pnp_ws/src
$ wstool init .
$ wstool merge https://raw.githubusercontent.com/lude-ma/baxter_rosinstall/master/baxter_pnp.rosinstall
$ wstool update
$ source /opt/ros/indigo/setup.bash
$ cd ~/ros_baxter_pnp_ws
$ catkin_make
$ catkin_make install
```

### Step 6: Configure Baxter communication/ROS workspace
The [baxter.sh](http://sdk.rethinkrobotics.com/wiki/Baxter.sh) script is a 
convenient script which allows for intuitive modification of the core ROS 
environment components. 
This user edited script will allow for the quickest and easiest ROS setup.
Further information and a detailed description is available on the 
[baxter.sh](http://sdk.rethinkrobotics.com/wiki/Baxter.sh) page.

#### Download the baxter.sh script
```bash
$ cd ~/ros_baxter_pnp_ws
$ wget https://github.com/RethinkRobotics/baxter/raw/master/baxter.sh
$ chmod u+x baxter.sh
```

#### Customize the baxter.sh script
Using your favorite editor, edit the baxter.sh shell script making the 
necessary modifications to describe your development workstation.

- Edit the `baxter_hostname` field to match the hostname of your Baxter 
robot.
- Edit **either** the `your_ip` **or** the `your_hostname` field to 
match the IP or hostname of your development workstation.
Only one of those fields can be active at a time.
The other variable should be commented out!

#### Initialize your SDK environment
```bash
$ cd ~/ros_baxter_pnp_ws
$ . baxter.sh
```

#### Verify environment
To verify that all your changes are applied correctly, perform
```bash
$ env | grep ROS
```
The important fields at this point are

- **ROS_MASTER_URI** (this should now contain your robot's hostname)
- **ROS_IP** or **ROS_HOSTNAME** (this should now contain your development
workstation's ip address or hostname. The unused field should **not** be 
available!)


## Run the demonstration
To run the demonstration, initialize your SDK environment and `rosrun` the 
demonstration.
That is, do
```bash
$ cd ~/ros_baxter_pnp_ws
$ . baxter.sh
$ rosrun baxter_pick_and_place demonstration.py -n N
```
where `N` is the number of objects the robot is supposed to pick up and place 
in a bin.

Use the `-h` command line option to learn more about the demonstration and its
required and optional parameters.


## Run the demonstration in simulation mode
To start up the simulation environment (Gazebo) and run the demonstration, 
initialize your SDK environment in simulation mode, `roslaunch` the simulator
and `rosrun` the demonstration.
That is, in a terminal do
```bash
$ cd ~/ros_baxter_pnp_ws
$ . baxter.sh sim
$ roslaunch baxter_pick_and_place simulation.launch
```
In another terminal do
```bash
$ cd ~/ros_baxter_pnp_ws
$ . baxter.sh sim
$ rosrun baxter_pick_and_place demonstration.py -n N
```
where `N` describes the number of objects to pick up as it does for the 
demonstration with the real Baxter robot.

Note: The `roslaunch` files have parameters to modify their behavior. Please
have a look at the files for more information.


### Demonstration convenience launch file
There also is a launch file that collect the two separate steps above into
one file.
To start up the simulation environment and run the demonstration, do
```bash
$ cd ~/ros_baxter_pnp_ws
$ . baxter.sh sim
$ roslaunch baxter_pick_and_place demonstration.launch number:=N
```
