# Installing and Preparing the Human-Baxter Collaboration Framework

The following instructions are adapted from [here](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup).


The Human-Baxter collaboration framework is implemented as a ROS package.
It has been tested with a development workstation with [Ubuntu 14.04](http://releases.ubuntu.com/14.04/) and [ROS Indigo](http://wiki.ros.org/indigo).

> Note: If you have Ubuntu, ROS and and the Baxter SDK dependencies already 
> installed, you only need to perform steps 3, 5 and 6 to clone, install and 
> setup the Human-Baxter collaboration framework!

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
In the following, we assume that the framework is installed in a catkin workspace called `$WS_HBCF`.
```bash
$ mkdir -p $WS_HBCF/src
$ source /opt/ros/indigo/setup.bash
$ cd $WS_HBCF
$ catkin_make
$ catkin_make install
```

### Step 4: Install Baxter SDK- and Baxter simulator dependencies
```bash
$ sudo apt-get update
$ sudo apt-get install git-core python-argparse python-wstool python-vcstools python-rosdep ros-indigo-control-msgs ros-indigo-joystick-drivers
$ sudo apt-get install gazebo2 ros-indigo-qt-build ros-indigo-driver-common ros-indigo-gazebo-ros-control ros-indigo-gazebo-ros-pkgs ros-indigo-ros-control ros-indigo-control-toolbox ros-indigo-realtime-tools ros-indigo-ros-controllers ros-indigo-xacro python-wstool ros-indigo-tf-conversions ros-indigo-kdl-parser
```

### Step 5: Install this package and its dependencies
Using the [wstool](http://wiki.ros.org/wstool) workspace tool, you will 
checkout all required Github repositories into your ROS workspace source 
directory.
```bash
$ cd $WS_HBCF/src
$ wstool init .
$ wstool merge https://raw.githubusercontent.com/BRML/baxter_rosinstall/master/baxter_pnp.rosinstall
$ wstool update
$ source /opt/ros/indigo/setup.bash
$ cd $WS_HBCF
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
