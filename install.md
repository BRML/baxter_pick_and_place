# Installing and Preparing the Human-Baxter Collaboration Framework

> The following instructions are adapted in parts from [here](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup).


The Human-Baxter collaboration framework is implemented as a ROS package.
It has been tested with a development workstation with [Ubuntu 14.04](http://releases.ubuntu.com/14.04/) and [ROS Indigo](http://wiki.ros.org/indigo).

> Note: If you have Ubuntu, ROS and and the Baxter SDK dependencies already 
> installed, you only need to perform steps 3, 5 and 6 to clone, install and 
> setup the Human-Baxter collaboration framework!


## Step 1: Install Ubuntu

Follow the standard Ubuntu Installation Instructions for 14.04 (Desktop).


## Step 2: Install ROS Indigo

Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse."


### Setup your sources.list

```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
```


### Setup your keys

```bash
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
```


### Verify latest debians, install ROS Indigo Desktop Full and rosinstall

```bash
$ sudo apt-get update
$ sudo apt-get install ros-indigo-desktop-full
$ sudo rosdep init
$ rosdep update
$ sudo apt-get install python-rosinstall
```


## Step 3: Create ROS workspace

> In the following, we assume that the framework is installed in a catkin workspace called `$WS_HBCF`.

```bash
$ mkdir -p $WS_HBCF/src
$ source /opt/ros/indigo/setup.bash
$ cd $WS_HBCF
$ catkin_make
$ catkin_make install
```


## Step 4: Install Baxter SDK-, Baxter simulator and Kinect dependencies

```bash
$ sudo apt-get update
$ sudo apt-get install git-core python-argparse python-wstool python-vcstools python-rosdep ros-indigo-control-msgs ros-indigo-joystick-drivers
$ sudo apt-get install gazebo2 ros-indigo-qt-build ros-indigo-driver-common ros-indigo-gazebo-ros-control ros-indigo-gazebo-ros-pkgs ros-indigo-ros-control ros-indigo-control-toolbox ros-indigo-realtime-tools ros-indigo-ros-controllers ros-indigo-xacro python-wstool ros-indigo-tf-conversions ros-indigo-kdl-parser
```

To install the Microsoft Kinect V2 for Linux we need the
[libfreenect2](https://github.com/OpenKinect/libfreenect2/blob/master/README.md#linux) library, which we will build from source and install into `~/freenect2`:
```bash
$ sudo apt-get install libturbojpeg libopenni2-dev
$ git clone https://github.com/OpenKinect/libfreenect2.git ~/libfreenect2
$ cd ~/libfreenect2/depends
$ ./download_debs_trusty.sh
$ sudo dpkg -i debs/libglfw3*deb
$ sudo apt-get install -f
$ cd ~/libfreenect2
$ mkdir build
$ cd build
$ cmake .. -DENABLE_CXX11=ON -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
$ make
$ make install
$ sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
```
To verify the installation, plug in the Kinect V2 into an USB 3 port and run the `./bin/Protonect` test program.


## Step 5: Install this package and its dependencies

Using the [wstool](http://wiki.ros.org/wstool) workspace tool, you will checkout all required Github repositories into your ROS workspace source directory.
The [ROS bridge for the Kinect V2](https://github.com/code-iai/iai_kinect2#install) will be installed separately.
```bash
$ cd $WS_HBCF/src
$ wstool init .
$ wstool merge https://raw.githubusercontent.com/BRML/baxter_rosinstall/master/baxter_pnp.rosinstall
$ wstool update
# now install dependencies for iai_kinect2
$ cd iai_kinect2
$ source /opt/ros/indigo/setup.bash
$ rosdep install -r --from-paths .
$ cd $WS_HBCF
$ catkin_make -DCMAKE_BUILD_TYPE="Release"
$ catkin_make install
```

We now install [R-FCN](https://github.com/Orpine/py-R-FCN#requirements-software) for object detection using deep neural networks into `$DEVEL` (default in the framework settings is `~/software`).
Please refer to the [Caffe installation instructions](http://caffe.berkeleyvision.org/installation.html) if anything is unclear.
```bash
$ mkdir $DEVEL
$ cd $DEVEL
# clone R-FCN and Microsoft Caffe commit 1a2be8e
$ git clone https://github.com/Orpine/py-R-FCN.git
$ cd $DEVEL/py-R-FCN
$ git clone https://github.com/Microsoft/caffe.git
$ cd $DEVEL/py-R-FCN/caffe
$ git reset --hard 1a2be8e
# build the Cython modules
$ cd $DEVEL/py-R-FCN/lib
$ make
# with your Makefile.config in place, build Caffe
$ cd $DEVEL/py-R-FCN/caffe
$ make -j8 && make pycaffe
```
**Note**: Caffe *must* be built with support for Python layers!
That is, in your `Makefile.config`, make sure to have this line uncommented
```
WITH_PYTHON_LAYER := 1
```

Optionally, [faster R-CNN](https://github.com/rbgirshick/py-faster-rcnn#requirements-software) and [MNC](https://github.com/daijifeng001/MNC#installation-guide) can be installed in a quite similar fashion.
Please refer to the respective installation instructions for details.


## Step 6: Configure Baxter communication/ROS workspace

The [baxter.sh](http://sdk.rethinkrobotics.com/wiki/Baxter.sh) script is a convenient script which allows for intuitive modification of the core ROS environment components.
This user edited script will allow for the quickest and easiest ROS setup.
Further information and a detailed description is available on the [baxter.sh](http://sdk.rethinkrobotics.com/wiki/Baxter.sh) page.


### Download the baxter.sh script

```bash
$ cd $WS_HBCF
$ wget https://github.com/RethinkRobotics/baxter/raw/master/baxter.sh
$ chmod u+x baxter.sh
```


### Customize the baxter.sh script

Using your favorite editor, edit the baxter.sh shell script making the necessary modifications to describe your development workstation.

- Edit the `baxter_hostname` field to match the hostname of your Baxter robot.
- Edit **either** the `your_ip` **or** the `your_hostname` field to match the IP or hostname of your development workstation.
Only one of those fields can be active at a time.
The other variable should be commented out!


### Initialize your SDK environment

```bash
$ cd $WS_HBCF
$ . baxter.sh
```

### Verify environment

To verify that all your changes are applied correctly, perform
```bash
$ env | grep ROS
```
The important fields at this point are

- **ROS_MASTER_URI** (this should now contain your robot's hostname)
- **ROS_IP** or **ROS_HOSTNAME** (this should now contain your development workstation's ip address or hostname.
The unused field should **not** be available!)
