#!/usr/bin/env python

# DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
# http://answers.ros.org/question/192723/cant-find-python-scripts-after-sourcing/?answer=192738#post-id-192738

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


d = generate_distutils_setup()
d['packages'] = ['core', 'demo', 'hardware', 'instruction', 'motion_planning', 'servoing', 'simulation', 'vision']
d['package_dir'] = {'': 'src'}
d['requires'] = ['rospy', 'roswtf', 'tf',
                 'geometry_msgs', 'std_msgs', 'std_srvs', 'gazebo_msgs',
                 'baxter_interface', 'baxter_core_msgs',
                 'os', 'time', 'logging', 'numpy',
                 'cv2', 'caffe']
setup(**d)
