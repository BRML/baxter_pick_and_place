# Copyright (c) 2016, BRML
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import rospy
import roswtf

from std_msgs.msg import Empty


def sim_or_real():
    """ Find out whether we are in simulation mode or on the real robot. If
    called from a baxter shell in simulation mode, wait for Gazebo to
    instantiate properly.
    :return: Whether the script is run in a baxter shell initialized by
    '. baxter.sh sim' (True), meaning in simulation mode, or in a baxter shell
     initialized by '. baxter.sh' (False), meaning on the real robot.
    """
    sim = 'localhost' in roswtf.os.environ['ROS_MASTER_URI']
    if sim:
        rospy.loginfo("We are running in simulation mode.")
        # Wait for the 'All Clear' from simulator startup
        rospy.wait_for_message("/robot/sim/started", Empty)
    else:
        rospy.loginfo("We are running on the real baxter.")
    return sim
