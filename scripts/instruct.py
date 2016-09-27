#!/usr/bin/env python

# Copyright (c) 2015--2016, BRML
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


# http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)


import rospy
from std_srvs.srv import (
    Trigger,
    TriggerResponse
)

from instruction import KeyboardInput


def _instruction_wrapper(trigger):
    """ROS wrapper for the instruction module.

    return: A TriggerResponse containing fields response.success and
    response.message.
    """
    instr = KeyboardInput().instruct()
    rospy.logdebug("Sending instruction {}".format(instr))
    return TriggerResponse(True, instr)


def main():
    """A ROS node starting a ROS server waiting to get triggered. Once it
    is triggered, it requests input from the user and writes the instruction
    to the topic that triggered it.
    """
    print 'Initializing node ...'
    rospy.init_node('demo_instruction_module')
    _ = rospy.Service('demo_instruction', Trigger, _instruction_wrapper)
    rospy.loginfo("Ready to send instruction.")
    rospy.spin()

if __name__ == '__main__':
    main()
