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

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel
)
from geometry_msgs.msg import (
    Pose,
    Point
)
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
        rospy.loginfo("Waiting for Gazebo to be fully started ...")
        rospy.wait_for_message("/robot/sim/started", Empty)
    else:
        rospy.loginfo("We are running on the real baxter.")
    return sim


def load_gazebo_model(urdf_path):
    """ Load a robot's model XML from a URDF file and serialize it.
    :param urdf_path: The path to the URDF file describing the model.
    :return: The model XML as a string.
    """
    rospy.logdebug("Loading model XML from file.")
    with open(urdf_path) as urdf_file:
        model_xml = urdf_file.read().replace('\n', '')
    return model_xml


def spawn_gazebo_model(model_xml, model_name, robot_namespace,
                       model_pose=Pose(position=Point(x=0.0, y=0.0, z=0.0)),
                       model_reference_frame="world"):
    """ Spawn a Gazebo model from its XML string.
    :param model_xml: A string describing the model.
    :param model_name: The name of the model to be spawn.
    :param robot_namespace: Spawn robot and all ROS interfaces under this
        namespace.
    :param model_pose: The initial pose of the robot.
    :param model_reference_frame: 'model_pose' is defined relative to the
        frame of this model/body.
    :return: (bool success, string status_message)
    """
    service_spawn = '/gazebo/spawn_urdf_model'

    rospy.logdebug("Waiting for service {0}".format(service_spawn))
    rospy.wait_for_service(service_spawn)
    resp_urdf = tuple()
    rospy.logdebug("Calling service {0}".format(service_spawn))
    try:
        spawn_urdf = rospy.ServiceProxy(service_spawn, SpawnModel)
        rospy.logdebug("Spawning {0} model".format(model_name))
        resp_urdf = spawn_urdf(model_name=model_name,
                               model_xml=model_xml,
                               robot_namespace=robot_namespace,
                               initial_pose=model_pose,
                               reference_frame=model_reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed:", e)
    return resp_urdf


def delete_gazebo_model(model):
    """Delete a given Gazebo model.
    We do not wait for the Gazebo DeleteModel service, since Gazebo should
    already be running. If the service is not available, it is fine to error
    out.

    :param model: The name of the model to delete.
    :return: (bool success, string status_message)
    """
    service_delete = '/gazebo/delete_model'

    resp_delete = tuple()
    rospy.logdebug("Calling service {0}".format(service_delete))
    try:
        delete_model = rospy.ServiceProxy(service_delete, DeleteModel)
        rospy.logdebug("Deleting {0} model".format(model))
        resp_delete = delete_model(model)
    except rospy.ServiceException as e:
        rospy.logerr("Delete model service call failed:", e)
    return resp_delete


def delete_gazebo_models(models):
    """ Delete Gazebo models.
    Call this on ROS exit. We do not wait for the Gazebo Delete Model
    service, since Gazebo should already be running. If the service is not
    available since Gazebo has been killed, it is fine to error out.
    :param models: A list of models to delete.
    :return: list((bool success, string status_message))
    """
    service_delete = '/gazebo/delete_model'

    if not isinstance(models, list):
        models = list(models)
    resp_deletes = list()
    rospy.logdebug("Calling service {0}".format(service_delete))
    try:
        delete_model = rospy.ServiceProxy(service_delete, DeleteModel)
        for model in reversed(models):
            rospy.logdebug("Deleting {0} model".format(model))
            resp_deletes.append(delete_model(model))
    except rospy.ServiceException as e:
        rospy.logerr("Delete model service call failed:", e)
    return resp_deletes
