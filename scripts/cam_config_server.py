#!/usr/bin/env python

# See http://wiki.ros.org/dynamic_reconfigure/Tutorials/SettingUpDynamicReconfigureForANode%28python%29

import baxter_interface
from baxter_interface import CHECK_VERSION
import cv2
import cv_bridge
import numpy as np
import rospy
from tf import transformations

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import (
    Pose,
    PoseStamped
)
from sensor_msgs.msg import Image

from baxter_pick_and_place.cfg import CamConfig
from baxter_pick_and_place.image import (
    resize_imgmsg,
    white_imgmsg
)


class Cam(object):
    def __init__(self):
        self._cam = baxter_interface.CameraController('left_hand_camera')
        self._cam_sub = None
        self._exposure = -1
        self._gain = 0
        self._wb_red = -1
        self._wb_green = -1
        self._wb_blue = -1

        self._display_pub = rospy.Publisher('/robot/xdisplay', Image,
                                            queue_size=10, latch=True)

        self._limb = baxter_interface.Limb('left')
        self._top_pose = [
            0.45,  # x = (+) front, (-) back
            0.0,  # y = (+) left, (-) right
            0.15,  # z = (+) up, (-) down
            -1.0*np.pi,  # roll = horizontal
            0.0*np.pi,  # pitch = vertical
            0.0*np.pi  # yaw = rotation
        ]

        print "\nGetting robot state ... "
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print "Enabling robot... "
        self._rs.enable()

        self._limb.move_to_neutral()
        self._display_pub.publish(white_imgmsg())
        self._cam.resolution = (1280, 800)
        self._cam.fps = 14.0
        self._update()

    def _update(self, exposure=-1, gain=0, wb_red=-1, wb_green=-1, wb_blue=-1):
        s = ""
        if self._exposure != exposure:
            s += 'exp: %i -> %i' % (self._exposure, exposure)
            self._cam.exposure = exposure
            self._exposure = exposure
        if self._gain != gain:
            s += 'gain: %i -> %i' % (self._gain, gain)
            self._cam.gain = gain
            self._gain = gain
        if self._wb_red != wb_red:
            s += 'red: %i -> %i' % (self._wb_red, wb_red)
            self._cam.white_balance_red = wb_red
            self._wb_red = wb_red
        if self._wb_green != wb_green:
            s += 'green: %i -> %i' % (self._wb_green, wb_green)
            self._cam.white_balance_green = wb_green
            self._wb_green = wb_green
        if self._wb_blue != wb_blue:
            s += 'blue: %i -> %i' % (self._wb_blue, wb_blue)
            self._cam.white_balance_blue = wb_blue
            self._wb_blue = wb_blue
        if len(s) > 0:
            rospy.loginfo(s)

    def start(self):
        print 'Starting camera control ...'
        if self._move_to_pose(self._top_pose):
            s = '/cameras/left_hand_camera/image'
            self._cam_sub = rospy.Subscriber(s, Image, callback=self._cam_callback)
        else:
            print 'Failed motion'

    def _cam_callback(self, data):
        """
        Callback routine for the camera subscriber.
        """
        # self._display_pub.publish(resize_imgmsg(data))
        try:
            img = cv_bridge.CvBridge().imgmsg_to_cv2(data, 'bgr8')
        except cv_bridge.CvBridgeError:
            raise
        except AttributeError:
            print 'ERROR-imgmsg2img-Something is wrong with the ROS image message.'
            raise
        cv2.imshow('left camera', img)
        cv2.waitKey(3)

    def stop(self):
        print 'Exiting camera control ...'
        if self._cam_sub is not None:
            self._cam_sub.unregister()
        self._display_pub.publish(white_imgmsg())
        self._limb.move_to_neutral()
        if not self._init_state:
            print "Disabling robot..."
            self._rs.disable()
        cv2.destroyAllWindows()
        return True

    def rec_callback(self, config, level):
        self._update(exposure=config['exposure'], gain=config['gain'],
                     wb_red=config['white_red'], wb_green=config['white_green'],
                     wb_blue=config['white_blue'])
        return config

    def _move_to_pose(self, pose):
        """ Move robot limb to Cartesian pose.
        :type pose: [float, float, float, float, float, float]
        :param pose: desired Cartesian pose
        :return: boolean flag on completion
        """
        try:
            cmd = self._inverse_kinematics(pose)
        except Exception as e:
            print e
            return False
        self._limb.move_to_joint_positions(cmd)
        return True

    def _inverse_kinematics(self, pose=None):
        """ Inverse kinematics of one Baxter arm.
        Compute valid set of joint angles for a given Cartesian pose using the
        inverse kinematics service. Return current joint angles if None pose is
        given.
        :type pose: {'position': (x, y, z), 'orientation': (a, b, c)}
        :param pose: desired Cartesian pose
        :return: valid set of joint angles
        """
        if pose is None:
            return self._limb.joint_angles()

        qp = list_to_pose_stamped(pose, "base")

        node = "ExternalTools/left/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        ik_request.pose_stamp.append(qp)
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            raise

        if ik_response.isValid[0]:
            # convert response to joint position control dictionary
            return dict(zip(ik_response.joints[0].name,
                            ik_response.joints[0].position))
        else:
            s = "inverse kinematics - No valid joint configuration found"
            rospy.logerr(s)
            raise Exception(s)


def list_to_pose(pose_list):
    """ Pose list to ROS Pose msg.
    :param pose_list: pose list
    :return: ROS Pose message
    """
    pose_msg = Pose()
    if len(pose_list) == 7:
        pose_msg.position.x = pose_list[0]
        pose_msg.position.y = pose_list[1]
        pose_msg.position.z = pose_list[2]
        pose_msg.orientation.x = pose_list[3]
        pose_msg.orientation.y = pose_list[4]
        pose_msg.orientation.z = pose_list[5]
        pose_msg.orientation.w = pose_list[6]
    elif len(pose_list) == 6:
        pose_msg.position.x = pose_list[0]
        pose_msg.position.y = pose_list[1]
        pose_msg.position.z = pose_list[2]
        q = transformations.quaternion_from_euler(pose_list[3], pose_list[4],
                                                  pose_list[5])
        pose_msg.orientation.x = q[0]
        pose_msg.orientation.y = q[1]
        pose_msg.orientation.z = q[2]
        pose_msg.orientation.w = q[3]
    else:
        print "Expected either 6 or 7 elements in list: (x,y,z,r,p,y) or (x,y,z,qx,qy,qz,qw)"
    return pose_msg


def list_to_pose_stamped(pose_list, target_frame):
    """ Pose list to ROS PoseStamped msg.
    :param pose_list: pose list
    :param target_frame: name of the target frame
    :return: ROS PoseStamped message
    """
    pose_msg = PoseStamped()
    pose_msg.pose = list_to_pose(pose_list)
    pose_msg.header.frame_id = target_frame
    pose_msg.header.stamp = rospy.Time.now()
    return pose_msg


if __name__ == "__main__":
    rospy.init_node("baxter_pick_and_place", anonymous=True)

    cam = Cam()
    rospy.on_shutdown(cam.stop)
    srv = Server(CamConfig, cam.rec_callback)
    cam.start()
    rospy.spin()
