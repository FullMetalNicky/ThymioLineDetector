#!/usr/bin/env python
import rospy
import sys
import cv2

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from math import cos, sin, asin, tan, atan2
# msgs and srv for working with the set_model_service
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty
import os

# a handy tool to convert orientations
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
from MazeWalker import MazeWalker

PI=3.1415926


class BasicThymio:

    def __init__(self, thymio_name):
        """init"""
        self.thymio_name = thymio_name
        rospy.init_node('basic_thymio_controller', anonymous=True)

        self.current_pose = Pose()
        self.current_twist = Twist()
        self.state = "none"
        # publish at this rate
        self.rate = rospy.Rate(10)  
        self.pose_subscriber = rospy.Subscriber(self.thymio_name + '/odom',Odometry, self.update_state)
        self.tf_listener = tf.TransformListener()       
        self.maze_walker = MazeWalker(thymio_name)


    def thymio_state_service_request(self, position, orientation):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            model_state = ModelState()
            model_state.model_name = self.thymio_name
            model_state.reference_frame = '' # the frame for the pose information
            model_state.pose.position.x = position[0]
            model_state.pose.position.y = position[1]
            model_state.pose.position.z = position[2]
            qto = quaternion_from_euler(orientation[0], orientation[1], orientation[2], axes='sxyz')
            model_state.pose.orientation.x = qto[0]
            model_state.pose.orientation.y = qto[1]
            model_state.pose.orientation.z = qto[2]
            model_state.pose.orientation.w = qto[3]
            # a Twist can also be set but not recomended to do it in a service
            gms = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            response = gms(model_state)
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def update_state(self, data):
        self.current_pose = data.pose.pose
        self.current_twist = data.twist.twist
        quat = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion (quat)
       

    def RandomWalker(self):
         self.maze_walker.RandomWalker()

    def GameLoop(self):
        self.maze_walker.GameLoop()


def usage():
    return "Wrong number of parameters. basic_move.py [thymio_name]"


if __name__ == '__main__':
    if len(sys.argv) == 2:
        thymio_name = sys.argv[1]
        print "Now working with robot: %s" % thymio_name
    else:
        print usage()
        sys.exit(1)
    thymio = BasicThymio(thymio_name)
    thymio.thymio_state_service_request([0.0,-0.5,0.], [0.,0.,0.0])
    rospy.sleep(1.)
 
    thymio.GameLoop()

