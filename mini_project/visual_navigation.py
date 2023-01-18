#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import sys
import matplotlib.pylab as plt
import cv2
from cv_bridge import CvBridge

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
# For pose information.
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry

WALL_OFFSET = 2.
WIDTH = 0

X = 0
Y = 1
YAW = 2
bridge = CvBridge()

def get_velocity(img):
    lower_red = np.array([161, 155, 84])
    upper_red = np.array([179, 255, 255])

    mask = cv2.inRange(img, lower_red, upper_red)

    contours, _ = cv2.findContours(mask, mode=cv2.RETR_EXTERNAL, method= cv2.CHAIN_APPROX_NONE)

    max_u = .2

    u = max_u 
    w = 0. 
       
    r = 0.33
    d = 0.105 

    #if goal
    if(contours):

        if(contours[0][0][0][0] == 0):
            u = .0
            w = .0

            return u, w

        return u, w

    else:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edged = cv2.Canny(gray, 30, 200)
        contours, _ = cv2.findContours(image=edged, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)

        #if obstacle
        if(contours):
            largest_contour = max(contours, key=cv2.contourArea)
            largest_contour_moments = cv2.moments(largest_contour)

            if(largest_contour_moments['m00'] != 0):
                center_x = int(largest_contour_moments['m10'] / largest_contour_moments['m00'])
            else:
                center_x = int(largest_contour_moments['m10'])

            if(largest_contour[0][0][0]>=21 and largest_contour[0][0][0]<=26):
                return u, w

            center_x = WIDTH/2 - center_x * 0.1

            if(center_x != 0.0):
                u = r/2*abs(center_x)
                w = r/d*center_x

            return u, w

    return u, w


class GroundtruthPose(object):
    def __init__(self):
        self._pose = [np.nan, np.nan, np.nan]

    def callback(self, msg):
        self._pose[0] = msg.pose.pose.position.x
        self._pose[1] = msg.pose.pose.position.y
        _, _, self._pose[2] = R.from_quat([
                                msg.pose.pose.orientation.x,
                                msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z,
                                msg.pose.pose.orientation.w]).as_euler('XYZ')

    @property
    def pose(self):
        return self._pose

    @property
    def ready(self):
        return not np.isnan(self._pose[0])

class PotentialFieldNavigation(Node):
    def __init__(self, args):
        super().__init__('visual_field_navigation')
        self.image = [[]]
        self._publisher = self.create_publisher(Twist, 'cmd_vel', 5)
        # Keep track of groundtruth position for plotting purposes.
        self._groundtruth = GroundtruthPose()
        self._groundtruth_subscriber = self.create_subscription(Odometry, 'odom',
                                                                self._groundtruth.callback, 5)
        self.camera_subscriber = self.create_subscription(Image, 'TurtleBot3Burger/camera', self.image_callback, 5)
        share_tmp_dir = os.path.join(get_package_share_directory('mini_project'), 'tmp')
        os.makedirs(share_tmp_dir, exist_ok=True)
        file_path = os.path.join(share_tmp_dir, 'visual_field_logging.txt')
        self._temp_file = file_path
        self._pose_history = []
        self._vis = False
        self._rate_limiter = self.create_timer(timer_period_sec=0.1, callback=self.timer_callback)
        
        with open(self._temp_file, 'w+'):
          pass

    def image_callback(self, msg):
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        img = np.asarray(cv_image, dtype=np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
        img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        img = cv2.flip(img, 1)
        WIDTH = img.shape[1]
        self.image = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    def timer_callback(self):
        # Make sure all measurements are ready.
        if not self._groundtruth.ready:
            return

        # Get velocity.
        u, w = get_velocity(self.image)
        vel_msg = Twist()
        vel_msg.linear.x = u
        vel_msg.angular.z = w
        self._publisher.publish(vel_msg)

        # Log groundtruth positions in tmp/visual_field_logging.txt
        self._pose_history.append(self._groundtruth.pose)
        if len(self._pose_history) % 10:
          with open(self._temp_file, 'a') as fp:
            fp.write('\n'.join(','.join(str(v) for v in p) for p in self._pose_history) + '\n')
            self._pose_history = []


def run(args):
    rclpy.init()

    visual_field_navigation_node = PotentialFieldNavigation(args)

    rclpy.spin(visual_field_navigation_node)

    visual_field_navigation_node.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Runs visual navigation')
    args, unknown = parser.parse_known_args()
    run(args)


if __name__ == '__main__':
    main()

