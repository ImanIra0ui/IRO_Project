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
ROBOT_RADIUS = 0.105 / 2.
CYLINDER_POSITION = np.array([.5, .6], dtype=np.float32)
CYLINDER_RADIUS = .3
GOAL_POSITION = np.array([1.5, 1.5], dtype=np.float32)
MAX_SPEED = .25
EPSILON = .2
WIDTH = 64
P_COEFFICIENT = 0.5

X = 0
Y = 1
YAW = 2
bridge = CvBridge()

def process_image(img):
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    lower_red = np.array([50, 150, 0])
    upper_red = np.array([200, 230, 255])

    lower_gray = np.array([0, 0, 0])
    upper_gray = np.array([255, 10, 255])

    mask = cv2.inRange(img, lower_red, upper_red)
    mask2 = cv2.inRange(img, lower_gray, upper_gray)

    # Find the largest segmented contour (red ball) and its center
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours2, _ = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    velocity = 0
    u = 0.  # [m/s]
    w = 0.  # [rad/s] going counter-clockwise.

    # MISSING: Implement a braitenberg controller that takes the range
    # measurements given in argument to steer the robot.
    max_u = .2    
    r = 0.033
    d = 0.16

    #if goal
    if(contours):
    #     output = cv2.bitwise_and(img, img, mask = mask)
	# # show the images
    #     cv2.imshow("images", np.hstack([img, output]))
    #     cv2.waitKey(0)
    
        print("GOAL")
        largest_contour = max(contours, key=cv2.contourArea)
        largest_contour_center = cv2.moments(largest_contour)
        if(largest_contour_center['m00'] != 0):
            center_x = int(largest_contour_center['m10'] / largest_contour_center['m00'])
        else:
            center_x = int(largest_contour_center['m10'])

        # Find error (ball distance from image center)
        error = WIDTH / 2 - center_x
        print(error)    
        print("woohoo", error== -3.0)
        
        if(error == -3.0 or error == -2.0):
            print("HERE")
            u = max_u
            w = .0

            return u, w

        # Use simple proportional controller to follow the ball
        velocity  = error * P_COEFFICIENT

        u += r/2*velocity
        w += r/d*velocity

        return u, w

    #if obstacle
    if(contours2):
        print("OBSTACLE")
        largest_contour = max(contours2, key=cv2.contourArea)
        largest_contour_center = cv2.moments(largest_contour)
        if(largest_contour_center['m00'] != 0):
            center_x = int(largest_contour_center['m10'] / largest_contour_center['m00'])
        else:
            center_x = int(largest_contour_center['m10'])

        # Find error (ball distance from image center)
        error = WIDTH / 2 - center_x

        # Use simple proportional controller to avoid the obstacle
        velocity = error * P_COEFFICIENT

        if(velocity == 0):
            u = max_u
            w = .0

        else:

            u += r/2*velocity
            w += - r/d*velocity

    return u, w


def cap(v, max_speed):
  n = abs(v)
  if n > max_speed:
    return v / n * max_speed
  return v

def feedback_linearized(pose, velocity, epsilon):
    u = 0.  # [m/s]
    w = 0.  # [rad/s] going counter-clockwise.

    if pose[0] == 1.5 and pose[1] == 1.5:
        return u, w

    theta = pose[2]
    
    x_dot = velocity[0]
    y_dot = velocity[1]


    u = x_dot*np.cos(theta) + y_dot*np.sin(theta)
    w = (1/epsilon)*((-x_dot * np.sin(theta)) + (y_dot*np.cos(theta)))

    if(u>MAX_SPEED):
        u = MAX_SPEED

    u = np.float64(u)

    return u, -w


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
        super().__init__('potential_field_navigation')
        self.image = [[]]
        self._publisher = self.create_publisher(Twist, 'cmd_vel', 5)
        # Keep track of groundtruth position for plotting purposes.
        self._groundtruth = GroundtruthPose()
        self._groundtruth_subscriber = self.create_subscription(Odometry, 'odom',
                                                                self._groundtruth.callback, 5)
        self.camera_subscriber = self.create_subscription(Image, 'TurtleBot3Burger/camera', self.image_callback, 5)
        share_tmp_dir = os.path.join(get_package_share_directory('part2'), 'tmp')
        os.makedirs(share_tmp_dir, exist_ok=True)
        file_path = os.path.join(share_tmp_dir, 'potential_field_logging.txt')
        self._temp_file = file_path
        self._pose_history = []
        self._vis = False
        self._rate_limiter = self.create_timer(timer_period_sec=0.1, callback=self.timer_callback)
        
        with open(self._temp_file, 'w+'):
          pass

    def image_callback(self, msg):
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # img = msg
        img = np.asarray(cv_image, dtype=np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
        img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        self.image = cv2.flip(img, 1)

    def timer_callback(self):
        # Make sure all measurements are ready.
        if not self._groundtruth.ready:
            return

        # Get velocity.
        u, w = process_image(self.image)
        vel_msg = Twist()
        vel_msg.linear.x = u
        vel_msg.angular.z = w
        self._publisher.publish(vel_msg)

        # Log groundtruth positions in tmp/potential_field_logging.txt
        self._pose_history.append(self._groundtruth.pose)
        if len(self._pose_history) % 10:
          with open(self._temp_file, 'a') as fp:
            fp.write('\n'.join(','.join(str(v) for v in p) for p in self._pose_history) + '\n')
            self._pose_history = []


def run(args):
    rclpy.init()

    potential_field_navigation_node = PotentialFieldNavigation(args)

    rclpy.spin(potential_field_navigation_node)

    potential_field_navigation_node.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Runs potential field navigation')
    args, unknown = parser.parse_known_args()
    run(args)


if __name__ == '__main__':
    main()
