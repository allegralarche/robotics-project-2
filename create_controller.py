#!/usr/bin/env python
"""
ROS based interface for the iRobot Create 2, with localization using AprilTag detection.
Written for CIS390 at the University of Pennsylvania
Updated Oct 22, 2015
"""
import roslib
import numpy as np
import numpy.matlib
import sys
import rospy
import cv2

from std_msgs.msg import (
    Header,
    UInt16,
)

from nav_msgs.msg import Odometry

from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    Pose,
    Point,
    Quaternion,
    Twist,
)

from utility import *

class CreateController(object):
    def __init__(self):
        """
        ROS STUFF
        """
        name = rospy.get_param("~myname")
        self._image_fresh=False
        self._odom_fresh=False
        self._no_detection=True
        self._last_image_t = None
        self._last_time = None
        self._robot_t = None
        self._robot_R = None
        self._true_t = None
        self._true_R = None
        self.v = 0.0
        self.omega = 0.0
        # Data saving
        #self.savefile = open('~/Desktop/data.csv','ow')
        # Noise covariances
        self.Q_t = np.eye(3)
        self.R_t = np.eye(3)
        # Start with no information - origin with very large covariance
        self.x_t = np.array([[0,0,0]]).T
        self.P_t = np.array([[10.**6,0,0],[0,10.**6,0],[0,0,10.**6]])
        self._pub = rospy.Publisher("/"+name+"/cmd_vel",Twist,queue_size=10)
        rospy.Subscriber("/"+name+"/tag_detections_pose_noisy",PoseStamped,self._noisy_pose_callback)
        rospy.Subscriber("/"+name+"/tag_detections_pose",PoseStamped,self._ground_truth_pose_callback)

    def _noisy_pose_callback(self,posearray):
        """
        ROS STUFF
        """
        if self._last_image_t is not None:
            self._image_dt = posearray.header.stamp - self._last_image_t
        else:
            self._image_dt = 0
        self._last_image_t = posearray.header.stamp
        self._image_fresh = True
        if posearray.pose.position.z==0:
            self._robot_t = None
            self._robot_R = None
            self._no_detection = True
            return
        
        (self._robot_t, self._robot_R) = get_t_R(posearray.pose)
        self._no_detection = False

    def _ground_truth_pose_callback(self,posearray):
        """
        ROS STUFF
        """
        if posearray.pose.position.z==0:
            self._true_t = None
            self._true_R = None
            return
        (self._true_t, self._true_R) = get_t_R(posearray.pose)
  
    def get_ground_truth_pose(self):
        """
        Gives ground truth pose when noisy marker pose is given. Returns (x,y,theta) as
        3x1 numpy array. Returns None if no new detection is found
        """
        if self._no_detection:
            return None
        # Angle is yaw
        angle = np.arctan2(self._true_R[1,0],self._true_R[0,0])
        dx = self._true_t[0,0]
        dy = self._true_t[1,0]
        return np.array([[dx,dy,angle]]).T

    def F_matrix(self,dt,v,theta_t):
        """
        F matrix given in the prediction equations for the covariance estimate.
        Input: dt - time between two prediction steps (probably 1/60.0)
               v - norm of the velocity of the robot at current time
               theta_t - angle estimated at current time
        Output: 3x3 numpy array F
        """
        return np.array([[1, 0, -dt*v*np.sin(theta_t)],
                         [0, 1,  dt*v*np.cos(theta_t)],
                         [0, 0,                     1]])

    def get_marker_pose(self):
        """
        If a detection is found, returns the robot (x,y and theta) in the marker
        frame as an numpy array, as well as a boolean to determine if a new detection has arrived.
        If a new detection has arrived but no tags were found, returns all None.
        """
        if self._no_detection:
            return None, None
        # Angle is yaw
        angle = np.arctan2(self._robot_R[1,0],self._robot_R[0,0])
        dx = self._robot_t[0,0]
        dy = self._robot_t[1,0]
        fresh = self._image_fresh
        self._image_fresh=False
        return np.array([[dx,dy,angle]]).T, fresh

    def command_velocity(self,vx,wz):
        """
        Commands the robot to move with linear velocity vx and angular
        velocity wz
        """
        twist=Twist()
        twist.linear.x = vx
        twist.angular.z = wz
        self._pub.publish(twist)

    def command_create(self):
        MAX_SPEED=0.1
        """
        YOUR CODE HERE
        This function is called at 60Hz. At each iteration, check if a fresh measurement has come in.
        If so, use your controller to move the create according to the robot pose.
        """
        dt = None
        if self._last_time is not None:
            dt = (rospy.Time.now() - self._last_time).to_sec()
        self._last_time = rospy.Time.now()

        z_t, fresh = self.get_marker_pose()
        if fresh:
            # update step
            K = np.dot(self.P_t, np.linalg.inv(self.P_t + self.R_t))
            self.x_t = self.x_t + np.dot(K, z_t - self.x_t)
            self.P_t = np.dot(np.eye(3) - K, self.P_t)
        elif dt:
            # prediction step
            F = self.F_matrix(dt, self.v, self.x_t[2][0])
            self.x_t[0][0] = self.x_t[0][0] + self.v*dt*np.cos(self.x_t[2][0])
            self.x_t[1][0] = self.x_t[1][0] + self.v*dt*np.sin(self.x_t[2][0])
            self.x_t[2][0] = self.x_t[2][0] + self.omega*dt 
            self.P_t = np.dot(np.dot(F, self.P_t), F.T) + self.Q_t

        self.command_velocity(MAX_SPEED/10, 0) # just move forward in straight line


        # To save out data - for grading purposes
        # g = self.get_ground_truth_pose()
        # self.savefile.write('%f,%f,%f,%f,%f,%f\n' % self.x_t[0,0], self.x_t[1,0], self.x_t[2,0], g[0,0], g[1,0], g[2,0]) 
        return

def main(args):
    rospy.init_node('create_controller')
    controller = CreateController()
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        controller.command_create()
        r.sleep()
    # Done, stop robot
    twist = Twist()
    controller._pub.publish(twist)

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass

