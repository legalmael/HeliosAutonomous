#!/usr/bin/env python
# coding=utf-8

# Author : Guilherme SCHVARCZ FRANCO
# Date : 19/05/2018

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Quaternion
from math import pow, sqrt, atan2, pi, cos, sin, asin
from matplotlib import pyplot as plt
import numpy as np
import tf

plt.ion()

def quaternionToQuaternionMsg(quat):
    q = Quaternion()
    q.x = quat[0]
    q.y = quat[1]
    q.z = quat[2]
    q.w = quat[3]
    return q

def quaternionMsgToQuaternion(quat):
    return ( quat.x, quat.y, quat.z, quat   .w )

class LineFollowing:
    def __init__(self):
        self.path = Path()
        self.cur_pose = PoseStamped()

        self.cur_pose.pose.position.x = 0
        self.cur_pose.pose.position.y = 0
        self.cur_pose.pose.orientation = quaternionToQuaternionMsg(tf.transformations.quaternion_from_euler(0, 0, pi/6))

        self.maxDistance = rospy.get_param('max_distance', 3)
        self.maxRotation = rospy.get_param('max_rotation', pi/2)

        self.rate = rospy.Rate(rospy.get_param('rate', 3))

        rospy.Subscriber('pose', PoseStamped, self.cb_pose)
        rospy.Subscriber('line', Path, self.cb_path)
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10000)

    def calculateCtrl(self):
        cmd_vel = Twist()

        if (len(self.path.poses) >= 2):
            pt1, pt2 = self.path.poses[0].pose, self.path.poses[1].pose
            rospy.loginfo("RobotsPose: %f, %f", self.cur_pose.pose.position.x, self.cur_pose.pose.position.y)
            rospy.loginfo("Pt1: %f, %f", pt1.position.x, pt1.position.y)
            rospy.loginfo("Pt2: %f, %f", pt2.position.x, pt2.position.y)
            rospy.loginfo("Distance: %f, DesiredRotation: %f", self.distanceLine(), self.desiredRotation())
            if not self.goalReached():
                cmd_vel.linear.x = 10
                cmd_vel.angular.z = self.desiredRotation()
            self.pub_cmd.publish(cmd_vel)

    def cb_pose(self, msg):
        self.cur_pose = msg

    def cb_path(self, msg):
        self.path = msg
        rospy.loginfo("New mission received : %s", self.path.poses)

    def goalReached(self):
        pt1, pt2 = self.path.poses[0].pose, self.path.poses[1].pose
        x, y, x1, y1, x2, y2 = self.cur_pose.pose.position.x, self.cur_pose.pose.position.y, pt1.position.x, pt1.position.y, pt2.position.x, pt2.position.y
        return ((x1-x2)*(x-x2) + (y1-y2)*(y-y2)) < 0.

    def distanceLine(self):
        pt1, pt2 = self.path.poses[0].pose, self.path.poses[1].pose
        x, y, x1, y1, x2, y2 = self.cur_pose.pose.position.x, self.cur_pose.pose.position.y, pt1.position.x, pt1.position.y, pt2.position.x, pt2.position.y
        return ((y2-y1)*x - (x2-x1)*y + x2*y1 - y2*x1) / sqrt(pow(y2-y1,2) + pow(x2-x1,2))

    def distanceGoal(self):
        pt1, pt2 = self.path.poses[0].pose, self.path.poses[1].pose
        x, y, x1, y1, x2, y2 = self.cur_pose.pose.position.x, self.cur_pose.pose.position.y, pt1.position.x, pt1.position.y, pt2.position.x, pt2.position.y
        return sqrt(pow(y2-y,2) + pow(x2-x,2))

    def angleLine(self):
        pt1, pt2 = self.path.poses[0].pose, self.path.poses[1].pose
        x1, y1, x2, y2 = pt1.position.x, pt1.position.y, pt2.position.x, pt2.position.y
        return atan2(y2-y1, x2-x1)

    def desiredAngle(self):
        return self.angleLine() + self.maxRotation*max(min(self.distanceLine()/self.maxDistance,1.0),-1.0)

    def desiredRotation(self):
        dAngle = self.desiredAngle()
        yaw = self.getYaw()

        angleGoal = asin(sin(dAngle)*cos(yaw) - cos(dAngle)*sin(yaw))

        dotGoal = cos(dAngle)*cos(yaw) + sin(dAngle)*sin(yaw)
        if dotGoal < 0:
            angleGoal =  np.sign(angleGoal)*pi - angleGoal
        return angleGoal

    def getYaw(self):
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternionMsgToQuaternion(self.cur_pose.pose.orientation))
        return yaw

    def run(self):
        while not rospy.is_shutdown():
            self.calculateCtrl()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('line_following')
    LineFollowing().run()
