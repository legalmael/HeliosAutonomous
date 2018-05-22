#!/usr/bin/env python
# coding=utf-8

# Author : Guilherme SCHVARCZ FRANCO
# Date : 19/05/2018

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Twist, Quaternion
from math import pow, sqrt, atan2, pi, cos, sin, asin
import numpy as np
import tf

def quaternionToQuaternionMsg(quat):
    q = Quaternion()
    q.x = quat[0]
    q.y = quat[1]
    q.z = quat[2]
    q.w = quat[3]
    return q

def quaternionMsgToQuaternion(quat):
    return ( quat.x, quat.y, quat.z, quat   .w )

class SimBoat:
    def __init__(self):
        self.cur_pose = PoseStamped()
        self.cur_pose.pose.position.x = rospy.get_param('~initial_pose_x', 0)
        self.cur_pose.pose.position.y = rospy.get_param('~initial_pose_y', 5)
        yaw = rospy.get_param('~initial_pose_th', 0)
        self.cur_pose.pose.orientation = quaternionToQuaternionMsg(tf.transformations.quaternion_from_euler(0, 0, yaw))

        self.rate = rospy.Rate(rospy.get_param('~rate', 33))
        self.cmd_vel = Twist()
        self.lastTime = rospy.Time.now()

        rospy.Subscriber('cmd_vel', Twist, self.cb_cmd)
        self.pub_pose = rospy.Publisher('pose', PoseStamped, queue_size=10000)

    def getYaw(self):
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternionMsgToQuaternion(self.cur_pose.pose.orientation))
        return yaw

    def cb_cmd(self, msg):
        self.cmd_vel = msg

    def iterate(self):
        dt = (rospy.Time.now() - self.lastTime).to_sec()
        yaw = self.getYaw()
        self.cur_pose.header.stamp = rospy.Time.now()
        self.cur_pose.pose.position.x += dt*self.cmd_vel.linear.x*cos(yaw)
        self.cur_pose.pose.position.y += dt*self.cmd_vel.linear.x*sin(yaw)
        self.cur_pose.pose.orientation = quaternionToQuaternionMsg(tf.transformations.quaternion_from_euler(0, 0, yaw + dt*self.cmd_vel.angular.z))

        self.pub_pose.publish(self.cur_pose)
        self.lastTime = self.cur_pose.header.stamp

    def run(self):
        while not rospy.is_shutdown():
            self.iterate()
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('sim_control')
    SimBoat().run()
