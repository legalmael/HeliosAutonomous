#!/usr/bin/env python
# coding=utf-8

# Author : Guilherme SCHVARCZ FRANCO
# Date : 22/05/2018

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Int8
from matplotlib import pyplot as plt
from math import pow, sqrt, atan2, pi, cos, sin, asin
import tf

MANUAL_CONTROL = 0
WAYPOINT_NAV = 1
LINE_FOLLOWING = 2
KEEP_POSITION = 3

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

class DrawState:
    def __init__(self):
        self.cur_state = rospy.get_param('start_state', LINE_FOLLOWING)
        self.cur_pose = PoseStamped()
        self.cur_waypoint_goal = Pose()
        self.current_line = Path()
        self.current_line.poses = []
        self.path = Path()
        self.path.poses = self.current_line.poses

        self.rate = rospy.Rate(rospy.get_param('rate', 5))
        self.maxDistance = rospy.get_param('max_distance', 3)
        self.maxRotation = rospy.get_param('max_rotation', pi/2)

        # Subscriber
        rospy.Subscriber('new_waypoints_mission', Path, self.cb_path)
        rospy.Subscriber('pose', PoseStamped, self.cb_pose)
        rospy.Subscriber('cmd_state', Int8, self.cb_cmd_state)
        rospy.Subscriber('line', Path, self.cb_line)
        rospy.Subscriber('goal', Pose, self.cb_waypoint)
        rospy.Subscriber('keep_pose', Pose, self.cb_keeppose)

    def cb_path(self, msg):
        self.path = msg

    def cb_pose(self, msg):
        self.cur_pose = msg

    def cb_cmd_state(self, msg):
        # 0 : manual mode
        # 1 : waypoint navigation
        # 2 : linefollow
        # 3 : stationkeeping
        self.cur_state = msg.data

    def cb_line(self, msg):
        self.current_line = msg

    def cb_waypoint(self, msg):
        self.cur_waypoint_goal = msg

    def cb_keeppose(self, msg):
        pass

    def distanceLine(self):
        pt1, pt2 = self.current_line.poses[0].pose, self.current_line.poses[1].pose
        x, y, x1, y1, x2, y2 = self.cur_pose.pose.position.x, self.cur_pose.pose.position.y, pt1.position.x, pt1.position.y, pt2.position.x, pt2.position.y
        try:
            return ((y2-y1)*x - (x2-x1)*y + x2*y1 - y2*x1) / sqrt(pow(y2-y1,2) + pow(x2-x1,2))
        except ZeroDivisionError:
            return 0

    def distanceGoal(self):
        pt1, pt2 = self.current_line.poses[0].pose, self.current_line.poses[1].pose
        x, y, x1, y1, x2, y2 = self.cur_pose.pose.position.x, self.cur_pose.pose.position.y, pt1.position.x, pt1.position.y, pt2.position.x, pt2.position.y
        return sqrt(pow(y2-y,2) + pow(x2-x,2))

    def angleLine(self):
        pt1, pt2 = self.current_line.poses[0].pose, self.current_line.poses[1].pose
        x1, y1, x2, y2 = pt1.position.x, pt1.position.y, pt2.position.x, pt2.position.y
        return atan2(y2-y1, x2-x1)

    def desiredAngle(self):
        return self.angleLine() + self.maxRotation*max(min(self.distanceLine()/self.maxDistance,1.0),-1.0)

    def getYaw(self):
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternionMsgToQuaternion(self.cur_pose.pose.orientation))
        return yaw

    def drawMatplot(self):
        plt.clf()
        yaw = self.getYaw()
        if self.path.poses != []:
            x, y = zip(*[[pose.pose.position.x, pose.pose.position.y] for pose in self.path.poses])
            plt.plot(x, y, "k-")

        if self.cur_waypoint_goal != Pose():
            plt.plot(self.cur_waypoint_goal.position.x, self.cur_waypoint_goal.position.y, "ro")

        if (len(self.current_line.poses) >= 2):
            dAngle = self.desiredAngle()
            plt.plot([self.cur_pose.pose.position.x, self.cur_pose.pose.position.x+cos(dAngle)], [self.cur_pose.pose.position.y, self.cur_pose.pose.position.y+sin(dAngle)], "g-")

            pt1, pt2 = self.current_line.poses[0].pose, self.current_line.poses[1].pose
            plt.plot([pt1.position.x, pt2.position.x], [pt1.position.y, pt2.position.y], "b-")

        plt.plot([self.cur_pose.pose.position.x, self.cur_pose.pose.position.x+cos(yaw)], [self.cur_pose.pose.position.y, self.cur_pose.pose.position.y+sin(yaw)], "r-")
        plt.plot(self.cur_pose.pose.position.x, self.cur_pose.pose.position.y, "ro")

        plt.axis('equal')
        plt.pause(0.1)

    def run(self):
        while not rospy.is_shutdown():
            self.drawMatplot()
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('draw_state')
    DrawState().run()
