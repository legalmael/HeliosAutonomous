#!/usr/bin/env python
# coding=utf-8

# Author : Simon CHANU & Guilherme SCHVARCZ FRANCO
# Date : 06/03/2018
# Usage : Si on n'est pas en mode manuel, lorsque un waypoint est atteint, on envoie la ligne suivante

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Int8
from math import pow, sqrt, atan2, pi, cos, sin, asin

MANUAL_CONTROL = 0
WAYPOINT_NAV = 1
LINE_FOLLOWING = 2
KEEP_POSITION = 3

import pyproj as proj

UTM30N = proj.Proj("+init=EPSG:32630")
UTM31N = proj.Proj("+init=EPSG:32631")

def latlong2utm(lat, lon):
    return UTM30N(lon, lat)

def utm2latlong(x, y):
    return UTM30N(x, y, inverse=True)

class PathPlanner:
    def __init__(self):
        self.cur_state = rospy.get_param('~start_state', LINE_FOLLOWING)
        self.cur_pose = PoseStamped()
        self.last_wp = PoseStamped()
        self.next_wp = PoseStamped()
        self.current_line = Path()
        self.current_line.poses = [self.last_wp, self.next_wp]
        self.path = Path()
        self.path.poses = self.current_line.poses
        self.line_number = 0

        self.rate = rospy.Rate(rospy.get_param('~rate', 5))

        self.gotostart = False

        # Subscriber
        rospy.Subscriber('new_waypoints_mission', Path, self.cb_path)
        rospy.Subscriber('pose', PoseStamped, self.cb_pose)
        rospy.Subscriber('cmd_state', Int8, self.cb_cmd_state)

        # Publisher
        self.pub_line = rospy.Publisher('line', Path, queue_size=10000)
        self.pub_waypoint = rospy.Publisher('goal', Pose, queue_size=10000)
        self.pub_keep_pose = rospy.Publisher('keep_pose', Pose, queue_size=10000)

    def update_line(self):
        self.line_number = min(self.line_number, len(self.path.poses)-1)
        self.last_wp = self.path.poses[self.line_number-1]
        self.next_wp = self.path.poses[self.line_number]
        self.current_line.poses = [self.last_wp, self.next_wp]
        rospy.loginfo("update_line : following last_wp x:%f y:%f new_wp x:%f y:%f", self.last_wp.pose.position.x, self.last_wp.pose.position.y, self.next_wp.pose.position.x, self.next_wp.pose.position.y)

    def update_waypoint(self):
        self.line_number = min(self.line_number, len(self.path.poses)-1)
        self.last_wp = self.cur_pose
        self.next_wp = self.path.poses[self.line_number]
        self.current_line.poses = [self.last_wp, self.next_wp]
        rospy.loginfo("update_line : following last_wp x:%f y:%f new_wp x:%f y:%f", self.last_wp.pose.position.x, self.last_wp.pose.position.y, self.next_wp.pose.position.x, self.next_wp.pose.position.y)

    def cb_path(self, msg):
        self.path = msg
        #Because everything is coming in latlog from openCPN
        # for i in range(len(self.path.poses)):
        #     self.path.poses[i].pose.position.x, self.path.poses[i].pose.position.y = latlong2utm(self.path.poses[i].pose.position.y, self.path.poses[i].pose.position.x)

        rospy.loginfo("New mission received : %s", self.path.poses)

        self.line_number = 0

        self.gotostart = True

        if self.cur_state == WAYPOINT_NAV:
            rospy.loginfo("path : sendig last_wp x:%f y:%f new_wp x:%f y:%f", self.last_wp.pose.position.x, self.last_wp.pose.position.y, self.next_wp.pose.position.x, self.next_wp.pose.position.y)
            self.update_waypoint()
            self.pub_waypoint.publish(self.current_line.poses[1].pose)
        if self.cur_state == LINE_FOLLOWING:
            self.update_waypoint()
            self.pub_waypoint.publish(self.current_line.poses[1].pose)
            rospy.loginfo("path : sendig last_wp x:%f y:%f new_wp x:%f y:%f", self.last_wp.pose.position.x, self.last_wp.pose.position.y, self.next_wp.pose.position.x, self.next_wp.pose.position.y)

    def cb_pose(self, msg):
        self.cur_pose = msg
        if (self.distanceGoal() > 10 ) and ((self.cur_state == WAYPOINT_NAV) or self.gotostart):
            self.update_waypoint()

    def cb_cmd_state(self, msg):
        # 0 : manual mode
        # 1 : waypoint navigation
        # 2 : linefollow
        # 3 : stationkeeping
        last_state = self.cur_state
        self.cur_state = msg.data
        rospy.loginfo("Cmd received, switching to %d", msg.data)
        rospy.loginfo("Current state : %d", self.cur_state)
        rospy.loginfo("last_state : %d", last_state)

        if self.cur_state != last_state:
            if self.cur_state == MANUAL_CONTROL:
                self.pub_line.publish(Path())
                self.pub_waypoint.publish(Pose())
                self.pub_keep_pose.publish(Pose())
            elif self.cur_state == WAYPOINT_NAV:
                self.update_waypoint()
                self.pub_line.publish(Path())
                self.pub_waypoint.publish(self.current_line.poses[1].pose)
                self.pub_keep_pose.publish(Pose())
            elif self.cur_state == LINE_FOLLOWING:
                if self.gotostart:
                    self.update_waypoint()
                    self.pub_waypoint.publish(self.current_line.poses[1].pose)
                    self.pub_line.publish(Path())
                else:
                    self.update_line()
                    self.pub_line.publish(self.current_line)
                    self.pub_waypoint.publish(Pose())
                self.pub_keep_pose.publish(Pose())
            elif self.cur_state == KEEP_POSITION:
                self.pub_line.publish(Path())
                self.pub_waypoint.publish(Pose())
                self.pub_keep_pose.publish(self.cur_pose.pose)

    def verifyPose(self):
        if self.cur_state == MANUAL_CONTROL:
            pass
        elif self.cur_state == WAYPOINT_NAV:
            if self.goalReached():
                self.nextWaypointGoal()
        elif self.cur_state == LINE_FOLLOWING:
            if self.goalReached():
                self.nextLineGoal()
        elif self.cur_state == KEEP_POSITION:
            pass

    def nextLineGoal(self):
        if len(self.path.poses) and (self.line_number < len(self.path.poses)-2):
            if self.gotostart:
                self.gotostart = False
                self.pub_waypoint.publish(Pose())
            self.line_number += 1
            self.update_line()
            self.pub_line.publish(self.current_line)
        else:
            self.pub_line.publish(Path())

    def nextWaypointGoal(self):
        if len(self.path.poses) and (self.line_number < len(self.path.poses)-1):
            self.line_number += 1
            self.update_waypoint()
            self.pub_waypoint.publish(self.current_line.poses[1].pose)
        else:
            self.pub_waypoint.publish(Pose())

    def goalReached(self):
        pt1, pt2 = self.current_line.poses[0].pose, self.current_line.poses[1].pose
        x, y, x1, y1, x2, y2 = self.cur_pose.pose.position.x, self.cur_pose.pose.position.y, pt1.position.x, pt1.position.y, pt2.position.x, pt2.position.y
        return ((x1-x2)*(x-x2) + (y1-y2)*(y-y2)) < 0.

    def distanceGoal(self):
        x, y, goalX, goalY = self.cur_pose.pose.position.x, self.cur_pose.pose.position.y, self.current_line.poses[1].pose.position.x, self.current_line.poses[1].pose.position.y
        return sqrt(pow(goalX-x,2) + pow(goalY-y,2))

    def run(self):
        while not rospy.is_shutdown():
            self.verifyPose()
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('navigation_autonomy_state')
    PathPlanner().run()
