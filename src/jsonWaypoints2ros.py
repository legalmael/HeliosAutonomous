#!/usr/bin/env python
# coding=utf-8

# Author : Guilherme SCHVARCZ FRANCO
# Date : 19/05/2018

import rospy
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from std_msgs.msg import String, Bool
from nav_msgs.msg import Path
import pyproj as proj
import json

UTM30N = proj.Proj("+init=EPSG:32630")
UTM31N = proj.Proj("+init=EPSG:32631")

def latlong2utm(lat, lon):
    return UTM30N(lon, lat)

def utm2latlong(x, y):
    return UTM30N(x, y, inverse=True)

class Json2Waypoints:
    def __init__(self):
        self.pts = []
        self.fileName = rospy.get_param('~file_name', "mission.json")
        self.job = json.loads(open(self.fileName).read())
        self.extractLatLngs(self.job["missions"][0]["radiales"])

        rospy.Subscriber('getFileData', Bool, self.cb_get_file_data)
        rospy.Subscriber('setFileRadiale', String, self.cb_set_file_name)
        rospy.Subscriber('setJsonRadiale', String, self.cb_set_json_radiale)

        self.pub_path = rospy.Publisher('new_waypoints_mission', Path, queue_size=1000000)

    def extractLatLngs(self, radiales):
        sLat, sLon = radiales[0]["start"]["lat"], radiales[0]["start"]["lng"]
        self.pts.append([sLat, sLon])
        for r in radiales:
            eLat, eLon = r["end"]["lat"], r["end"]["lng"]
            self.pts.append([eLat, eLon])

    def cb_get_file_data(self, msg):
        if msg.data:
            self.publishWaypointList()

    def cb_set_json_radiale(self, msg):
        self.job = json.loads(msg.data)
        self.extractLatLngs(self.job["missions"][0]["radiales"])

    def cb_set_file_name(self, msg):
        self.fileName = msg.data
        self.job = json.loads(open(self.fileName).read())
        self.extractLatLngs(self.job["missions"][0]["radiales"])

    def publishWaypointList(self):
        waypoint_msg = Path()
        waypoint_msg.header.stamp = rospy.Time.now()
        waypoint_msg.header.frame_id = "mission_waypoints"

        for pt in self.pts:
            wp = PoseStamped()
            wp.pose.position.x, wp.pose.position.y = latlong2utm(pt[0], pt[1])
            waypoint_msg.poses.append(wp)
        self.pub_path.publish(waypoint_msg)

if __name__ == '__main__':
    rospy.init_node('json2waypoints')
    Json2Waypoints()
    rospy.spin()
