Helios Autonomous
=================

This project aims to be the repository to basic control of the Catamaran boat Helios from ENSTA Bretagne.

In this repository you will find:
  * Line following control
  * Waypoints navigation
  * Joystick control
  * Keep station behavior



## Prerequisites
Install all deps:
```
sudo apt-get install ros-kinetic-geometry-msgs ros-kinetic-nav-msgs python-pyproj python-scipy
```

## ROS Nodes
### sim_control
Node to simulate the command passed to a robot in absence of a real one.

#### Subscribed topics
`geometry_msgs/Twist` - cmd_vel: Velocity command to be applied.
#### Published topics
`geometry_msgs/PoseStamped` - pose: Estimated pose.
___

### navigation_autonomy_state
State machine to determine the boats behavior

#### Subscribed topics
`std_msgs/Int8` - cmd_state: Current state to be target by the robot.

`nav_msgs/Path` - new_waypoints_mission: Path to be followed by the robot.

`geometry_msgs/PoseStamped` - pose: Robot's current pose.

#### Published topics
`nav_msgs/Path` - line: Segment the robot should follow right away (to be pursuit by LineFollowing).

`geometry_msgs/Pose` - goal: Point the robot should follow right away (to be pursuit by WaypointNavigation).

`geometry_msgs/Pose` - keep_pose: Point the robot should remain (to be pursuit by KeepStation).
___

### line_following
Performs the line following regulation. Commands the robot to perform a specific trajectory according informed by `navigation_autonomy_state`.

#### Subscribed topics
`geometry_msgs/PoseStamped` - pose: Robot's current pose.

`nav_msgs/Path` - line: Segment the robot should follow right away.
#### Published topics
`geometry_msgs/Twist` - cmd_vel: Velocity command to be applied.
___

### waypoint_navigation
Commands the robot to a specific waypoints according informed by `navigation_autonomy_state`. It is implemented as a line following command from the robots current pose to the waypoint.

#### Subscribed topics
`geometry_msgs/PoseStamped` - pose: Robot's current pose.

`geometry_msgs/Pose` - goal: Point the robot should follow right away.
#### Published topics
`geometry_msgs/Twist` - cmd_vel: Velocity command to be applied.
___

### draw_state
A simple plot of robot's current state, mission and pose.

#### Subscribed topics
`nav_msgs/Path` - new_waypoints_mission: Path to be followed by the robot.

`geometry_msgs/PoseStamped` - pose: Robot's current pose.

`std_msgs/Int8` - cmd_state: Current state to be target by the robot.

`nav_msgs/Path` - line: Segment the robot should follow right away (to be pursuit by LineFollowing).

`geometry_msgs/Pose` - goal: Point the robot should follow right away (to be pursuit by WaypointNavigation).

`geometry_msgs/Pose` - keep_pose: Point the robot should remain (to be pursuit by KeepStation).
___

### jsonWaypoints2ros
A simple converter to convert a json file, exported by an ENSTA unnamed tool, to `nav_msgs/Path` waypoints.

#### Subscribed topics
`std_msgs/Bool` - getFileData: Commands the node to (re)publish its waypoints.

`std_msgs/String` - setFileRadiale: Set the file where the node should read the waypoints.

`std_msgs/String` - setJsonRadiale: Json containing all waypoints to be followed and converted to a `nav_msgs/Path` message.
#### Published topics
`nav_msgs/Path` - new_waypoints_mission: Path to be followed by the robot.
