#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import sys
import math
from copy import deepcopy

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS         = 100    # Number of waypoints we will publish
UPDATE_RATE           = 20     # The rate in Hz

MAX_DECELERATION      = 10.0


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.twist_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        self.current_pose = None
        self.current_velocity = None
        self.current_waypoints = None
        self.current_traffic_waypoint = None
        self.latest_traffic_waypoint = None
        self.latest_obstacle_waypoint = None
        
        self.maximum_velocity = self.get_maximum_velocity()

    def pose_cb(self, msg):
        self.current_pose = PoseStamped()
        self.current_pose = msg.pose
        self.update_waypoints()

    def twist_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

    def waypoints_cb(self, waypoints):
        self.current_waypoints = waypoints

    def traffic_cb(self, msg):
        self.latest_traffic_waypoint = msg.data
        self.update_waypoints()

    def obstacle_cb(self, msg):
        self.latest_obstacle_waypoint = msg.data
        self.update_waypoints()
    
    def update_waypoints(self):
        if self.current_waypoints == None or self.current_pose == None:
            rospy.logwarn("Waypoints or pose not available.")
            return
        
        velocity = self.current_velocity
        
        lane_waypoints = Lane()
        lane_waypoints.header.stamp = rospy.Time.now()
        lane_waypoints.header.frame_id = '/world'
        
        wp_ahead = self.get_waypoint_ahead()
        wp_to_stop = self.get_next_stop_waypoint(wp_ahead)
        
        max_dv = max(0.5, self.maximum_velocity / (LOOKAHEAD_WPS-10))    # Maybe helpful for high maximum velocities
        
        for wp_offset in range(0, LOOKAHEAD_WPS):
            next_waypoint = wp_ahead + wp_offset
            # Handle circular scenarios (like the used simulator)
            if (next_waypoint >= len(self.current_waypoints.waypoints)):
                next_waypoint = next_waypoint - len(self.current_waypoints.waypoints)
            
            #lane_waypoints.waypoints.append(deepcopy(self.current_waypoints.waypoints[next_waypoint]))
            waypoint_to_append = Waypoint()
            waypoint_to_append.pose.pose.position.x = self.current_waypoints.waypoints[next_waypoint].pose.pose.position.x
            waypoint_to_append.pose.pose.position.y = self.current_waypoints.waypoints[next_waypoint].pose.pose.position.y
            waypoint_to_append.pose.pose.position.z = self.current_waypoints.waypoints[next_waypoint].pose.pose.position.z
            waypoint_to_append.pose.pose.orientation.x = self.current_waypoints.waypoints[next_waypoint].pose.pose.orientation.x
            waypoint_to_append.pose.pose.orientation.y = self.current_waypoints.waypoints[next_waypoint].pose.pose.orientation.y
            waypoint_to_append.pose.pose.orientation.z = self.current_waypoints.waypoints[next_waypoint].pose.pose.orientation.z
            waypoint_to_append.pose.pose.orientation.w = self.current_waypoints.waypoints[next_waypoint].pose.pose.orientation.w
            waypoint_to_append.twist.twist.linear.x = self.current_waypoints.waypoints[next_waypoint].twist.twist.linear.x
            waypoint_to_append.twist.twist.linear.y = self.current_waypoints.waypoints[next_waypoint].twist.twist.linear.y
            waypoint_to_append.twist.twist.linear.z = self.current_waypoints.waypoints[next_waypoint].twist.twist.linear.z
            waypoint_to_append.twist.twist.angular.x = self.current_waypoints.waypoints[next_waypoint].twist.twist.angular.x
            waypoint_to_append.twist.twist.angular.y = self.current_waypoints.waypoints[next_waypoint].twist.twist.angular.y
            waypoint_to_append.twist.twist.angular.z = self.current_waypoints.waypoints[next_waypoint].twist.twist.angular.z

            # Apply deceleration of required
            if (wp_to_stop != -1):
                dist_to_stop_point = self.distance(self.current_waypoints.waypoints, wp_ahead, wp_to_stop)
                if (dist_to_stop_point > 5.0):
                    velocity = velocity - max_dv
                    velocity = max(0.0, velocity)
                    waypoint_to_append.twist.twist.linear.x = velocity
                else:
                    waypoint_to_append.twist.twist.linear.x = 0.0
                #rospy.loginfo("Velocity at waypoint %d is %.3f", next_waypoint, waypoint_to_append.twist.twist.linear.x)
            
            lane_waypoints.waypoints.append(waypoint_to_append)
        
        self.final_waypoints_pub.publish(lane_waypoints)
    
    def get_waypoint_ahead(self):
        # Search for nearest waypoint next to our current pose
        waypoint_ahead = 0
        dist = sys.float_info.max
        for wp_index in range(0, len(self.current_waypoints.waypoints)):
            dist_to_wp = self.get_distance(self.current_waypoints.waypoints[wp_index].pose.pose.position, self.current_pose.position)
            if dist_to_wp <= dist:
                waypoint_ahead = wp_index
                dist = dist_to_wp
        
        # Check the heading between the waypoint and the pose. Maybe the nearest waypoint is the waypoint behind
        dx = self.current_pose.position.x - self.current_waypoints.waypoints[waypoint_ahead].pose.pose.position.x
        dy = self.current_pose.position.y - self.current_waypoints.waypoints[waypoint_ahead].pose.pose.position.y
        heading_wp_to_pose = math.atan2(dy, dx)
        rpy_current_pose = tf.transformations.euler_from_quaternion((self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w))
        # Check, if the signs of the heading from pose to the waypoint and the current heading at the pose have the same
        # sign or not. If they don't have the same sign, the next waypoint ahead, is the next of the nearest waypoint
        #if (math.fabs(heading_wp_to_pose + rpy_current_pose[2]) != math.fabs(heading_wp_to_pose) + math.fabs(rpy_current_pose[2])):
        if (math.fabs(rpy_current_pose[2]-heading_wp_to_pose) > math.pi/4.0):
            waypoint_ahead = waypoint_ahead + 1
            # Handle circular scenarios (like the used simulator)
            if (waypoint_ahead >= len(self.current_waypoints.waypoints)):
                waypoint_ahead = waypoint_ahead - len(self.current_waypoints.waypoints)
        return waypoint_ahead
    
    def is_waypoint_in_lookahead_waypoints(self, wp, wp_of_interest):
        if (wp_of_interest == -1):
            return False
        if (wp_of_interest >= wp and wp_of_interest <= wp + LOOKAHEAD_WPS):
            return True
        return False
    
    def apply_velocities(self, lane_waypoints, waypoint_ahead, waypoint_to_stop):
	velocity = self.current_velocity
	for wp_offset in range(0, LOOKAHEAD_WPS):
            next_waypoint = waypoint_ahead + wp_offset
            # Handle circular scenarios (like the used simulator)
            if (next_waypoint >= len(self.current_waypoints.waypoints)):
                next_waypoint = next_waypoint - len(self.current_waypoints.waypoints)
            
            if (next_waypoint >= waypoint_to_stop):          # Set all points up from the point to stop, to a velocity of zero
                lane_waypoints.waypoints[wp_offset].twist.twist.linear.x = 0.0
            else:                                          # Add a deceleration ramp
		# We want to stop 5 meters in front of the traffic-/obstacle-waypoint
                dist_to_stop_point = self.distance(self.current_waypoints.waypoints, waypoint_ahead, waypoint_to_stop) - 5.0
                decelaration_required = 0.0
		if (dist_to_stop_point > 1.0):
                    decelaration_required = self.current_velocity / (2.0*dist_to_stop_point)      # Calculates the absolute value
              	dist = self.distance(self.current_waypoints.waypoints, waypoint_ahead, next_waypoint)
		velocity = max(0.0, velocity-math.sqrt(2.0*dist*decelaration_required))
                if (velocity < 0.5):
                    velocity = 0.0
                lane_waypoints.waypoints[wp_offset].twist.twist.linear.x = velocity
            rospy.loginfo("Set the following velocity: %.3f for waypoint %d", velocity, next_waypoint)
        return lane_waypoints
    
    def get_next_stop_waypoint(self, waypoint_ahead):
        next_stop_waypoint = -1
        dist_to_next_stop_waypoint = sys.float_info.max
        if (self.latest_traffic_waypoint != None and self.latest_traffic_waypoint != -1):
            if (self.is_waypoint_in_lookahead_waypoints(waypoint_ahead, self.latest_traffic_waypoint) == True):
                next_stop_waypoint = self.latest_traffic_waypoint
                dist_to_next_stop_waypoint = self.distance(self.current_waypoints.waypoints, waypoint_ahead, next_stop_waypoint)
        if (self.latest_obstacle_waypoint != None and self.latest_obstacle_waypoint != -1):
            if (self.is_waypoint_in_lookahead_waypoints(waypoint_ahead, self.latest_obstacle_waypoint) == True):
                dist = self.distance(self.current_waypoints.waypoints, waypoint_ahead, self.latest_obstacle_waypoint)
                if (dist < dist_to_next_stop_waypoint):
                    next_stop_waypoint = self.latest_obstacle_waypoint
                    dist_to_next_stop_waypoint = dist
        return next_stop_waypoint

    def get_maximum_velocity(self):
      # Convert from km/h to m/s - because m/s is used in ROS standard unit for velocities
      max_velocity_ms = rospy.get_param('/waypoint_loader/velocity') / 3.6
      rospy.loginfo("Maximum velocity: %.3f", max_velocity_ms)
      return max_velocity_ms
    
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity
    
    def get_distance(self, wp1, wp2):
      return math.sqrt((wp1.x-wp2.x)**2 + (wp1.y-wp2.y)**2 + (wp1.z-wp2.z)**2)

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
        rate = rospy.Rate(UPDATE_RATE)
        while not rospy.is_shutdown():
          rospy.spin()
          rate.sleep()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
