
# Udacity Self-Driving Car Engineer Nanodegree
## Final Project - System Integration - Term Start: 4th January
------------------------------------
![](images_readme/readme_image_top.jpg "")

## Group Members
------------------------------------
Sebastian Gangl (team lead)	- sebastian.gangl@googlemail.com

Felix Sellmann - fsellmann@gmx.de	

João Alves - jbga14@gmail.com	

Julio Aguliar - julio.c.aguilar.z@gmail.com	

Ibe Noriaki - aqrcity@gmail.com	

## System Architecture Diagram
------------------------------------
For this project, it requires writing ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following!
It will be tested with its code using a simulator, and it can be submitted the project to be run on Carla.


The following is a system architecture diagram showing the ROS nodes and topics used in the project. 
![](images_readme/readme_image_1.png "")

## Code Structure and Implementation Setails
------------------------------------

### (path_to_project_repo)/ros/src/tl_detector/
This package contains the traffic light detection node: tl_detector.py. This node takes in data from the /image_color, /current_pose, and /base_waypoints topics and publishes the locations to stop for red traffic lights to the /traffic_waypoint topic.

The /current_pose topic provides the vehicle's current position, and /base_waypoints provides a complete list of waypoints the car will be following.

It's required to build both a traffic light detection node and a traffic light classification node. Traffic light detection should take place within tl_detector.py, whereas traffic light classification should take place within ../tl_detector/light_classification_model/tl_classfier.py.
![](images_readme/readme_image_2.png "")

### (path_to_project_repo)/ros/src/waypoint_updater/
This package contains the waypoint updater node: waypoint_updater.py. The purpose of this node is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. This node will subscribe to the /base_waypoints, /current_pose, /obstacle_waypoint, and /traffic_waypoint topics, and publish a list of waypoints ahead of the car with target velocities to the /final_waypoints topic.
![](images_readme/readme_image_3.png "")


### (path_to_project_repo)/ros/src/twist_controller/
Carla is equipped with a drive-by-wire (dbw) system, meaning the throttle, brake, and steering have electronic control. This package contains the files that are responsible for control of the vehicle: the node dbw_node.py and the file twist_controller.py, along with a pid and lowpass filter that you can use in your implementation. The dbw_node subscribes to the /current_velocity topic along with the /twist_cmd topic to receive target linear and angular velocities. Additionally, this node will subscribe to /vehicle/dbw_enabled, which indicates if the car is under dbw or driver control. This node will publish throttle, brake, and steering commands to the /vehicle/throttle_cmd, /vehicle/brake_cmd, and /vehicle/steering_cmd topics.
![](images_readme/readme_image_4.png "")


## Implementation Details
------------------------------------

### waypoint updater (Sebastian Gangl)

### twist_controller (Julio Aguliar)

### dbw_node (Felix Sellmann)

### tl_detector (Ibe Noriaki)
The traffic light detection node (tl_detector.py) subscribes to four topics:

* /base_waypoints provides the complete list of waypoints for the course.

* /current_pose can be used used to determine the vehicle's location.

* /image_color which provides an image stream from the car's camera. These images are used to determine the color of upcoming traffic lights.

* /vehicle/traffic_lights provides the (x, y, z) coordinates of all traffic lights.

The node should publish the index of the waypoint for nearest upcoming red light's stop line to a single topic:

* /traffic_waypoint

The approach "Brute-force algorithm" applied into #TODO in *def get_closest_waypoint(self, pose):* to solve  The Closest pair of points problem.
        
        The Closest pair of points problem   
             (https://en.wikipedia.org/wiki/Closest_pair_of_points_problem)
        Brute-force algorithm
            
        minDist = infinity
        for i = 1 to length(P) - 1
            for j = i + 1 to length(P)
            let p = P[i], q = P[j]
                if dist(p, q) < minDist:
                    minDist = dist(p, q)
                    closestPair = (p, q)
        return closestPair 

Regarding *def process_traffic_lights(self):*, the following steps was taken.
* Get the closest light position from all List of positions that correspond to the line to stop in front of for a given intersection
* Caluclate distance from the car to the closest light
* check the light if it's in the certain distance





### tl_classifier (João Alves)



```python

```
