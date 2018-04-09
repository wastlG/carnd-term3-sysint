
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

## Rubric Points
------------------------------------
* Smoothly follows waypoints in the simulator.
* Respects the target top speed set for the waypoints' twist.twist.linear.x in waypoint_loader.py. Works by testing with different values for kph velocity parameter in /ros/src/waypoint_loader/launch/waypoint_loader.launch. Vehicle adheres to the kph target top speed set here.
* Stops at traffic lights when needed.
* Stops and restarts PID controllers depending on the state of /vehicle/dbw_enabled.
* Publishes throttle, steering, and brake commands at 50hz.
* Launches correctly using the launch files provided in the capstone repo.

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


## Some implementation Details
------------------------------------

### waypoint updater
First of all, it implemented the callbacks for the both subscribers for the base waypoints and the current pose of the vehicle. These callbacks only safes the input in member variables of the class and calls a function to update the waypoints. At first, the update function determines the next waypoint ahead of the vehicle. This is the start point for the next published waypoints. After that, the next 100 waypoints will be appended up from this waypoint. The velocities are set to the maximum allowed velocity.

In the second step, it implemented two furhter subscribers for the traffic and obstacle waypoints and implemented their callbacks. The callbacks safes the new information in class member variables. As like before, the callbacks
functions also calls the update function. There, It added a check function, if a traffic or obstacle waypoint is in front of the car and in range of the next 100 waypoints. If a traffic or obstacle waypoint is in front, It applied a deceleration ramp to smoothly stop the vehicle.

### Twist controller package files (twist controller & DBW Node)
### twist controller
It's the Controller class,which can be use to implement vehicle control. The control method can take twist data as input and return throttle, brake, and steering values. Within this class,  it imports and uses the provided pid.py and lowpass.py for acceleration, and yaw_controller.py for steering.

### DBW Node
Regarding dbw_node publishers and subscribers, it add ROS subscribers for the /current_velocity, /twist_cmd, and /vehicle/dbw_enabled topics. and it imports the Controller class from twist_controller.py which will be used for implementing the necessary controllers. The function used to publish throttle, brake, and steering is publish.

Note that throttle values passed to publish should be in the range 0 to 1, although a throttle of 1 means the vehicle throttle will be fully engaged. Brake values passed to publish should be in units of torque (N*m). The correct values for brake can be computed using the desired acceleration, weight of the vehicle, and wheel radius.

### Traffic Light Detection package files ( tl detector and 
### tl detector
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

### tl classifier
It contains the TLClassifier class, which can be used to implement traffic light classification.
The get_classification method can take a camera image as input and return an ID corresponding to the color state of the traffic light in the image and it's implemented as follows go get image and convert to tensor to classifier.

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        img= cv2.resize(image,dsize=(224,224), interpolation = cv2.INTER_CUBIC)
        img=cv2.normalize(img.astype('float'), None, -0.5, .5, cv2.NORM_MINMAX)

        #Convert to tensor
        np_image_data = np.asarray(img)
        reshaped_img = np.expand_dims(np_image_data,axis=0)

        results = self.sess.run(self.output_operation.outputs[0],
                      {self.input_operation.outputs[0]: reshaped_img})
        results = np.squeeze(results)
        
        
        top_k = results.argsort()[-5:][::-1]
        predict_label = self.labels[top_k[0]]
        print(predict_label)

        return self.labels_dic[predict_label]
        
###### This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here]https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images



```python

```
