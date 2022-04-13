### Ros interface :
Allow to interface with ros message. It contains publisher interfaces, reading and transform ros messages for both modules Surface and Footstep planner. Memmo_planner_ros should be installed to run ros experiments, on feature-walkgen branch : https://github.com/thomascbrs/memmo_planner_ros/tree/feature-walkgen

#### Dependencies
numpy
- rospy : ```pip3 install rospy```
- pinocchio : ```sudo apt install -qqy robotpkg-py38-pinocchio```
- visualization_msgs : ```sudo apt-get install ros-noetic-visualization-msgs```
- geometry_msgs ```sudo apt-get install ros-noetic-geometry-msgs```
- footstep_msgs (Memmo_planner_ros)
- std_msgs
