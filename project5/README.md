# Write up for project5 

notice: the pose estimatie is not very accurate and better to set the 2d pose estimate in the rviz file to ensure the tasks

### scripts

in the scripts files, we can run the code

(1) test the launch

```
sh ./launch.sh
```
(2) test slam and navigation

```
sh ./test_slam.sh
```

```
sh ./test_navigation.sh
```

(3) test pick_objects and add_markers

```
sh ./pick_objects.sh
```
```
sh ./add_markers.sh
```


(4) run the home service robot
```
sh ./home_service.sh
```


### Official ROS packages I refer:


https://github.com/ros-perception/slam_gmapping.git 
 
https://github.com/turtlebot/turtlebot.git
 
https://github.com/turtlebot/turtlebot_interactions.git 

https://github.com/turtlebot/turtlebot_simulator.git


### My_robot 


the original package that includes the yuwei.world for use.


### Pick_objects


the package for pick up and drop off tasks for robots


### Add_markers

add some useful markers so that we can see the target



