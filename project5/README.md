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
<p align="center">
   <img src="images/Screenshot from 2020-05-25 13-14-22.png" width="70%" height="70%">

</p>


<p align="center">
   <img src="images/Screenshot from 2020-05-25 13-27-03.png" width="70%" height="70%">

</p>

(3) test pick_objects and add_markers

```
sh ./pick_objects.sh
```
```
sh ./add_markers.sh
```

<p align="center">
   <img src="images/Screenshot from 2020-05-25 14-56-00.png" width="70%" height="70%">

</p>

(4) run the home service robot
```
sh ./home_service.sh
```


<p align="center">
   <img src="images/Screenshot from 2020-05-25 14-56-00.png" width="70%" height="70%">

</p>


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



