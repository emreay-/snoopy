##Snoopy
<p align="center">
<img src="https://github.com/emreay-/snoopy/blob/master/demos/render.png" width="300" height="300"> <img src="https://github.com/emreay-/snoopy/blob/master/demos/robot.jpg" width="300" height="300"> 
</p>
Metapackage for the custom mobile robot called "Snoopy". Snoopy was equipped with a LIDAR, an RGB-D camera, motor encoders, a uArm and a NUC. Metapackage includes particle filter localization, control, navigation, path planning, vision codes;

* **snoopy_launch:** General launch files
* **snoopy_setup**: Includes nodes for setting up the snoopy including YAML parameter files, brain, tf broadcasting and listener nodes, exploration node etc...
* **snoopy_control**: PID for motor control. 
* **snoopy_odom**: For dead reckoning
* **snoopy_teleop**: Teleoperating using keyboard
* **snoopy_vision**: Object detection, shape classification and color detection from camera.
* **snoopy_pathfind**: Path finding using A\*
* **snoopy_localize**: Particle filter localization
* **snoopy_navigate**: Action lib for navigation
* **snoopy_map**: Map server for occupancy grid map
* **snoopy_uarm**: Using uarm


## Necessary packages to build the package
ros_control pkg: "sudo apt-get install ros-indigo-ros-control ros-indigo-ros-controllers"

#Demos 
## Particle Filter Localization with *snoopy_localize*
The PF localization system was tested in the maze environment for both global localization and pose tracking. The dead reck- oning system was also kept tracked to be compared. In the experiment, 2000 particles and every 8th range measurement were used. With this setup, the mean estimation rate was a lit- tle more than 3 Hz.
* Detailed report on localization can be found [here.](https://github.com/emreay-/snoopy/blob/master/demos/Emre%20Ay%20-%20Implementing%20Particle%20Filter%20Localization%20on%20a%20Mobile%20Robot%20with%20ROS.pdf)

![Alt Text](https://github.com/emreay-/snoopy/blob/master/demos/pf_demo.gif)

## A\* Path Finding with with *snoopy_pathfind*
To navigate the robot to a desired location A\* is implemented. In the demo the code is running and giving a path from an origin to the clicked points on the map. 

![Alt Text](https://github.com/emreay-/snoopy/blob/master/demos/astar_demo.gif)

## Map Exploration
The explore service in *snoopy_setup* node was tracking the tiles in the map that were explored, and when called it was sampling a target point in the map from a uniform distribution that was lying between a minimum and maximum distance from the current position of the robot which was neither occupied nor explored. Exploration state was switching in *brain* node if there is an object detected from the vision or a collision is detected.

![Alt Text](https://github.com/emreay-/snoopy/blob/master/demos/explore_demo.gif)
