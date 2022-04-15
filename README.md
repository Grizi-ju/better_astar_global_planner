# better_astar_global_planner
The better A* global planner plugin in ROS

Tutorials : https://www.bilibili.com/video/BV1U44y137dc?spm_id_from=333.999.0.0

**Ubuntu 18.04**

**ros melodic**

### Install:

``````
cd demo_ws/src
git clone git@github.com:Grizi-ju/better_astar_global_planner.git
cd ..
catkin_make
source devel/setup.bash
``````

### How to use:

1.place the simulation environment in the same src directory：

![图片](https://user-images.githubusercontent.com/80267952/163544079-5c147049-a466-4329-8c16-b1273238b28f.png)


2.open the file move_base.launch and add the new planner as parameters of the global planner, as follows: 

`````
<node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen" clear_params="true">
  <param name="base_global_planner" value="BAstar_planner/BAstarPlannerROS"/>
``````

```BAstar_planner/BAstarPlannerROS``` is the name of plugin

3.run
``````
roslaunch mbot_gazebo view_mbot_with_laser_gazebo.launch
roslaunch mbot_navigation exploring_slam_demo.launch
``````

### Test:
![图片](https://user-images.githubusercontent.com/80267952/163546481-d1822200-4694-4c8a-8cd8-fa486612fbb7.png)

**Reference:** 

http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS

https://www.youtube.com/watch?v=We1gGDXAO_o

https://www.youtube.com/watch?v=t4A_niNlDdg

https://github.com/coins-lab/relaxed_astar

