##If you have Differential Drive Robot
1. To run this package, clone this folder in your workspace and run `catkin build` in your terminal. 
2. Change the topic to which the node subscribes to in the source file. 

##If you don't have Differential Drive Robot
1. If you don't have access to a differential drive robot model simulation - clone this [repository](https://github.com/karanchawla/CallistoRover)
and build it using `catkin build`.
2. In terminal 1, run `roslaunch callisto_gazebo callisto_course.launch`
3. In terminal 2, run `rosrun callisto_linefollower linefollow.py`
4. In terminal 3, run `rosrun rqt_graph rqt_graph` to visualize the interation between the nodes.
