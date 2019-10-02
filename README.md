# CS412T1C1

### Contributors
* Malcolm MacArthur
* John Chmilar

## References
We built upon and converted alex_hubers_follower.py into a state machine

### How To Run
1. Clone Repo in a directory
2. `cd CS412T1C1`
3. `catkin_make`
4. `source ./devel/setup.bash`
5. Run simulation/real robot specific launch files in new terminals
    * If you are running the program on Gazebo, run `roslaunch turtlebot_gazebo turtlebot_world.launch`
    * If you are running the program on a real robot, run:
        1.`roslaunch turtlebot_bringup minimal.launch --screen`
        2.`roslaunch turtlebot_bringup 3dsensor.launch`
    * If you want to run rviz along with the program, run `roslaunch turtlebot_rviz_launchers view_robot.launch`
    * If you want to run smach viewer along with the program, run `rosrun smach_viewer smach_viewer.py`
6. `roslaunch cop_bot start.launch` Note: A controller(Logitech F710 for example) MUST be connected to the computer