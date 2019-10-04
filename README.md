# CS412T1C1

### Contributors
* Malcolm MacArthur
* John Chmilar

## References
We built upon and converted alex_hubers_follower.py into a state machine

## How To Run
1. Clone Repo in a directory
2. `cd CS412T1C1`
3. `catkin_make`
4. `source ./devel/setup.bash`
5. Run simulation/real robot specific launch files in new terminals
    * If you are running the program on Gazebo, run `roslaunch turtlebot_gazebo turtlebot_world.launch`
    * If you are running the program on a real robot:
        1. Put the Asus camera usb and turtlebot usb into the laptop
        2. `roslaunch turtlebot_bringup minimal.launch --screen`
        3. `roslaunch turtlebot_bringup 3dsensor.launch`
    * If you want to run rviz along with the program, run `roslaunch turtlebot_rviz_launchers view_robot.launch`
    * If you want to run smach viewer along with the program, run `rosrun smach_viewer smach_viewer.py`
6. Make sure the Asus camera is in the turtlebot's standard camera area which is at the back of the robot. Also make
sure the camera is not tilted, centered, and is perpendicular to the base of the camera
7. Put Logitech F710 usb into the laptop, and make sure the controller is on.
8. `roslaunch cop_bot start.launch` 


## How to Operate Bot

When the program has started, the bot will wait for a command from the controller.
When you press the green "A" button on the Logitech Controller, the bot will start following the closest object to its
left or center. More specifically, it will move to the closest closest object that is 60% from the left most side of the
robot's laser scanner. This means that the robot basically cannot rotate right, and it can only turn left. To make the
robot wait, press the green "A" button, on the controller, again. You can switch between following and waiting any 
number of times by pressing the green "A" button. To completely stop the robot, press ctrl-c on the laptop.

## How it Works and State Machine

There are three states, WAIT, FORWARD, DONE. The program starts at the WAIT state, In the wait state, 
it checks every second if the green "A" button has been 
pressed or not. If it is not pressed, the robot will do nothing. If it is pressed, the program switches from the WAIT 
state to the FORWARD state. In the FORWARD state it
rotates to face and moves toward the closest object that is 60% from the left most side of the
robot's laser scanner. There is no sleeping in the FORWARD state just constant updating. The FORWARD state also checks
if the green "A" button has been pressed or not. If the button has been pressed while the robot is in the FORWARD state,
the program switches from the FORWARD state to the WAIT state. It will loop like this forever, until a SIGINT signal
is received in which the program will move from either WAIT or FORWARD state to the DONE state.