#!/usr/bin/env python

import signal
import rospy
import smach
import smach_ros
import math
from math import tanh
from geometry_msgs.msg import Twist
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy

global button_start
global shutdown_requested


class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start', 'done'])

    def execute(self, userdata):
        global button_start
        global shutdown_requested
        while not shutdown_requested:
            if button_start:
                return 'start'
            rospy.sleep(1)
        return 'done'


class Forward(smach.State):

    def __init__(self, follow_distance=2, stop_distance=1, max_speed=0.6, min_speed=0.01):
        smach.State.__init__(self, outcomes=['wait', 'done'])

        # Subscribe to the laser data
        self.sub = rospy.Subscriber('scan', LaserScan, self.laser_callback)

        # Publish movement commands to the turtlebot's base
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist)

        # How close should we get to things, and what are our speeds?
        self.stopDistance = stop_distance
        self.max_speed = max_speed
        self.min_speed = min_speed
        # at what distance do we start following something/someone?
        self.followDist = follow_distance

        # the distance to the closest object, and its position in array, respectively.
        self.closest = 0
        self.position = 0
        # Create a Twist message, and fill in the fields.  We're also
        # going to fill in the linear.x field, even though we think
        # we're going to set it later, just to be sure we know its
        # value.
        self.command = Twist()
        self.command.linear.x = 0.0
        self.command.linear.y = 0.0
        self.command.linear.z = 0.0
        self.command.angular.x = 0.0
        self.command.angular.y = 0.0
        self.command.angular.z = 0.0
        self.hit = 0
        return

    def execute(self, userdata):
        global button_start
        global shutdown_requested
        while not shutdown_requested:
            if not button_start:
                button_start = False
                return 'wait'
        return 'done'

    def laser_callback(self, scan):
        global button_start
        # determines the closest thing to the Robot.
        self.get_position(scan)
        rospy.logdebug('position: {0}'.format(self.position))
        # if there's something within self.followDist from us, start following.

        if self.closest < self.followDist and button_start:
            self.follow()
        # else just don't run at all.
        else:
            self.stop()
        # Add a log message, so that we know what's going on
        rospy.logdebug('Distance: {0}, speed: {1}, angular: {2}'.format(self.closest, self.command.linear.x, self.command.angular.z))
        # Ensure we have only one publish command.
        self.pub.publish(self.command)

    def controller_callback(self, event):
        print(event)

    # Starts following the nearest object.
    def follow(self):
        self.command.linear.x = tanh(5 * (self.closest - self.stopDistance)) * self.max_speed
        # turn faster the further we're turned from our intended object.
        self.command.angular.z = ((self.position-320.0)/320.0)*2

        # if we're going slower than our min_speed, just stop.
        if abs(self.command.linear.x) < self.min_speed:
            self.command.linear.x = 0.0

    def stop(self):
        self.command.linear.x = 0.0
        self.command.angular.z = 0.0
        # function to occupy self.closest and self.position

    def get_position(self, scan):
        # Use 60% of the left side scan data
        scan_range = scan.ranges[int(math.floor(len(scan.ranges)*0.4)):]
        # scan_range = scan.ranges
        # Build a depths array to rid ourselves of any nan data inherent in scan.ranges.
        depths = []
        for dist in scan_range:
            if not np.isnan(dist):
                depths.append(dist)
                # scan.ranges is a tuple, and we want an array.
        full_depths_array = scan.ranges[:]

            # If depths is empty that means we're way too close to an object to get a reading.
            # thus establish our distance/position to nearest object as "0".
        if len(depths) == 0:
            self.closest = self.stopDistance
            self.position = 320
        else:
            self.closest = min(depths)
            self.position = full_depths_array.index(self.closest)


def controller_callback(event):
    global button_start
    if event.buttons[1] == 1:
        button_start = not button_start


def request_shutdown(sig, frame):
    global shutdown_requested
    shutdown_requested = True


def main():
    global button_start
    global shutdown_requested
    button_start = False
    shutdown_requested = False

    rospy.init_node('cop_bot')
    controller_sub = rospy.Subscriber('joy', Joy, controller_callback)

    # Create done outcome which will stop the state machine
    sm_turtle = smach.StateMachine(outcomes=['DONE'])

    with sm_turtle:
        smach.StateMachine.add('WAIT', Wait(),
                               transitions={'start': 'FORWARD', 'done': 'DONE'})
        smach.StateMachine.add('FORWARD', Forward(follow_distance=3),
                               transitions={'wait': 'WAIT', 'done': 'DONE'})

    # Create and start the instrospection server - needed for smach_viewer
    sis = smach_ros.IntrospectionServer('TRAVELLER_server', sm_turtle, 'FOLLOWER')
    sis.start()

    # Start state machine and run until SIGINT received
    signal.signal(signal.SIGINT, request_shutdown)
    sm_turtle.execute()

    # Stop server
    sis.stop()


if __name__ == '__main__':
    main()
