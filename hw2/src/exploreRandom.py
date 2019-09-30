#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import random

regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'rMove',
    1: 'tracing wall',
    2: 'right',
    3: 'left'
}

distance = 1

def clbk_laser(msg): #gotten from online change later
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }

    take_action()

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state

def take_action():
    global regions_
    global distance
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    if regions['front'] > distance and regions['fleft'] > distance and regions['fright'] > distance: #no block
        change_state(0)

    elif regions['front'] > distance and regions['right'] > 0.5 and regions['fright'] < 0.5: #prevent collision
        change_state(3)

    elif regions['front'] > distance and regions['fleft'] > distance and regions['fright'] < distance: #right block
        change_state(1)

    elif regions['front'] > distance and regions['fleft'] < distance and regions['fright'] > distance: #left block
        change_state(2)

    elif regions['front'] < distance and regions['fleft'] > distance and regions['fright'] > distance: #front block
        change_state(2)

    elif regions['front'] > distance and regions['fleft'] < distance and regions['fright'] < distance: #left-right block
        change_state(1)

    elif regions['front'] < distance and regions['fleft'] < distance and regions['fright'] > distance: #left-front block
        change_state(2)

    elif regions['front'] < distance and regions['fleft'] > distance and regions['fright'] < distance: #right-front block
        change_state(2)

    elif regions['front'] < distance and regions['fleft'] < distance and regions['fright'] < distance: #all block
        change_state(2)

    else:
        pass

def rMovement():
    global distance

    #get random values for angular velocity
    angle = random.random()

    #randomly turn left or right
    #direc = random.randrange(1)
    angle = 0.5

    #move the robot
    msg = Twist()
    msg.linear.x = 0.7
    msg.angular.z = angle
    return msg

def tracingWall():
    global distance
    global regions_
    msg = Twist()
    msg.linear.x = 0.5
    return msg


def turningRight():
    msg = Twist()
    msg.angular.z = -0.25
    return msg

def turningLeft():
    msg = Twist()
    msg.angular.z = 0.5
    return msg

def main():
    global pub_

    rospy.init_node('reading_laser')

    pub_ = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = rMovement()
        elif state_ == 1:
            msg = tracingWall()
        elif state_ == 2:
            msg = turningRight()
        elif state_ == 3:
            msg = turningLeft();
        else:
            rospy.logerr('Unknown state!')

        pub_.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    main()
