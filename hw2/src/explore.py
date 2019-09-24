#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry 
import math

regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
    3: 'turn right',
    4: 'reverse',
}

left_Count = 0
count = 0
    
def clbk_laser(msg): # the ranges array is the laser scan results returned by the /scan topic, it consists of 640 points spread 
    global regions_  # over 180 degrees. The regions are divided almost equally into 5 parts. . 
    regions_ = {
        'right':  min(min(msg.ranges[0:128]),   10), 
        'fright': min(min(msg.ranges[129:256]), 10),
        'front':  min(min(msg.ranges[257:384]), 10),
        'fleft':  min(min(msg.ranges[385:512]), 10),
        'left':   min(min(msg.ranges[513:639]), 10),
    }
 
    take_action()
 
 
def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state


def take_action():
    global regions_
    global left_Count, count
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    d = 1.5
    d1 = 0.3
    
    if regions['front'] > d and regions['fright'] > d and regions['fleft'] > d and regions['right'] > d and regions['left'] > d: #find wall
        print('1 fw')
        change_state(0)

    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d and regions['right'] > d: #go straight
        change_state(2)

    if regions['front'] < d and regions['fleft'] > d and regions['fright'] > d: #turn left
        change_state(1)

    if regions['front'] > d and regions['fleft'] > d and regions['fright'] < d: #go straight
        change_state(2)

    if regions['front'] > d and regions['fleft'] < d and regions['fright'] > d: #find wall #changed from 0 to 1 turn left
        print('2nd fw')
        change_state(2)

    if regions['front'] < d and regions['fleft'] > d and regions['fright'] < d: #turn left 
        state_description = 'case 5 - front and fright'
        change_state(1)

    if regions['front'] < d and regions['fleft'] < d and regions['fright'] > d: #turn left
        change_state(1)

    if regions['front'] < d and regions['fleft'] < d and regions['fright'] < d: #turn left
        change_state(1)

    if regions['front'] > d and regions['fleft'] < d and regions['fright'] < d: #find wall
        change_state(0)

    if regions['right'] > 3 and regions['front'] > d and regions['fleft'] > d and regions['left'] > d : #turn right
        change_state(3)

    if regions['right'] > 3 and regions['front'] < d and regions['fleft'] < d and regions['left'] < d : #turn right '''regions['fright'] > d1 and '''
        change_state(3)

    if regions['front'] < d1 and regions['fright'] < d1 and regions['fleft'] < d1 : #too close to wall
        change_state(4)

    if regions['front'] < d1 or regions['fright'] < d1 or regions['fleft'] < d1 or regions['right'] < d1 or regions['left'] < d1 : #too close to wall
        change_state(4)

    if regions['front'] > 2 and regions['fright'] < d and regions['fleft'] > d and regions['left'] > d and regions['right'] < d: #go straight
        change_state(2)

    if regions['front'] > 2 and regions['fright'] <= 2 and regions['fleft'] > d and regions['left'] > d and regions['right'] < d: #go straight
        change_state(2)

    else:
        pass        
        #state_description = 'unknown case'
        #rospy.loginfo(regions)


    
def find_wall():
    global count
    print('find wall')
    msg = Twist()
    msg.linear.x = 0.7
    msg.angular.z = 0.2
    count = 1 
    #rospy.sleep(5)
    return msg 
 
def turn_left():
    global left_Count
    print('turn_left')
    msg = Twist()
    msg.angular.z = 0.5
    left_Count += 1
    #rospy.sleep(5)
    return msg

def turn_right():
    print('turn_right')
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.4
    left_Count = 0
    #ospy.sleep(5)
    return msg
 
def follow_the_wall():
    global regions_
    print('turn_left')
    msg = Twist()
    msg.linear.x = 0.6
    #rospy.sleep(5)
    return msg
def reverse():
    print('reverse')
    msg = Twist()
    msg.linear.x = - 0.6
    ms.angular.z = 0.3


    
def main():
    global pub_
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 3:
            msg = turn_right()
        elif state_ == 4:
            msg = reverse()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()
 
if __name__ == '__main__':
    main()
