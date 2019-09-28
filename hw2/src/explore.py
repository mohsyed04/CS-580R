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
    4: 'adjust_left',
    5: 'adjust_right',
}

left_Count = 0
count = 0
    
def clbk_laser(msg):
    global regions_
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
    
    d = 1.3
    d1 = 0.5
    d2 = 2.0
    
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d: #find wall
        change_state(0)

    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d: 
        change_state(0)

    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d: #go straight
        change_state(2)

    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d: #find wall
        change_state(0)

    elif regions['front'] > d and regions['right'] < d and regions['fright'] < d: #adjust left
    	change_state(4)

    elif regions['front'] > d and regions['left'] < d and regions['fleft'] < d: #adjust right
    	change_state(5)

    elif regions['right'] > d2 and regions['fright'] > d1 and regions['fright'] < d2 and regions['front'] < d or regions['front'] > d or regions['fleft'] < d or regions['fleft'] > d: #turn right
    	change_state(3)
    	
    elif regions['front'] < d:
    	change_state(1)
               
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
    msg.angular.z = 0.3
    left_Count += 1
    #rospy.sleep(5)
    return msg

def turn_right():
    print('turn_right')
    msg = Twist()
    #msg.linear.x = 0.1
    msg.angular.z = -0.5
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

def adjust_left():
    print('adjust_left')
    msg = Twist()
    msg.angular.z = 0.1
    return msg

def adjust_right():
    print('adjust_right')
    msg = Twist()
    msg.angular.z = -0.1
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
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 3:
            msg = turn_right()
        elif state_ == 4:
            msg = adjust_left()
        elif state_ == 5:
            msg = adjust_right()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()
 
if __name__ == '__main__':
    main()
