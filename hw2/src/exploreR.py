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
        'right':  min(min(msg.ranges[0:143]),   10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
        #'fleftm': max(msg.ranges[385:512]),
        #'frightm': max(msg.ranges[129:256]),
        #'frontm':  max(msg.ranges[257:384]),
        #'rightm':  max(msg.ranges[0:128]),
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
    
    d =  2.0
    d1 = 0.5
    d2 = 2.0
    d3 = 1.0

    if state_ == 2 and regions['right'] > d:
        change_state(3)
    elif state_ == 2 and regions['fright'] > d:
        change_state(3)
    else:
        if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
            change_state(0)

        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d: # only way ahead
            change_state(0)

        elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
            change_state(0)

        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
            change_state(1)

        elif regions['front'] < d and regions['left'] > d and regions['right'] > d:
            change_state(3)

        elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
            change_state(2)

        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
            change_state(0)

        elif regions['front'] < d and regions['left'] > d and regions['right'] < d:
            change_state(1)

        elif regions['front'] < d and regions['left'] < d and regions['right'] > d:
            change_state(3)

        elif regions['front'] < d and regions['left'] < d and regions['right'] < d:
            change_state(3)

        #elif regions['front'] < d or regions['right'] < d1 and regions['fright'] < d:
            #change_state(1)

        elif regions['front'] > d3 and regions['fleft'] > d3 and regions['fright'] > d3 and regions['left'] > d3 and regions['right'] > d3:
            change_state(2)

        else:
        	change_state(2)


   

    
def find_wall():
    print('find wall')
    msg = Twist()
    msg.linear.x = 0.7
    #msg.angular.z = -0.3
    return msg 
 
def turn_left():
    global left_Count
    print('turn_left')
    msg = Twist()
    msg.angular.z = 1
    #rospy.sleep(5)
    return msg

def turn_right():
    print('turn_right')
    msg = Twist()
    msg.angular.z = -0.5
    return msg
 
def follow_the_wall():
    global regions_
    print('follow_wall')
    msg = Twist()
    msg.linear.x = 0.3
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
        #elif state_ == 4:
            #msg = adjust_left()
        #elif state_ == 5:
            #msg = adjust_right()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()
        #if state_ == 0 or state_ == 2:
         #   msg = turn_right()
          #  pub_.publish(msg)
           # rate.sleep()
 
if __name__ == '__main__':
    main()
