#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

import math

yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180) # 5 degrees
position_ = Point()
initial_position_ = Point()
initial_position_.x = rospy.get_param('initial_x')
initial_position_.y = rospy.get_param('initial_y')
initial_position_.z = 0
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
regions_ = None

# callbacks
def clbk_odom(msg):
    global position_, yaw_
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def clbk_laser(msg):
    global regions_
    regions_ = {
        'front':  min(min(msg.ranges[0:22] + msg.ranges[337:359]), 10),
        'fleft':  min(min(msg.ranges[22:67]), 10),
        'left':   min(min(msg.ranges[67:112]), 10),
        'bleft':   min(min(msg.ranges[112:157]), 10),
        'back':   min(min(msg.ranges[157:202]), 10),
        'bright':  min(min(msg.ranges[202:247]), 10),
        'right':  min(min(msg.ranges[247:292]), 10),
        'fright': min(min(msg.ranges[292:337]), 10)
    }

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def Pythagoras(x1,x2,y1,y2):
    return math.sqrt(((x1 - x2)**2 + (y1 - y2)**2))

def follow_wall(regions):
    d = 0.4
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    if regions['front'] < d or regions['fright'] < d or regions['fleft'] or regions['left']:
        twist_msg.angular.z = 0.5
        return twist_msg
    elif regions['bleft'] < d or regions['back'] < d or regions['bright']:
        twist_msg.angular.z = -0.5
        return twist_msg
    elif regions['right'] < d:
        twist_msg.linear.x = 0
        twist_msg.linear.x = 0.3
        return twist_msg
    else:
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        return twist_msg

def go_to_point(goal, position, err_yaw):
    goal_precision = 0.2
    twist_msg = Twist()
    dist = Pythagoras(goal.x,position.x,goal.y,position_.y)
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    if math.fabs(err_yaw) > yaw_error_allowed_:
        twist_msg.angular.z = 0.4 if err_yaw > 0 else -0.4
        # rospy.loginfo("Turning")
    elif dist > goal_precision:
        twist_msg.linear.x = 0.3
        twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
        # rospy.loginfo("Going straight")
    return twist_msg

def main():
    
    rospy.init_node('bug0')
    
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    rospy.wait_for_service('/gazebo/set_model_state')
    
    srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    # set robot initial position
    model_state = ModelState()
    model_state.model_name = 'turtlebot3_burger'
    model_state.pose.position.x = initial_position_.x
    model_state.pose.position.y = initial_position_.y
    resp = srv_client_set_model_state(model_state)
    
    rate = rospy.Rate(20)
    min_dist_from_wall = 0.4

    while not rospy.is_shutdown():
        if regions_ == None:
            continue

        desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
        err_yaw = normalize_angle(desired_yaw - yaw_)

        if regions_['front'] > min_dist_from_wall and regions_['right'] > min_dist_from_wall and regions_['fright'] > min_dist_from_wall:
            cmd_vel = go_to_point(desired_position_, position_, err_yaw)
        else:
            cmd_vel = follow_wall(regions_)

        pub_cmd_vel.publish(cmd_vel)
            
        rate.sleep()

if __name__ == "__main__":
    main()