#!/usr/bin/python3
import rospy
from keyboard.msg import Key
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

""" Transform mocap data into total contraction of actuator in mm. """

class Actuator():

    def __init__(self):
        self.top_x = 0
        self.top_y = 0
        self.top_z = 0
        self.top_w = 0

        self.bottom_x = 0
        self.bottom_y = 0
        self.bottom_z = 0
        self.bottom_w = 0

        self.initial_length = 0

        self.flag = True
        self.disp = 0

def zero(length):
    # 'tare' : set current length to initial length
    act.initial_length = length

def update(top):
    # update top of actuator position once and store value

    # # if mocap glitches, use previous position of top marker
    # if top.pose.orientation.x == 0 and top.pose.orientation.y == 0 and top.pose.orientation.z == 0:
    #     pass
        
    # else:    
    act.top_y = top.pose.position.y
    act.top_x = top.pose.position.x
    act.top_z = top.pose.position.z
    act.top_w = top.pose.orientation.w

def set_zero(key_msg):
    key = key_msg.code

    if key == 122:
        act.flag = True

def calculate(bottom):
    # calculate distance between top and bottom of actuator

    # if bottom.pose.orientation.x == 0 and bottom.pose.orientation.y == 0 and bottom.pose.orientation.z == 0:
    #     pass
        
    # else:  
    act.bottom_y = bottom.pose.position.y
    act.bottom_x = bottom.pose.position.x
    act.bottom_z = bottom.pose.position.z
    act.bottom_w = bottom.pose.orientation.w

    # calculate difference of two markers' x y z location
    x_diff = act.bottom_x - act.top_x
    y_diff = act.bottom_y - act.top_y
    z_diff = act.bottom_z - act.top_z

    # square values
    x_sq = x_diff * x_diff
    y_sq = y_diff * y_diff
    z_sq = z_diff * z_diff

    # sum 
    length_sq = x_sq + y_sq + z_sq

    # take squareroot
    length = math.sqrt(length_sq)

    # check flag to reset initial length
    if act.flag == True:
        zero(length)
        act.flag = False

    act.disp = act.initial_length - length

    # publish disp in mm
    pub.publish(act.disp*1000.0)

def start():
    # initialize node
    rospy.init_node('disp_node_biceps')

    # initialize classn to update values
    global act
    act = Actuator()

    # initialize publisher
    global pub	
    pub = rospy.Publisher('disp_biceps', Float32, queue_size = 2)

    # subscribe to pose on topic 'vrpn_client_node/act_top/pose' using callback functions
    rospy.Subscriber("mocap_node/bicep_top/pose", PoseStamped, update)
    rospy.Subscriber("mocap_node/bicep_bottom/pose", PoseStamped, calculate)
    
    # subscribe to keyboard input
    rospy.Subscriber("keyboard/keydown", Key, set_zero)

    rospy.spin()

if __name__ == '__main__':
    start()   