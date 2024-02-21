#!/usr/bin/env python
import rospy
from keyboard.msg import Key
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from sttr_phase2.msg import CalRef

"""
Prescribe an angle (deg) given rise time, fall time, half time and max orientation.
Use parameters from launch file to define profile. Otherwise use default values. 
Kill with 'k', run with 'enter' key.

"""

# gather profile parameters from launch file
peak_theta  = rospy.get_param('/profile/peak', 1)
half_time = rospy.get_param('/profile/half_time', 1)
rise_time = rospy.get_param('/profile/rise_time', 1)
fall_time = rospy.get_param('/profile/fall_time', 1)

# initialize time
time = 0

# define a time increment to update profile
time_res = 0.001 # time step to update position profile (period in seconds)

class State:
    # initialize values to be stored and updated
    def __init__(self):

        # initialize kill bool
        self.killswitch = True

        # initialize counter for for loop
        self.i = 0

        # initialize start bool
        self.start = False

def set_start(msg):
    # set start bool to true when calibration is complete
    state.start = msg.cal_bool

def set_position(msg):
    # determine position at each time given position profile parameters

    key = msg.code

    if key == 13:
        state.killswitch = False

    elif key == 107: # k-key for kill
        state.killswitch = True

    if (state.killswitch == False and state.start == True):
        time = state.i * time_res

        if time <= rise_time:
            theta_ref = peak_theta * (time/rise_time)
        elif time > rise_time and time <= half_time:
            theta_ref = peak_theta
        elif time > half_time and time <= half_time + fall_time:
            theta_ref = peak_theta - peak_theta * (time - half_time) / fall_time
        elif time > half_time + fall_time and time <= 2 * half_time:
            theta_ref = 0
        elif time > 2*half_time:
            theta_ref = 0
            state.i = 0

        state.i = state.i + 1

    else:
        time = 0
        theta_ref = 0 
        state.i = 0
        

    # publish theta and kill bool
    theta_pub.publish(theta_ref)
    kill_pub.publish(state.killswitch) 

def start():
    # start node
    rospy.init_node('profile_generator')

    # initialize state
    global state
    state = State()

    # subscribe to keyboard input
    rospy.Subscriber("keyboard/keydown", Key, set_position)

    # subscribe to calibration bool
    rospy.Subscriber("cal_ref", CalRef, set_start)

    # publish voltage command
    global theta_pub
    theta_pub = rospy.Publisher('theta_ref', Float32, queue_size=2)

    # publish kill command
    global kill_pub
    kill_pub = rospy.Publisher('kill_ref', Bool, queue_size=2)


if __name__ == '__main__':

    start()
    rate = rospy.Rate(1/time_res)

    while not rospy.is_shutdown():
        rate.sleep()
        set_position(Key)
