#!/usr/bin/python3

import rospy
from keyboard.msg import Key

# import custom message
from sttr_phase2.msg import MCURef_MIMO_SYSID

mcu_ref = MCURef_MIMO_SYSID()

kp = 0 
ki = 0 
kd = 0

# set time resolution
time_res = 0.01   # update rate # 100 Hz

class State:
    def __init__(self):
        # set killswitch to true
        self.killswitch = True
        # set step to false
        self.step = False
        # set voltage to 0
        self.volt = 0

def set_cmd(msg):
    key = msg.code

    if key == 13:  # enter key
        state.killswitch = False

    if key == 107: # k-key for kill
        state.killswitch = True

    # if key = u, step [u]p
    if key == 117:
        state.volt = state.volt + 1000

    # if key = d, step [d]own
    if key == 100:
        state.volt = state.volt - 1000

def command(event):

    if state.killswitch == True:
        state.volt = 0
    else:
        pass

    # assign values to mcu_ref msg
    
    mcu_ref.ch1_ref = state.volt
    mcu_ref.ch2_ref = 0
    mcu_ref.kill_ref = state.killswitch
    mcu_ref.KP = kp
    mcu_ref.KI = ki
    mcu_ref.KD = kd

    # publish values
    mcu_pub.publish(mcu_ref)
    

def start():
    # start node
    rospy.init_node('validate_volt_ctrl')

    # initialize state
    global state
    state = State()

    # subscribe to keyboard input
    rospy.Subscriber("keyboard/keydown", Key, set_cmd)

    # publish charge and kill commands
    global mcu_pub
    mcu_pub = rospy.Publisher('mcu_ref', MCURef_MIMO_SYSID, queue_size = 2)

    # attach timer to publish callback function to publish data at specified rate
    rospy.Timer(rospy.Duration(time_res), command)

    # update callback functions
    rospy.spin()


if __name__ == "__main__":

    start()

    
