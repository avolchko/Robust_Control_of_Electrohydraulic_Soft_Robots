#!/usr/bin/python3


import rospy
import random

from keyboard.msg import Key
from std_msgs.msg import Int32, Bool

# import custom message
from sttr_phase2.msg import MCURef_MIMO_SYSID

"""
Prescribe a voltage to the HASELs for Voltage SYSID using DMDc. 
Charge profile will include a mix of step inputs to gather 
the dynamics of interest from the system. 
This input will consist of 'impulses' ranging in charge intensity
and varying between triceps and biceps actuated or not. 
We want to test this at varying 'steady state' voltages. 
"""

mcu_ref = MCURef_MIMO_SYSID()

# gather profile parameters from launch file
delta_volt_min = 100
delta_volt_max = 400
step_time_min = 0.2
delay_time_min = 0.2

# set time resolution
time_res = 0.01   # update rate # 100 Hz

# name actuators to help keep track
biceps = 0
triceps = 1

class State:
    # initialize values to be stored and updated
    def __init__(self):

        # initialize kill bool
        self.killswitch = True

        # initialize counter for clock-loop
        self.i = 0

        # initialize step up vs step down bool
        self.k = 0

        # initialize other values
        self.time = 0.0

        # set muscle to actuate | biceps = 0, triceps = 1
        self.muscle = 1

        # set steady state voltage
        self.volt_ss = 500

        self.step_time = 0      # random number generator
        self.delay_time = 0     # random number generator 
        self.delta_volt = 0     # random number generator

        # initialize running bool
        self.running = False


def set_base_volt(msg):
    state.volt_ss = msg.data

def set_running(msg):
    state.running = msg.data

def killswitch_callback(msg):
    state.killswitch = msg.data

def set_cmd(msg):
    key = msg.code

    if key == 13:  # enter key
        state.killswitch = False

    if key == 107: # k-key for kill
        state.killswitch = True



def charge(muscle, k):
    # set muscle to k and publish data
    if muscle == biceps:
        mcu_ref.ch1_ref = k
    
    elif muscle == triceps:
        mcu_ref.ch2_ref = k


def step(muscle):

    if state.k == 0:
        # charge muscle up to step voltage
        charge(muscle, state.volt_ss+state.delta_volt)
    if state.k == 1:
        # discharge muscle back to volt_ss
        charge(muscle, state.volt_ss)


# update every 1/freq Hz
def clock(event):
    if state.killswitch == False and state.running == True:

        # set ss voltage
        charge(state.muscle, state.volt_ss)

        state.time = state.i * time_res

        if state.i < state.step_time / time_res:
            state.k = 0
            step(state.muscle)
            state.i = state.i + 1

        if state.i >= state.step_time / time_res and state.i < (state.step_time + state.delay_time) / time_res:
            state.k = 1
            step(state.muscle)
            state.i = state.i + 1

        elif state.i >= (state.step_time + state.delay_time) / time_res:
            # reset counter
            state.i = 0
            # choose new random time and step values
            # select random time in seconds [min = 0.2s, max = 1.2s]
            state.step_time = random.random() + step_time_min
            state.delay_time = random.random() + delay_time_min

            # select random voltage step [min = 100V, max = 400V] or [min = -400V, max = -100V]
            if random.choice([0,1]) == 0:
                state.delta_volt = random.randint(-delta_volt_max, -delta_volt_min)
            else:
                state.delta_volt = random.randint(delta_volt_min, delta_volt_max)

    elif state.killswitch == False and state.running == False:
        if state.muscle == 0:
            mcu_ref.ch1_ref = state.volt_ss
            mcu_ref.ch2_ref = 0
        elif state.muscle == 1:
            mcu_ref.ch1_ref = 0
            mcu_ref.ch2_ref = state.volt_ss
        else:
            mcu_ref.ch1_ref = 0
            mcu_ref.ch2_ref = 0

    # if killswitch on - set all outputs and time to 0
    elif state.killswitch == True:
        state.time = 0
        state.k = 0
        state.i = 0

        # write to message to publish    
        mcu_ref.ch1_ref = 0
        mcu_ref.ch2_ref = 0


    # publishing information

    # no voltage loop PID controller
    mcu_ref.KP = 0
    mcu_ref.KI = 0
    mcu_ref.KD = 0

    mcu_ref.kill_ref = state.killswitch

    # publish values
    mcu_pub.publish(mcu_ref)


def start():
    # start node
    rospy.init_node('volt_profile')

    # initialize state
    global state
    state = State()

    # subscribe to keyboard input
    rospy.Subscriber("keyboard/keydown", Key, set_cmd)

    # subscribe to base voltage input
    rospy.Subscriber("base_volt", Int32, set_base_volt)

    rospy.Subscriber("running", Bool, set_running)

    rospy.Subscriber("killswitch", Bool, killswitch_callback)

    # publish charge and kill commands
    global mcu_pub
    mcu_pub = rospy.Publisher('mcu_ref', MCURef_MIMO_SYSID, queue_size = 2)

    # attach timer to publish callback function to publish data at specified rate
    rospy.Timer(rospy.Duration(time_res), clock)

    # update callback functions
    rospy.spin()


if __name__ == "__main__":

    start()