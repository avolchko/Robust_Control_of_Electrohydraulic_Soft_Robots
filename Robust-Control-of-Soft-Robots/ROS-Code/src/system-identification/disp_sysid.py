#!/usr/bin/python3

import rospy
import random
from keyboard.msg import Key

# import custom messages
from sttr_phase2.msg import CalRef
from sttr_phase2.msg import MCURef_MIMO_SYSID

"""
Prescribe a voltage profile to the HASELs for Displacement SYSID using DMDc
after the muscles are pretensioned with respect to each other. 
Charge profile will include a mix of randomized step inputs 
with random time intervals to gather the dynamics of interest from the system. 
This input will consist of 'steps' ranging in charge intensity
and varying between triceps and biceps actuated or not. 
We want to test this at varying 'steady state' voltages prescribed by the pretension node. 
"""

mcu_ref = MCURef_MIMO_SYSID()


# gather profile parameters from launch file
delta_volt_min = 200
delta_volt_max = 500
step_time_min = 0.2
delay_time_min = 0.2
# update_freq = rospy.get_param('/volt_profile_pretension/update_freq', 100)
update_freq = 100

# set time resolution
time_res = 1/update_freq        # s

# naming convention to keep track
biceps = 0
triceps = 1

class Muscle:
    def __init__(self):

        self.volt = 0
        self.volt_base = 0


class State:
    # initialize values to be stored and updated
    def __init__(self):

        # initialize kill bool
        self.killswitch = True

        # initialize counter for clock-loop
        self.i = 0

        # initialize counter for for-loop
        self.k = 0

        # initialize other values
        self.time = 0.0

        # initialize other values
        self.init_bool = 1

        # initialize cal bool
        self.cal = 0

        # initialize muscle to actuate to biceps
        self.muscle_bool = 0 

        self.step_time = 0      # random number generator
        self.delay_time = 0     # random number generator 
        self.delta_volt = 0     # random number generator

def reset():
    state.time = 0
    state.k = 0
    state.i = 0
    # write to message to publish    
    mcu_ref.ch1_ref = 0
    mcu_ref.ch2_ref = 0
    state.init_bool = 1
    state.cal = 0

def set_cmd(msg):
    key = msg.code

    if key == 13:  # enter key
        state.killswitch = False

    if key == 107: # k-key for kill
        state.killswitch = True

    # assign muscle to actuate
    if key == 98: # b-key for biceps
        state.muscle_bool = 0
    
    if key == 116: # t-key for triceps
        state.muscle_bool = 1

    # various base voltages are set in the pretension node


def tension(msg):

    state.cal = msg.cal_bool
    tricep.volt = msg.tricep_volt
    bicep.volt = msg.bicep_volt
        
    if (msg.cal_bool == 1 and state.init_bool == 1):
        # set base voltages to current voltages
        tricep.volt_base = tricep.volt
        bicep.volt_base = bicep.volt
        # set initalization bool to false now that base values are set
        state.init_bool = 0
    else:
        pass


def charge(muscle, k):
    # set muscle to k and publish data
    if muscle == biceps:
        if k < 0:
            k = 0
        elif k > 6000:
            k = 6000
        mcu_ref.ch1_ref = k
    
    elif muscle == triceps:
        if k < 0:
            k = 0
        elif k > 6000:
            k = 6000
        mcu_ref.ch2_ref = k


def step():
    # begin voltage profile after pretensioning

    # set muscle to apply volt profile
    if state.muscle_bool == 0: # biceps
        # charge biceps from base voltage 
        if state.k == 0:
            charge(biceps, bicep.volt_base+state.delta_volt)
            # return biceps to base voltage
        if state.k == 1:
            charge(biceps, bicep.volt_base)

    elif state.muscle_bool == 1: # triceps
        # charge triceps from base voltage
        if state.k == 0:
            charge(triceps, tricep.volt_base+state.delta_volt)
        # return triceps to base voltage
        if state.k == 1:
            charge(triceps, tricep.volt_base)


# update every step_time
def clock(event):
        
    if (state.cal == 1 and state.killswitch == 0):
        if state.i < state.step_time / time_res:
            state.k = 0
            step()
            state.i = state.i + 1

        elif state.i >= state.step_time / time_res and state.i < (state.step_time + state.delay_time) / time_res:
            state.k = 1
            step()
            state.i = state.i + 1

        elif state.i >= (state.step_time + state.delay_time) / time_res:
            # reset counter
            state.i = 0
            # generate randomized values
            state.step_time = 1*random.random() + step_time_min
            state.delay_time = 1*random.random() + delay_time_min
            if random.choice([0,1]) == 0:
                state.delta_volt = random.randint(-delta_volt_max, -delta_volt_min)
            else:
                state.delta_volt = random.randint(delta_volt_min, delta_volt_max)


    elif (state.cal == 0 and state.killswitch == 0):
        mcu_ref.ch1_ref = bicep.volt
        mcu_ref.ch2_ref = tricep.volt

    # if killswitch on - set all outputs and time to 0
    elif state.killswitch == 1:
        reset()
        mcu_ref.ch1_ref = 0
        mcu_ref.ch2_ref = 0

    # no voltage loop controller
    # we can delete these in the message if we wish to clean it up
    mcu_ref.KP = 0
    mcu_ref.KI = 0
    mcu_ref.KD = 0

    mcu_ref.kill_ref = state.killswitch

    # publish values
    mcu_pub.publish(mcu_ref)


def start():
    # start node
    rospy.init_node('disp_sysid_node')

    # initialize state amd muscle classes
    global state
    state = State()

    global bicep, tricep
    bicep = Muscle()
    tricep = Muscle()

    # subscribe to keyboard input
    rospy.Subscriber("keyboard/keydown", Key, set_cmd)

    # subscribe to base voltages for biceps and triceps and add on or subtract from there
    rospy.Subscriber("cal_ref", CalRef, tension)

    # publish charge and kill commands
    global mcu_pub
    mcu_pub = rospy.Publisher('mcu_ref', MCURef_MIMO_SYSID, queue_size = 2)

    # attach timer to publish callback function to publish data at specified rate
    rospy.Timer(rospy.Duration(time_res), clock)

    # update callback functions
    rospy.spin()


if __name__ == "__main__":

    start()
