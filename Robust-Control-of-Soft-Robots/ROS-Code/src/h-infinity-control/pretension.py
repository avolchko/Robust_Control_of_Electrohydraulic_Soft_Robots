#!/usr/bin/env python
import rospy
import math
from keyboard.msg import Key
from std_msgs.msg import Float32

# import custom messages for publishing and subscribing
from sttr_phase2.msg import CalRef
from sttr_phase2.msg import MCUStat_MIMO_SYSID

cal_ref = CalRef()

"""
Contract tricep to pretension the system after giving the biceps an initial state. 
"""

# create class for actuators
class Muscle:
    # use following line to auto instantiate class
    def __init__(self):
        self.volt = 0       
        self.volt_last = 0       
        self.volt_ss = 0
        self.init_disp = 0      # we will want to map capacitance to disp eventually anyways. 
        self.disp = 0
        
class State:
    def __init__(self):

        self.n = 0              # initialize bool for cal
        self.k = 0              # initialize counter
        self.cal = False        # boolean to track calibration status
        self.killswitch = True
        self.theta  = 0         # deg # initialize variable 
        self.theta_init = 0
        self.start = False

delta_disp = 0.1  # mm 
delta_volt = 1    # set delta voltage in triceps

def reset():
    # reset on/off bool
    state.n = 0
    # reset counter
    state.k = 0
    # reset init disp
    bicep.init_disp = 0
    bicep.disp = 0
    # reset initial voltage
    bicep.volt = 0
    # reset init volt
    tricep.volt = 0
    tricep.volt_last = 0
    # reset cal boolean
    state.cal = False

    state.theta_init = 0 
    state.start = False

def key_cmd(msg):
    key = msg.code

    if key == 13:  # enter key
        state.killswitch = False

    if key == 107: # k-key for kill
        state.killswitch = True

    # assign steady state voltage to biceps
    if key == 48: # 0-key
        bicep.volt_ss = 1500

    if key == 49: # 1-key
        bicep.volt_ss = 2000

    if key == 50: # 2-key
        bicep.volt_ss = 2500

    if key == 51: # 3-key
        bicep.volt_ss = 3000

    if key == 52: # 4-key
        bicep.volt_ss = 3500

    if key == 53: # 5-key
        bicep.volt_ss = 4000 

    if key == 54: # 6-key
        bicep.volt_ss = 4500 

    if key == 55: # 7-key
        bicep.volt_ss = 5000 

    if key == 115: # S-key
        state.start = True

    if key == 107: # K-key
        reset()

def update_theta(msg):
    # continuously update theta
    state.theta = msg.data # deg

def update_disp(msg):
    # continuously update the biceps disp mmt
    bicep.disp = msg.data # mm


# send ref volt to biceps
def update_signals(msg):

    if state.killswitch == True:
        reset()

    elif state.killswitch == False:

        bicep.volt = msg.volt1

        if (state.cal == False and state.start == True):
            # start to tension tricep until there is motion in the bicep
            
            # set initial displacement
            if (state.n == 0):
                bicep.init_disp = bicep.disp # read bicep disp info
                state.n = 1

            elif (state.n == 1):

                # check disp on biceps until > than delta_disp
                if (abs(bicep.disp - bicep.init_disp) <  delta_disp) or (bicep.disp > bicep.init_disp):
                    # ramp up the voltage from the triceps
                    tricep.volt = tricep.volt_last + int(state.k * delta_volt)
                    
                    # SAFETY 
                    if tricep.volt > 5000:
                        tricep.volt = 5000
                    else:
                        pass

                    # set last tricep volt
                    tricep.volt_last = tricep.volt

                    # increase tricep voltage
                    state.k = state.k + 0.001

                elif (abs(bicep.disp - bicep.init_disp) >=  delta_disp) and (bicep.disp < bicep.init_disp):
                    # update init theta and initial displacemnt 
                    state.theta_init = state.theta 
                    state.cal = True

            else:
                pass
            
        elif state.cal == True:
            pass

    # publishing initial theta ref info
    cal_ref.theta_0 = state.theta_init      # float32
    cal_ref.bicep_volt = bicep.volt_ss      # int16
    cal_ref.tricep_volt = tricep.volt       # int16
    cal_ref.cal_bool = state.cal            # bool


    # publish values
    cal_pub.publish(cal_ref)


def start():
    # start node
    rospy.init_node('pretension')

    # initialize state
    global state
    state = State()

    global bicep
    global tricep
    bicep = Muscle()
    tricep = Muscle()

    # subscribe to keyboard input
    rospy.Subscriber("keyboard/keydown", Key, key_cmd)

    # subscribe to pwm signals
    rospy.Subscriber('mcu_status', MCUStat_MIMO_SYSID, update_signals)

    # subscribe to theta signals
    rospy.Subscriber('theta', Float32, update_theta)

    # subscribe to theta signals
    rospy.Subscriber('disp', Float32, update_disp)

    # publish charge and kill commands
    global cal_pub
    cal_pub = rospy.Publisher('cal_ref', CalRef, queue_size = 2)

    # update callback functions
    rospy.spin()


if __name__ == "__main__":

    start()