#!/usr/bin/python3

# import all libraries and types
import rospy
import math
import numpy as np
import pandas as pd

from std_msgs.msg import Float32
from sttr_phase2.msg import MCURef_MIMO_SYSID
from sttr_phase2.msg import Error_MIMO
from sttr_phase2.msg import MCUStat_MIMO_SYSID
from keyboard.msg import Key
from sttr_phase2.msg import DispRef
from sttr_phase2.msg import CalRef

"""
Use error between reference disp and realtime measurements in conjuction with an H infinity dynamic controller
to provide the system with voltage inputs that will drive the actual system to the desired output.
"""

# Instantiate publishing messages
mcu_ref = MCURef_MIMO_SYSID()
error_msg = Error_MIMO()

# set frequency of control loop
freq = 200 # hz
period = 1/freq # seconds # set as the discrete time interval arg in c2d function in matlab

# scale signals
input_factor = 1/8 # rescale error signal
output_factor = 6000 # rescale output

# Set initial actuator length
actuator_length = 393 # mm

# define file location for H-inf matrices
folder = os.getcwd()

# Read matrices from csv files saved from matlab
A = pd.read_csv(folder + '/Ad.csv', header = None)
B = pd.read_csv(folder + '/Bd.csv', header = None)
C = pd.read_csv(folder + '/Cd.csv', header = None)
D = pd.read_csv(folder + '/Dd.csv', header = None)

# antiwindup limits for voltage values / control signal
max_val = 5900
min_val = 0

# get dimesnions of matrices
Bn, Bm = B.shape

class State:

    # initialize values
    def __init__(self):
        
        # initialize kill bool
        self.killswitch = True

        # initialize disp ref and error
        self.ref_disp = np.zeros(Bm)
        self.disp = np.zeros(Bm)
        self.error = np.zeros(Bm)

        self.biceps_signal = 0
        self.triceps_signal = 0

        # initiate all variables with appropriately sized matrices
        self.A_output = np.zeros(len(A))
        self.B_output = np.zeros(len(B))
        self.C_output = np.zeros(len(C))
        self.D_output = np.zeros(len(D))

        # initialize state value
        self.x = np.zeros(len(A))

        # initialize time
        self.curr_time = rospy.get_time()
        self.prev_time = self.curr_time

        self.clamp_biceps = False
        self.clamp_triceps = False

        self.calibrated = False
        self.n = 0

        # initialize precalibrated values
        self.voltage = np.zeros(2)
        self.precal_volt1 = 0
        self.precal_volt2 = 0
        self.init_voltage = np.zeros(2)
        self.init_disp = np.zeros(2)

        #   Initialize total output matrices as well
        self.output = np.zeros(2)

def set_chg(msg):
    key = msg.code

    if key == 13:  # enter key
        state.killswitch = False

    if key == 107: # k-key for kill
        state.killswitch = True

def calibrate(msg):
    state.precal_volt1 = msg.bicep_volt
    state.precal_volt2 = msg.tricep_volt
    state.calibrated = msg.cal_bool
    """when calibrated-boolean is triggered, record initial displacment and voltage values to use as equlibrium point (xeq, ueq)"""
    if (state.calibrated == True and state.n == 0):
        state.init_disp[0] = state.disp[0]
        state.init_disp[1] = state.disp[1]
        state.init_voltage[0] = msg.bicep_volt
        state.init_voltage[1] = msg.tricep_volt
        state.n = 1

    else:
        pass

def reset():
    state.ref_disp = np.zeros(Bm)
    state.disp = np.zeros(Bm)
    state.error = np.zeros(Bm)

    state.biceps_signal = 0
    state.triceps_signal = 0
    state.x = np.zeros(len(A))
    state.output = np.zeros(2)

    # initiate all variables with appropriately sized matrices
    state.A_output = np.zeros(len(A))
    state.B_output = np.zeros(len(B))
    state.C_output = np.zeros(len(C))
    state.D_output = np.zeros(len(D))

    # initialize time
    state.curr_time = rospy.get_time()
    state.prev_time = state.curr_time
    state.clamp_biceps = False
    state.clamp_triceps = False
    state.calibrated = False
    state.n = 0

    # initialize precalibrated values
    state.precal_volt1 = 0
    state.precal_volt2 = 0

    state.init_disp = np.zeros(2)
    state.init_voltage = np.zeros(2)

def delt():
    """calculate delta t with rospy time"""
    state.curr_time = rospy.get_time()        # get the current time in float seconds
    dt = state.curr_time - state.prev_time    
    state.prev_time = state.curr_time         # set current time as prev time for future calculation
    return dt

def update_ref(reference_length):
    """update reference displacement values given reference actuator lengths"""
    state.ref_disp[0] = actuator_length - reference_length.bicep 
    state.ref_disp[1] = actuator_length - reference_length.tricep 
    if state.calibrated == False:
        state.ref_disp[0] = 0
        state.ref_disp[1] = 0

def update_bicep(current):
    state.disp[0] = current.data 

def update_tricep(current):
    state.disp[1] = current.data 

def calculate_error():
    state.error = (state.ref_disp - state.disp) * input_factor
    
def update_voltage(msg):
    state.voltage[0] = msg.volt1
    state.voltage[1] = msg.volt2

def clamp():
    # antiwindup logic

    # first check if the output is saturating, if so, set logic to true
    # compare the saturated value to the controller output
    if state.output[0] >= max_val:
        biceps_sat1 = True
    elif state.output[0] <= min_val:
        biceps_sat1 = True
    else:
        biceps_sat1 = False

    # check triceps 
    if state.output[1] >= max_val:
        triceps_sat1 = True
    elif state.output[1] <= min_val:
        triceps_sat1 = True
    else:
        triceps_sat1 = False
    

    # check sign of error signal
    biceps_error_sign = np.sign(state.error[0])
    triceps_error_sign = np.sign(state.error[1])

    # check sign of biceps signal 
    biceps_sign = np.sign(state.output[0] - state.init_voltage[0]) # account for initial voltage offset
    # check sign of triceps signal 
    triceps_sign = np.sign(state.output[1] - state.init_voltage[1])

    # if error is same as biceps, output is saturated, set logic to 1
    if biceps_error_sign == biceps_sign:
        biceps_sat2 = True
    else:
        biceps_sat2 = False
        
    # if error is opposite as triceps, output is saturated, set logic to 1
    if triceps_error_sign == triceps_sign:
        triceps_sat2 = True
    else:
        triceps_sat2 = False

    # if logic 1 biceps and logic 2 biceps are true, set clamp_biceps to true
    if biceps_sat1 == True and biceps_sat2 == True:
        state.clamp_biceps = True
    else:
        state.clamp_biceps = False

    # if logic 1 triceps and logic 2 triceps are true, set clamp_triceps to true
    if triceps_sat1 == True and triceps_sat2 == True:
        state.clamp_triceps = True
    else:
        state.clamp_triceps = False

def ctrl(event):

    if state.calibrated == True:

        # calculate error
        calculate_error()

        # clamp biceps and triceps if necessary
        if state.clamp_biceps == True or state.clamp_triceps == True:
            state.error = np.zeros(Bm)
        else:
            pass

        # multiply by B
        state.A_output = np.matmul(A, state.x)
        state.B_output = np.matmul(B, state.error)
        state.x = state.A_output + state.B_output 
        state.C_output = np.matmul(C, state.x)
        state.D_output = np.matmul(D, state.error)

        # final voltage values in volts
        hinf_output = (state.C_output + state.D_output) * output_factor
        state.output = hinf_output + state.init_voltage

        clamp()

        # set output limits
        for i in range(len(state.output)):
            if state.output[i] > max_val:
                state.output[i] = max_val
            elif state.output[i] < min_val:
                state.output[i] = min_val
            else:
                pass
        
        mcu_ref.ch1_ref = int(state.output[0])
        mcu_ref.ch2_ref = int(state.output[1])

    elif state.calibrated == False:

        mcu_ref.ch1_ref = state.precal_volt1
        mcu_ref.ch2_ref = state.precal_volt2

    if state.killswitch == True:
        reset()
        mcu_ref.ch1_ref = 0
        mcu_ref.ch2_ref = 0
    else:
        pass

    # write to error message to publish 
    error_msg.bicep_error = state.error[0] / input_factor
    error_msg.tricep_error = state.error[1] / input_factor
    error_msg.bicep_ref = state.ref_disp[0]
    error_msg.tricep_ref = state.ref_disp[1]
    error_msg.cal_msg = state.calibrated
    error_msg.sat_tricep = state.clamp_triceps
    error_msg.sat_bicep = state.clamp_biceps

    # write to message to publish    

    mcu_ref.kill_ref = state.killswitch
    mcu_ref.KP = 0
    mcu_ref.KI = 0
    mcu_ref.KD = 0

    pub.publish(mcu_ref)
    err_pub.publish(error_msg)

def start():
    # start the node
    rospy.init_node('HINF_node')

    # initialize class 
    global state
    state = State() 

    # create publisher
    global pub  
    pub = rospy.Publisher('mcu_ref', MCURef_MIMO_SYSID, queue_size = 1)

    global err_pub
    err_pub = rospy.Publisher('error', Error_MIMO, queue_size = 1)

    # subscribe to keyboard input
    rospy.Subscriber("keyboard/keydown", Key, set_chg)

    # create subscriber for current voltage
    rospy.Subscriber('mcu_status', MCUStat_MIMO_SYSID, update_voltage)

    # create subscriber for reference signal
    rospy.Subscriber('disp_ref', DispRef, update_ref)

    # create subscriber for current state
    rospy.Subscriber('disp_biceps', Float32, update_bicep)
    rospy.Subscriber('disp_triceps', Float32, update_tricep)

    # create subscriber for calibration status
    rospy.Subscriber('cal_ref', CalRef, calibrate)

    # Set the desired fixed period in seconds
    ros_freq = rospy.Duration(period) 

    # Create a timer with the desired period and callback function
    rospy.Timer(ros_freq, ctrl)

    rospy.spin()

if __name__ == '__main__':
    start()  