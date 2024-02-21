#!/usr/bin/env python
import rospy
import math
from keyboard.msg import Key
from std_msgs.msg import Float32

# import custom messages
from sttr_phase2.msg import DispRef

disp_ref = DispRef()

"""
Prescribe lengths (mm) to actuators given a prescribed lever arm angle.

"""
# get refconverter setup values from launch file
# updated these values 10/26/23
act_length = 393         # mm
bicep_height = 393       # mm
bicep_pin = 2                # no. of pinholes from fulcrum (not including hidden pin holes)

tricep_height = 387.3    # mm
tricep_pin = 2            # no. of pinholes from fulcrum (not including hidden pin holes)



# create class for actuators
class Muscle:
    # use following line to auto instantiate class
    # def __init__(self):
    def __init__(self, length, height, pin):
        self.len = length   # mm # initialize variable 
        self.ht = height    # mm
        self.base = ( pin + 1 ) * 11  # length (mm) from pin to fulcrum # account for hidden pin hole with +1
        self.ref_len = 0
        self.theta_init = math.degrees(math.acos( (length**2 - height**2 - self.base ** 2 ) / (-2 * height * self.base)))


class State:
    def __init__(self):
        self.killswitch = True
        self.theta  = 0     # deg # initialize variable 
        self.start = False


# create new instance of Muscle class and assign this object to variables bicep / tricep
global bicep
global tricep
bicep = Muscle(act_length, bicep_height, bicep_pin)
tricep = Muscle(act_length, tricep_height, tricep_pin)


def key_cmd(msg):
    key = msg.code

    if key == 13:  # enter key
        state.killswitch = False

    if key == 107: # k-key for kill
        state.killswitch = True


def reset():

    state.theta = 0 
    state.start = False
    bicep.ref_len = 0
    tricep.ref_len = 0


def update_theta(msg):

    state.theta = msg.data

    if state.killswitch == True:
        reset()

    elif state.killswitch == False:

        # calculate the change in length of biceps given theta and actuator length

        theta_rad = (bicep.theta_init - state.theta ) * (math.pi/180)
        bicep.ref_len = math.sqrt((bicep.ht**2) + (bicep.base**2) - (2*bicep.ht*bicep.base*math.cos(theta_rad)))
        
        # calculate change of length of triceps given theta and actuator length
        theta_rad_tricep = math.radians(180 - 2 * (90 - bicep.theta_init)) - theta_rad
        tricep.ref_len = math.sqrt((tricep.ht**2) + (tricep.base**2) - (2*tricep.ht*tricep.base*math.cos(theta_rad_tricep)))
        
        # length to disp calculation performed in lqri node

    # update message
    disp_ref.bicep = bicep.ref_len      # float32
    disp_ref.tricep = tricep.ref_len    # float32

    # publish values
    disp_pub.publish(disp_ref)


def start():
    # start node
    rospy.init_node('refconverter')

    # initialize state
    global state
    state = State()

    # subscribe to keyboard input
    rospy.Subscriber("keyboard/keydown", Key, key_cmd)

    # subscribe to theta signals
    rospy.Subscriber('theta_ref', Float32, update_theta)

    # publish charge and kill commands
    global disp_pub
    disp_pub = rospy.Publisher('disp_ref', DispRef, queue_size = 2)

    # update callback functions
    rospy.spin()


if __name__ == "__main__":

    start()