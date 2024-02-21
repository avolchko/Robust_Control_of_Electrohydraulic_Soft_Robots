#!/usr/bin/python3


import rospy
from std_msgs.msg import Int32, Bool
from sttr_phase2.msg import RecordedData
from keyboard.msg import Key

current_stage = 0
base_volt = 0
finished = False
last_msg_time = -10

def recorded_data_callback(msg):
    global last_msg_time
    last_msg_time = rospy.get_time()

def keyboard_callback(msg):
    if msg.code == 13:
        rospy.Timer(rospy.Duration(0.1), clock)

def clock(event):
    global current_stage
    global base_volt
    global finished
    global last_msg_time

    # if finished ignore rest of function
    if finished:
        return
    

    if current_stage == 0:
        base_volt = base_volt + 500

        # check if test has finished
        if base_volt == 6000:
            finished = True
            running_pub.publish(False)
        else:
            kill_pub.publish(False)
            base_volt_pub.publish(base_volt)
    elif current_stage == 1:
        record_pub.publish(True)
    elif current_stage == 2:
        running_pub.publish(True)        
    elif current_stage == 102:
        running_pub.publish(False)
        kill_pub.publish(True)
    elif current_stage == 103:
        record_pub.publish(False)
    elif current_stage == 105:
        current_stage = -1
    
    if rospy.get_time() - last_msg_time > 0.5:
        current_stage = current_stage + 1


def start():
    # start node
    rospy.init_node('sysid_node')

    # publish running command
    global running_pub
    running_pub = rospy.Publisher('running', Bool, queue_size = 2)

    # publish base voltage
    global base_volt_pub
    base_volt_pub = rospy.Publisher('base_volt', Int32, queue_size = 2)

    # publish recording command
    global record_pub
    record_pub = rospy.Publisher('recording', Bool, queue_size = 2)

    #publisher for killswitch
    global kill_pub
    kill_pub = rospy.Publisher('killswitch', Bool, queue_size = 2)

    recorded_data_sub = rospy.Subscriber("recorded_data", RecordedData, recorded_data_callback)

    keyboard_sub = rospy.Subscriber("keyboard/keydown", Key, keyboard_callback)

    # update callback functions
    rospy.spin()

if __name__ == "__main__":

    start()