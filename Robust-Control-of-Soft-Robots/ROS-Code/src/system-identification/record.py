#!/usr/bin/python3
import rospy
from keyboard.msg import Key
from std_msgs.msg import Bool

class State:
    def __init__(self):

        self.record = False

def key_cmd(msg):
    key = msg.code

    if key == 114:  # r-key for record
        state.record = not state.record

        record_msg = state.record

        pub.publish(record_msg)

    else:
        pass

def start():
    # start node
    rospy.init_node('record_node')

    # initialize state
    global state
    state = State()

    # subscribe to keyboard input
    rospy.Subscriber("keyboard/keydown", Key, key_cmd)

    # publish charge and kill commands
    global pub
    pub = rospy.Publisher('recording', Bool, queue_size = 2)

    # update callback functions
    rospy.spin()


if __name__ == "__main__":

    start()