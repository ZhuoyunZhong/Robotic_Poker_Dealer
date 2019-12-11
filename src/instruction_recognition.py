#!/usr/bin/env python

import rospy
from tf2_msgs.msg import TFMessage
from s_dealer.srv import GetInstruction, GetInstructionResponse


def get_instruction(head, right_hand, left_hand, right_elbow, left_elbow):
    # Doing a Cross above the head
    if left_hand[2] > head[2] and left_hand[1] < left_elbow[1] and\
       right_hand[2] > head[2] and right_hand[1] > right_elbow[1]:
        print "Game Stops"
        return 1
    
    # Stretch Right Hand above the head
    if left_hand[2] > head[2] and left_hand[1] > left_elbow[1]:
        print "Game Continues"
        return 2

    # print ("No Instruction")
    return 0


def handle_tf(msg):
    # Get locations of head, right_hand, left_hand, 
    #                  right_elbow and left_elbow
    for tfmsg in msg.transforms:
        ids = tfmsg.child_frame_id
        head_loc = ids.find("head")
        right_hand_loc = ids.find("right_hand")
        left_hand_loc = ids.find("left_hand")
        right_elbow_loc = ids.find("right_elbow")
        left_elbow_loc = ids.find("left_elbow")

        if head_loc != -1: # Head
            head[0] = tfmsg.transform.translation.x
            head[1] = tfmsg.transform.translation.y
            head[2] = tfmsg.transform.translation.z
        elif right_hand_loc != -1: # Right_hand
            right_hand[0] = tfmsg.transform.translation.x
            right_hand[1] = tfmsg.transform.translation.y
            right_hand[2] = tfmsg.transform.translation.z
        elif left_hand_loc != -1: # Left_hand
            left_hand[0] = tfmsg.transform.translation.x
            left_hand[1] = tfmsg.transform.translation.y
            left_hand[2] = tfmsg.transform.translation.z
        elif right_elbow_loc != -1: # Right_elbow
            right_elbow[0] = tfmsg.transform.translation.x
            right_elbow[1] = tfmsg.transform.translation.y
            right_elbow[2] = tfmsg.transform.translation.z
        elif left_elbow_loc != -1: # Left_elbow
            left_elbow[0] = tfmsg.transform.translation.x
            left_elbow[1] = tfmsg.transform.translation.y
            left_elbow[2] = tfmsg.transform.translation.z

    
def handle_get(req):
    res = get_instruction(head, right_hand, left_hand, right_elbow, left_elbow)
    return GetInstructionResponse(res)


def instruction_recognition():
    rospy.init_node("test_instruction")
    sub = rospy.Subscriber("/tf", TFMessage, handle_tf)

    s = rospy.Service("get_instruction", GetInstruction, handle_get)

    rospy.spin()


if __name__ == "__main__":
    # Start Kinect and skeleton tracker before testing
    global head, right_hand, left_hand, right_elbow, left_elbow
    head = [0, 0, 0]
    right_hand = [0, 0, 0]
    left_hand = [0, 0, 0]
    right_elbow = [0, 0, 0]
    left_elbow = [0, 0, 0]
    
    instruction_recognition()