#!/usr/bin/env python

from s_dealer.srv import StartGameFlag, StartGameFlagResponse, GetInstruction
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2

from player_recognition import face_rec
from areas_determination import det_areas
import pcl
import numpy as np
import time
import rospy


def get_signal():
    rospy.wait_for_service("get_instruction")
    try:
        signal = rospy.ServiceProxy('get_instruction', GetInstruction)
        result = signal()

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    return result


# Game Procedure
def handle_game(req):
    print "Waiting for instruction"
    start_signal = 0
    while start_signal != 2:
        start_signal = get_signal().result
    print "Game Starts"
    
    # Determine number of players
    # Get rgb image from kinect
    bridge = CvBridge()
    image_message = rospy.wait_for_message("/camera/rgb/image_color", Image)
    rbg_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")

    # Get pointclouds from kinect
    data =rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
    pc = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
    pc_list = []
    for point in pc:
        pc_list.append( [point[0],point[1],point[2]] )

    p = pcl.PointCloud()
    p.from_list(pc_list)

    # Perform Facial Recognition
    print "Start Facial Recognition"
    loc = face_rec(rbg_image)
    '''
    if len(loc) < 2:
        print "There is only", len(loc), "player."
        return StartGameFlagResponse("Not enough players")'''

    # Determine players area
    print "Start Players Area Detection"
    players_info = det_areas(rbg_image, p, loc)
    print(players_info)

    # Deal the first two facing-down cards
    print "First deal"
    time.sleep(2)
    print "Waiting for instruction"

    # Acquire instruction
    game_continue = 0
    while game_continue == 0:
        game_continue = get_signal().result
    if game_continue == 1:
        return StartGameFlagResponse("Game Ends after the first deal")

    # Deal the first three facing-up cards
    print "Second deal"
    time.sleep(2)
    print "Waiting for instruction"

    # Acquire instruction
    game_continue = 0
    while game_continue == 0:
        game_continue = get_signal().result
    if game_continue == 1:
        return StartGameFlagResponse("Game Ends after second deal")

    # Deal the forth facing-up cards
    print "Third deal"
    time.sleep(2)
    print "Waiting for instruction"

    # Acquire instruction
    game_continue = 0
    while game_continue == 0:
        game_continue = get_signal().result
    if game_continue == 1:
        return StartGameFlagResponse("Game Ends after third deal")

    # Deal the last facing-up cards
    print "Final deal"

    return StartGameFlagResponse("Game Ends after all deals")


# Connector services
def main():
    rospy.init_node('s_dealer')
    s = rospy.Service('start_game', StartGameFlag, handle_game)
    print "Ready to start"

    rospy.spin()


if __name__ == "__main__":
    main()