#!/usr/bin/python

# This computer serves as a clinet
# The other computer is using robotstudio in
# Windows in order to control ABB robot
import socket
import rospy

from s_dealer.srv import GetInstruction
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import pcl

from player_recognition import face_rec
from areas_determination import det_areas


# Create Client
host = '130.215.13.7'
port = 55000
client = socket.socket()
client.connect((host, port))


# Acqurie Value
def get_loc():
    # Get image
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

    player_message = str("")
    for player in players_info:
        player_message += str(player[0])+","+str(player[1])+";"
    return player_message


def get_ins():
    rospy.wait_for_service("get_instruction")
    try:
        signal = rospy.ServiceProxy('get_instruction', GetInstruction)
        result = signal()

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    return str(result)

'''
# Test Value
def get_loc():
    return "0,0.37263497904614834;0,0.9886415550523182;1,0.40263497904614834;"
def get_ins():
    return "0"
'''

# Communicating with server
while True:
    req = client.recv(1024)
    if not req:
        continue
    if req == "0":
        player_message = get_loc()
        client.send(player_message)
        print "sent", player_message
    elif req == "1":
        instruction_message = get_ins()
        client.send(instruction_message)
        print "sent", instruction_message

client.close() 