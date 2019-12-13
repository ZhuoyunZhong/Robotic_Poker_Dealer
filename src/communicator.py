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


# Acqurie Value
def get_loc():
    # Get image
    bridge = CvBridge()
    image_message = rospy.wait_for_message("/camera/rgb/image_color", Image)
    rbg_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
    print "Got image"

    # Get pointclouds from kinect
    data =rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
    pc = pc2.read_points(data, skip_nans=False, field_names=("x", "y", "z"))
    pc_list = []
    for point in pc:
        pc_list.append( [point[0],point[1],point[2]] )

    p = pcl.PointCloud()
    p.from_list(pc_list)
    print "Got point cloud"

    # Perform Facial Recognition
    loc = face_rec(rbg_image)

    # Determine players area
    players_info = det_areas(rbg_image, p, loc)
    print(players_info)

    player_message = str("")
    for player in players_info:
        player_message += str(player[0])+","+str(player[1])+","
    return player_message


def get_ins():
    rospy.wait_for_service("get_instruction")
    try:
        signal = rospy.ServiceProxy('get_instruction', GetInstruction)
        instruction = signal().result

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    if instruction != 0:
        print(instruction)

    return str(instruction)+","


def communicator():
    rospy.init_node("communicator")

    # Create Client
    host = '192.168.100.100'
    port = 55000
    client = socket.socket()
    client.connect((host, port))

    '''
    # Test Value
    def get_loc():
        return "1,402,2,372,2,988,"
    def get_ins():
        return "2,"
    '''

    while not rospy.is_shutdown():
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
            if instruction_message != 0:
                print "sent", instruction_message
    
    client.close()
    rospy.spin()


if __name__ == "__main__":
    communicator()
    
