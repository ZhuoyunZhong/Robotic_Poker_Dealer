#!/usr/bin/env python

import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt

import pcl
from sensor_msgs.msg import PointCloud


def det_areas(rgb_image, pc2, loc):
    '''
    Return player side information and displacement in table coordinate
    :param rbg_image: rbg kinect image
    :param pc2: point cloud 
    :param loc: players' location in image
    :return: A list consisting of player's side and player's displacement 
             in table coordinate of each player
    '''
    # Find the table 
    #TODO

    # Determine table coordinate
    #TODO


    players_info = []
    # load each player
    for player_loc in loc:
        # Player info container
        # First element decides where the player is
        # Second element decides the displacement between the player and the origin
        info = []
        
        # See whether the player is sitting on the left or right
        # 0 represents left, 1 represents right
        if player_loc[3] < 470:
            info.append(0)
        else:
            info.append(1)

        # Get facial points coordinates(x, y, z)
        # with respect to the table coordinates
        # in order to calculate face displacement between the player 
        # and the origin in world coordinates
        face_index = []
        # Calculate face coordinate
        for y in range(player_loc[0], player_loc[2]+1):
            for x in range(player_loc[3], player_loc[1]+1):
                face_index.append(y*len(rgb_image[0]) + x)
        
        # Extract face points
        coord_face = []
        for index in face_index:
            coord_face.append(list(pc2[index]))
        array_face = np.array(coord_face, dtype='float32')

        # Transform from numpy array to point cloud
        pcl_face = pcl.PointCloud(array_face)
        # Pass through the far away points
        pass_fil = pcl_face.make_passthrough_filter()
        pass_fil.set_filter_field_name("z")
        pass_fil.set_filter_limits(0, 3)
        passed = pass_fil.filter()
        # Perform a outlier filter
        fil = passed.make_statistical_outlier_filter()
        fil.set_mean_k(50)
        fil.set_std_dev_mul_thresh(1.0)
        filtered = fil.filter()
        # Transform back from point cloud to numpy array
        array_face = filtered.to_array()

        

        # Transform from camera coordinate to table coordinate
        #TODO

        ''' # For debugging
        from mpl_toolkits.mplot3d import Axes3D
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(array_face[:,0], array_face[:,1], array_face[:,2], zdir='z', c= 'orange')
        plt.show()
        ''' 

        # Calculate displacement in new coordinate
        #TODO
        displacement = 0
        info.append(displacement)

        # Save result
        print(info)
        players_info.append(info)

    return players_info


if __name__ == "__main__":
    from player_recognition import face_rec

    # load image
    rbg_image_path = "../data/test_rgb.jpg"
    rbg_image = mpimg.imread(rbg_image_path)
    pc2 = pcl.load("../data/test.pcd")

    # detect areas
    loc = face_rec(rbg_image)
    infos = det_areas(rbg_image, pc2, loc)

    # show result
    for info in infos:
        print ["Player side:", info[0], "Player displacement:", info[1]]