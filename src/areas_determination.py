#!/usr/bin/env python

import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt

import pcl
from sensor_msgs.msg import PointCloud


# Extra points from point cloud according to indices
def extra_pc(pc, indices):
    coord = []
    for index in indices:
        coord.append(list(pc[index]))
    array = np.array(coord, dtype='float32')

    pc = pcl.PointCloud(array)
    return pc

# Pass through pc filter
def pass_through(pc, field="z", range=[0,3]):
    pass_fil = pc.make_passthrough_filter()
    pass_fil.set_filter_field_name(field)
    pass_fil.set_filter_limits(range[0], range[1])

    return pass_fil.filter()

# Down sampling point clouds
def down_sampling(pc, leaf_size=[0.03, 0.03, 0.03]):
    sor = pc.make_voxel_grid_filter()
    sor.set_leaf_size(leaf_size[0], leaf_size[1], leaf_size[2])
    
    return sor.filter()

# Point cloud plane detection
def plane_detection(pc, k=50, optimize=True, model=pcl.SACMODEL_NORMAL_PLANE,
                    method=pcl.SAC_RANSAC, dis_threshold=0.01, normal_weight=0.01, iteration=100):
    seg = pc.make_segmenter_normals(ksearch=k)
    seg.set_optimize_coefficients(optimize)
    seg.set_model_type(model)
    seg.set_method_type(method)
    seg.set_distance_threshold(dis_threshold)
    seg.set_normal_distance_weight(normal_weight)
    seg.set_max_iterations(iteration)

    indices, coefficients = seg.segment()
    if len(indices) == 0:
        print('Could not estimate a planar model for the given dataset.')
        exit(0)
    return indices, coefficients

# Outlier filter
def outlier_fil(pc, mean=50, sdv=1):
    outlier_fil = pc.make_statistical_outlier_filter()
    outlier_fil.set_mean_k(mean)
    outlier_fil.set_std_dev_mul_thresh(sdv)
    
    return outlier_fil.filter()


# Get homogeneous matrix from point o, x, y
def get_homogeneous(o, x, y):
    # Origin coordinate
    x0 = np.array([1, 0, 0])
    y0 = np.array([0, 1, 0])
    z0 = np.array([0, 0, 1])
    
    # New coordinate
    x1 = (x-o) / np.linalg.norm(x-o)
    y1 = (y-o) / np.linalg.norm(y-o)
    z1 = np.cross(x1,y1)
    
    # Transform homogeneous matrix
    homo = np.array([[np.dot(x1,x0), np.dot(y1,x0), np.dot(z1,x0), o[0]], 
                     [np.dot(x1,y0), np.dot(y1,y0), np.dot(z1,y0), o[1]], 
                     [np.dot(x1,z0), np.dot(y1,z0), np.dot(z1,z0), o[2]], 
                     [0,             0,             0,             1]])
    return homo


# Main
def det_areas(rgb_image, pc2, loc):
    '''
    Return player side information and displacement in table coordinate
    :param rbg_image: rbg kinect image
    :param pc2: point cloud 
    :param loc: players' location in image
    :return: A list consisting of player's side and player's displacement 
             in table coordinate of each player
    '''
    # Find The Table 
    # Pass through the far away points
    passed_pc = pass_through(pc2)
    # Down sampling to simplify calculation
    sored_pc = down_sampling(passed_pc)
    # Find plane
    indices, coefficients = plane_detection(sored_pc)
    # Extra plane
    plane_pc = extra_pc(sored_pc, indices)
    # Filte outlier
    table_pc = outlier_fil(plane_pc)

    # Determine Table Coordinate
    # Sort the points
    array_table = table_pc.to_array()
    z_array = array_table[array_table[:,2].argsort()] # sort according to z
    x_array = array_table[array_table[:,0].argsort()] # sort according to x
    # Use some samples to determine corners
    num_sample = 3
    top_corner = np.zeros(3)
    left_corner = np.zeros(3)
    right_corner = np.zeros(3)
    for i in range(num_sample):
        top_corner += z_array[-(i+1)]
        left_corner += x_array[i]
        right_corner += x_array[-(i+1)]
    top_corner /= num_sample
    left_corner /= num_sample
    right_corner /= num_sample
    # Calculate homogeneous matrix
    homo_matrix = get_homogeneous(top_corner, left_corner, right_corner)

    '''
    # For debugging
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(array_table[:,0], array_table[:,1], array_table[:,2], zdir='z', c='black')
    ax.scatter(top_corner[0], top_corner[1], top_corner[2], zdir='z', c='red')
    ax.scatter(left_corner[0], left_corner[1], left_corner[2], zdir='z', c='red')
    ax.scatter(right_corner[0], right_corner[1], right_corner[2], zdir='z', c='red')
    '''

    # Determine Player Location in Table Coordinate
    players_info = []
    # load each player
    for player_loc in loc:
        # Player Info Container
        # First element decides where the player is
        # Second element decides the displacement between the player and the origin
        info = []
        
        # See whether the player is sitting on the left or right
        if player_loc[3] < 470:
            info.append(0) # 0 represents left
        else:
            info.append(1) # 1 represents right


        # Get Facial Points in world coordinates
        face_index = []
        # Calculate face coordinate
        for y in range(player_loc[0], player_loc[2]+1):
            for x in range(player_loc[3], player_loc[1]+1):
                face_index.append(y*len(rgb_image[0]) + x)
        # Extra face points
        pc_face = extra_pc(pc2, face_index)

        # Pass through the far away points
        passed_face = pass_through(pc_face)
        # Down sampling to simplify calculation
        sored_face = down_sampling(passed_face)
        # Perform a outlier filter
        filtered_face = outlier_fil(sored_face)
        # Calculate player location
        face_loc = filtered_face    

        # Calculate Displacement in New Coordinate
        # Take the average coord as player location
        array_face = face_loc.to_array()
        face_camera_coord = np.average(array_face, axis=0)
        # Transform from camera coordinate to table coordinate
        face_table_coord = np.dot(homo_matrix, np.append(face_camera_coord, 1))
        # The displacement information
        if info[0]:
            displacement = face_table_coord[1]
        else:
            displacement = face_table_coord[0]
        info.append(displacement)

        # Save result
        players_info.append(info)

        '''
        # For debugging
        ax.scatter(face_camera_coord[0], face_camera_coord[1], face_camera_coord[2], zdir='z', c= 'yellow')
        ax.scatter(array_face[:,0], array_face[:,1], array_face[:,2], zdir='z', c= 'orange')

    plt.show() # For debugging '''
    return players_info


if __name__ == "__main__":
    from player_recognition import face_rec
    import os

    # load image
    rbg_image_path = os.path.join(os.path.dirname(__file__),"../data/test_rgb.jpg")
    rbg_image = mpimg.imread(rbg_image_path)
    pcl_path = os.path.join(os.path.dirname(__file__),"../data/test.pcd")
    pc2 = pcl.load(pcl_path)

    # detect areas
    loc = face_rec(rbg_image)
    infos = det_areas(rbg_image, pc2, loc)

    # show result
    for info in infos:
        print ["Player side:", info[0], "Player displacement:", info[1]]
