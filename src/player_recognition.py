#!/usr/bin/env python

import face_recognition
import matplotlib.image as mpimg
import matplotlib.pyplot as plt


def face_rec(np_image):
    '''
    Return number of players' face locations (top right, bottom left)
    :param image: nparray image
    :return: A tuple consisting of 4 numbers indicating locations
    '''
    face_locations = face_recognition.face_locations(np_image)
    return(face_locations)


if __name__ == "__main__":
    import os
    # test image path
    image_path = os.path.join(os.path.dirname(__file__),"../data/test_rgb.jpg")

    # calculate number of players
    image = face_recognition.load_image_file(image_path)
    rec_result = face_rec(image)
    print "There are(is)", len(rec_result), "players."

    # show recognition result
    img = mpimg.imread(image_path)
    for location in rec_result:
        print(location)
        plt.imshow(img[location[0]:location[2],location[3]:location[1],:])
        plt.show()