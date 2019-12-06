#!/usr/bin/env python

import face_recognition
import matplotlib.image as mpimg
import matplotlib.pyplot as plt


def face_rec(np_image):
    """
    Return number of players and their face locations (top right, bottom left)
    :param image: image
    :return: A tuple consisting of players number and a list of their face locations
    """
    face_locations = face_recognition.face_locations(np_image)
    return(len(face_locations), face_locations)


if __name__ == "__main__":
    # test image path
    image_path = "../data/test_rpg.jpg"

    # calculate number of players
    image = face_recognition.load_image_file(image_path)
    rec_result = face_rec(image)
    print "There are(is)", rec_result[0], "players."

    # show recognition result
    img = mpimg.imread(image_path)
    for location in rec_result[1]:
        print(location)
        plt.imshow(img[location[0]:location[2],location[3]:location[1],:])
        plt.show()
    
