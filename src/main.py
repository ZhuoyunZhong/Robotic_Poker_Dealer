#!/usr/bin/env python3

from player_recognition import face_rec
from areas_determination import det_areas
from s_dealer.srv import StartGameFlag, StartGameFlagResponse

import numpy as np
import rospy


# Game Procedure
def handle_game(req):
    #TODO Acquire start signal
    start_signal = False
    while not start_signal:
        start_signal = True
    print "Game Starts"
    
    # Determine number of players
    rbg_image = np.array()
    d_image = np.array()
    num, loc = face_rec(rbg_image)
    if num < 2:
        print("Sure, your biggest rival should be yourself!")
        return StartGameFlagResponse("Not enough players")

    # Determine players area
    players_info = det_areas(rbg_image, d_image, loc)


    #TODO Deal the first two facing-down cards
    # First deal

    #TODO Acquire instruction
    game_continue = True
    if not game_continue:
        return StartGameFlagResponse("Game Ends after the first deal")

    #TODO Deal the first three facing-up cards
    # Second deal

    #TODO Acquire instruction
    game_continue = True
    if not game_continue:
        return StartGameFlagResponse("Game Ends after second deal")

    #TODO Deal the forth facing-up cards
    # Third deal

    #TODO Acquire instruction
    game_continue = True
    if not game_continue:
        return StartGameFlagResponse("Game Ends after third deal")

    #TODO Deal the last facing-up cards
    # Final deal

    return StartGameFlagResponse("Game Ends after all deals")


# Connector services
def main():
    rospy.init_node('s_dealer')

    s = rospy.Service('start_game', StartGameFlag, handle_game)
    print "Ready to start"

    rospy.spin()


if __name__ == "__main__":
    main()