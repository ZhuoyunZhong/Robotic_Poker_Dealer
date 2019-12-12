# Robotic Texas Hold'em Dealer

This project is to develop a card dealing robot arm for a Texas Hold'em game. The basic rule of the game can be viewed on [Wikipedia Texas Hold'em](https://en.wikipedia.org/wiki/Texas_hold_%27em).

#### Video Demonstration

<p align="center">
  	<a href="https://www.youtube.com/watch?v=TODO">
  		<img src="demonstration/TODO.png"/>
	</a>
</p>

## Platform

Ubuntu 16.04 + ROS kinetic

XBOX 360 Kinect 1.0

ABB IRB 1600 1.45m

## Dependencies

#### Installation

**Kinect API**

`sudo apt-get install ros-kinetic-openni-camera`

`sudo apt-get install ros-kinetic-openni-launch`

**NITE and openni_tracker**

As openni_tracker does not officially support ROS kinetic, a tutorial of how to install openni_tracker and its dependencies can be found on [OpenNI Kinect Installation](https://www.reddit.com/r/ROS/comments/6qejy0/openni_kinect_installation_on_kinetic_indigo/).

**[Facial Recognition Library](https://github.com/ageitgey/face_recognition)**

Install dlib and python libraries following [Installing dlib](https://gist.github.com/ageitgey/629d75c1baac34dfa5ca2a1928a7aeaf)

Install Face recognition by

`pip3 install face_recognition`

#### Build Instructions

- Clone this repository.

- Make a build directory: catkin_make

- Run by

  launch Kinect

  `roslaunch openni_launch openni.launch depth_registration:=true`

  open Kinect skeleton tracker

  `rosrun openni_tracker openni_tracker`

  launch all other nodes

  `roslaunch s_dealer TODO.launch`

  start rviz to visualize results 

  `rosrun rviz rviz`

## Procedure Detail

#### Facial Recognition 

This project uses [Facial Recognition Library](https://github.com/ageitgey/face_recognition). It is a convenient and powerful library using [dlib](http://dlib.net/)'s state-of-the-art face recognition built with deep learning.



#### Player Area Detection



#### Instruction Recognition



#### Dealing



#### Motion Planning

