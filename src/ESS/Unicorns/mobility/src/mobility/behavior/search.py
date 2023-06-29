#! /usr/bin/env python
"""Searcher node."""

from __future__ import print_function

import sys
import rospy
import tf
import math
import random 

from geometry_msgs.msg import Point
from swarmie_msgs.msg import Obstacle

from mobility.planner import Planner
from mobility.swarmie import swarmie, TagException, HomeException, ObstacleException, PathException, AbortException, MoveResult

# *********************************************************************************
# Feel free to add functions, or edit this file in anyways and such to this file

# remeber to use search_exit or return with the approprate exit code
# search_exit(-1) if something is very broken
# search_exit(0) found a cube
# search_exit(1) go back home

# If you have a loop somewhere running your search it would be good to add this line
# if rospy.is_shutdown(): search_exit(-1)
# This will tell your search to stop if the robot is shutdown


# Notes
# If you wanted to turn around while ignoreing tags and the sonars it would look like this
# swarmie.turn(math.pi, ignore=Obstacle.IS_SONAR | Obstacle.VISION_SAFE)
# Please only ignore things if you absolutely need to, so like 

# If you want to keep one wheel stationary and look do a pivot around it 
#swarmie.circle()

# If you want to add some randomness like angles
# random.gauss(math.pi/2, math.pi/4)

# If you want to print out things while your code runs to help you inspect it you can do it like so
# rospy.loginfo("Stuff you want to print out")

# Drive forward and backward, keep in mind the sonars dont work when driving backward so try to avoid that if you can
# swarmie.drive(distance)

# If you want to turn so math.pi/2 would be 90 degrees
# swarmie.turn(rad)

# If you want to use the current heading and save for later
# prev_heading = swarmie.get_odom_location().get_pose().theta
# swarmie.set_heading(prev_heading) # you can also use this to turn and can add of subtract angles too

# *********************************************************************************

def search_exit(code):
    global planner, found_tag
    
    if found_tag:
        rospy.loginfo('Found a tag! Trying to get a little closer.')
        planner.face_nearest_block()
    sys.exit(code)

def main(**kwargs):
    global planner, found_tag
    found_tag = False
    planner = Planner()
    
    while True: 
 
            try:
              rospy.loginfo("Starting Search")
              swarmie.drive(3.5)
              swarmie.turn(3*math.pi/2)
              swarmie.drive(1)
              swarmie.turn(3*math.pi/2)
              swarmie.drive(3.5)
              swarmie.turn(math.pi/2)
              swarmie.drive(1)
              swarmie.turn(math.pi/2)
              swarmie.drive(3.5)
              swarmie.turn(3*math.pi/2)
              swarmie.drive(1)
              swarmie.turn(3*math.pi/2)
              swarmie.drive(3.5)
              swarmie.turn(math.pi/2)
              swarmie.drive(1)
              swarmie.turn(math.pi/2)
           

        except HomeException:
         rospy.loginfo("I saw home!")
                # @TODO what do you want to do if you run into the homeplate?
         swarmie.turn(math.pi,ignore=Obstacle.VISION_HOME)
         swarmie.drive(-0.2,ignore=Obstacle.VISION_HOME)



        except ObstacleException:
           swarmie.drive(3.5)
           rospy.loginfo("I saw an obstacle!")
           swarmie.drive(-0.2,ignore=Obstacle.IS_SONAR)
           swarmie.circle()
           swarmie.turn(math.pi/2,ignore=Obstacle.IS_SONAR)
                            


        except TagException:
        rospy.loginfo("I found a tag!")
        # Let's drive there to be helpful.
        rospy.sleep(0.3)
        if not planner.sees_home_tag(): # Make sure its not alredy in the home area
            found_tag = True
            search_exit(0)

        # Do this if you just want to start back up at home the robot will go back to the home plate
    print ("I'm homesick!") 
    search_exit(1)

if __name__ == '__main__' : 
    swarmie.start(node_name='search')
    sys.exit(main())   






