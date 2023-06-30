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
    num = 4
    for num in range(5):
        try:
            swarmie.drive(5-num)
            swarmie.turn(math.pi/2)
        except HomeException:
            rospy.loginfo("I saw home!")
            swarmie.turn(math.pi/2, ignore=Obstacle.VISION_HOME)
            swarmie.drive(1, ignore=Obstacle.VISION_HOME)
            swarmie.turn(-math.pi/2, ignore=Obstacle.VISION_HOME)
            swarmie.drive(0.67, ignore=Obstacle.VISION_HOME)
            swarmie.turn(math.pi/2, ignore=Obstacle.VISION_HOME)
        except ObstacleException:
            rospy.loginfo("I saw an obstacle!")
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
