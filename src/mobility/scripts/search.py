#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy 
import math
import random 

from std_msgs.msg import String

from obstacle_detection.msg import Obstacle 
from mobility.msg import MoveResult

from mobility.swarmie import Swarmie 

'''Searcher node.''' 

def main():
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)
    
    while not rospy.is_shutdown() : 
        rospy.loginfo("Wandering...")
        
        # Drive to a random location.   
        rval = swarmie.drive(2, random.gauss(0, math.pi/8))
        
        # Failed because we saw an obstacle with sonar.
        if rval == MoveResult.OBSTACLE_SONAR :
            rospy.loginfo("I see a wall!")
            rval = swarmie.drive(0, random.gauss(math.pi, math.pi/8), Obstacle.IS_SONAR)

        # Failed because we saw a target tag.        
        elif rval == MoveResult.OBSTACLE_VISION : 
            rospy.loginfo("I see a tag!")
        
        else:
            rospy.loginfo("Circling...")
            swarmie.circle()
            
    return 0

if __name__ == '__main__' : 
    main()

