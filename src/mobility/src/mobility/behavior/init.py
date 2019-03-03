#! /usr/bin/env python

from __future__ import print_function

import sys
import math 
import rospy 

from swarmie_msgs.msg import Obstacle
from mobility.msg import MoveResult
from mobility.srv import QueueRemove, QueueRemoveRequest

from mobility.swarmie import swarmie, Location
from mobility.behavior.find_home_corner import find_home_corner

def main(**kwargs):
    remove_from_queue = rospy.ServiceProxy('start_queue/remove', QueueRemove)

    # During a normal startup the rover will be facing the center and
    # close to the nest. But there's no guarantee where we will be if 
    # mobility crashes and is forced to restart. This checks to see 
    # if we've set home location prviously.
    
    # Assume the starting position is facing the center.
    # This should be valid by contest rules. 
    #
    # Drive closer until we can see the center. 
    try:
        # Ignore cubes if they're put in the way. It's more important to continue
        # this behavior and find a corner of home than it is to stop for a cube.
        swarmie.drive(1, ignore=Obstacle.TAG_TARGET)
    except: 
        # This could happen if we bump into another rover. 
        # Let's just call it good. 
        pass

    find_home_corner()

    swarmie.turn(
        -2 * math.pi / 3,
        ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR
    )

    remove_from_queue(QueueRemoveRequest(rover_name=swarmie.rover_name,
                                         notify_others=True))

    return 0 

if __name__ == '__main__' : 
    swarmie.start(node_name='init')
    sys.exit(main())
