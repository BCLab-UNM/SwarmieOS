"""Put mobility related libraries here.

Any ROS module imports here will break namespace.py's ability to modify
namespaces.
"""
from __future__ import print_function

import threading
import math
from functools import wraps
#from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D


class Sync(object):
    def __init__(self, lock):
        self.lock = lock

    def __call__(self, func):
        def wrapped_f(*args, **kwargs):
            with self.lock:
                return func(*args, **kwargs)

        return wrapped_f
