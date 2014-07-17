#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid

saved = False

def callback(data):
    if (saved):
        return

