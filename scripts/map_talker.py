#!/usr/bin/env python

import rospy
import nav_msgs.msg
from nav_msgs.msg import OccupancyGrid
import struct

file = 'map_vector'

def talker():
    pub = rospy.Publisher('map', OccupancyGrid, queue_size = 10)
    rospy.init_node('map_talker', anonymous = True)
    r = rospy.Rate(1)
    msg = OccupancyGrid()
    while not rospy.is_shutdown():
        msg.data = [1,2]
        msg.info.width = 2048
        msg.info.height = 2048
        msg.info.resolution = 0.025
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        f = open(file, 'rb')
        i = f.read(1)
        index = 0
        while (i is not None):
            print(struct.unpack('b', i))
            index += 1
            i = f.read(1)
            print(index)
        print(index)
        talker()
    except rospy.ROSInterruptException:
        pass
