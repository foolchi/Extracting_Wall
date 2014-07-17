#!/usr/bin/env python

import rospy
import nav_msgs.msg
from nav_msgs.msg import OccupancyGrid
import struct

file = 'data/map_vector'
data = []

def talker():
    pub = rospy.Publisher('map', OccupancyGrid, queue_size = 10)
    rospy.init_node('map_talker', anonymous = True)
    r = rospy.Rate(1)
    msg = OccupancyGrid()
    while not rospy.is_shutdown():
        msg.data = data
        msg.info.width = 2048
        msg.info.height = 2048
        msg.info.resolution = 0.025
        msg.info.origin.position.x = -25.6125
        msg.info.origin.position.y = -25.6125
        msg.info.origin.position.z = 0
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        f = open(file, 'rb')
        ib = f.read(1)
        index100 = 0
        index0 = 0
        index1 = 0
        while (ib is not None and ib != ""):
            i = (struct.unpack('b', ib))[0]
            data.append(i)
            if i == -1:
                index1 += 1
            elif i == 100:
                index100 += 1
            elif i == 0:
                index0 += 1
            ib = f.read(1)
            #print(index)
        print("index0: %d index1: %d index100: %d" %(index0, index1, index100))
        print(index0 + index1 + index100)
        talker()
    except rospy.ROSInterruptException:
        pass
