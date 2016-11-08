#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf import TransformListener
from copy import deepcopy

class MapWhiteout:


    def map_callback(self, map):
        print("received map!")
        self.map = map
        if self.pose is None: # uninitialized pose
            print("pose not received yet, passing on map...")
        else:
            grid = np.array(map.data)
            grid = np.resize(grid, (map.info.height, map.info.width))

            res = map.info.resolution
            origin = np.array([map.info.origin.position.x,map.info.origin.position.y])

            m_center = self.pose
            #print m_center
            m_center_adj = m_center - origin
            #print m_center_adj  
            cell_center = (m_center_adj / res)

            m_width = 0.4
            cell_width = (m_width / res)

            y_start = int(cell_center[0]-cell_width/2)
            y_stop = int(cell_center[0]+cell_width/2)

            x_start = int(cell_center[1]-cell_width/2)
            x_stop = int(cell_center[1]+cell_width/2)

            grid[x_start:x_stop, y_start:y_stop].fill(0)
            #print "grid value: " + str(any(grid))

            # print type(np.resize(grid, (1, map.info.height*map.info.width)).astype(int).tolist()[0])
            map.data = np.resize(grid, (1, map.info.height*map.info.width)).astype(np.int8).tolist()[0]

    def __init__(self):
        rospy.init_node("map_whiteout")

        self.map = None
        self.pose = None

        self.pub = rospy.Publisher("/turtlebot/map", OccupancyGrid, latch=True, queue_size=1)


        self.sub1 = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.tf = TransformListener()

        print("running map whiteout node...")
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                if self.tf.frameExists("/map") and self.tf.frameExists("/turtlebot/base_link"):
                    t = self.tf.getLatestCommonTime("/map", "/turtlebot/base_link")
                    pos, quat = self.tf.lookupTransform("/map", "/turtlebot/base_link", t)
                    print("got map to base link transform")
                    self.pose = np.array([pos[0], pos[1]])
            except:
                pass

            if self.map is not None:
                self.pub.publish(self.map)
                r.sleep()


if __name__ == '__main__':
    mw = MapWhiteout()