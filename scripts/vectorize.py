#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
import shapely
from shapely import affinity, geometry
from camerabot.msg import PolygonArray
from geometry_msgs.msg import Polygon, Point

from skimage import measure
from skimage.measure import find_contours, approximate_polygon, subdivide_polygon


def map_callback(map):
    global pub
    print("map received, building contours...")
    grid = np.array(map.data)
    grid = np.resize(grid, (map.info.height, map.info.width))
    res = map.info.resolution
    contours = measure.find_contours(grid, 0.8)
    print("done making contours!")

    array = PolygonArray()

    for n, contour in enumerate(contours):
        new_s = contour.copy()
        appr_s = approximate_polygon(new_s, tolerance=3)

        poly = Polygon()
        poly.points = [Point((a[1]-map.info.height/2)*res, (a[0]-map.info.width/2)*res, 0) for a in appr_s]
        if len(poly.points) > 3:
            array.polygons.append(poly)

            # coords = [(c[1], c[0]) for c in contour]
            # #print(coords)
            # poly = shapely.geometry.Polygon(coords)

            # plt.plot(appr_s[:, 1], appr_s[:, 0],linewidth=1)
            # plt.plot(contour[:, 1], contour[:, 0], linewidth=2)

    print("publishing contours!")
    pub.publish(array)

    # plt.show()

def main():
    global pub

    rospy.init_node("map_vectorizer")
    pub = rospy.Publisher("/vector_map", PolygonArray, queue_size=1, latch=True)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.spin()

main()