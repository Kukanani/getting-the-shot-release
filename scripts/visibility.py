#!/usr/bin/env python

from shapely.geometry import Polygon, MultiPolygon, LineString
import shapely.validation
import numpy as np
from matplotlib import collections as mc
import pylab as pl
import csv
import math

import time

current_milli_time = lambda: int(round(time.time() * 1000))

def angle_to(source, dest):
    if np.array_equal(source, dest):
      return 0
    return np.arctan2(dest[1] - source[1], dest[0] - source[0])

def unit(vec):
    return vec / np.linalg.norm(vec)

def load_poly(filename):
    with open(filename, 'rb') as worldfile:
        worldreader = csv.reader(worldfile, delimiter=';')
        for row in worldreader:
            points = []
            for point in row:
                points.append((float(point.split(',')[0]), float(point.split(',')[1])))
            poly = Polygon(points)

    return poly

def load_world(filename):
    # build the world from a CSV file
    world_lines = []
    world_points = []
    world_polys = []
    with open(filename, 'rb') as worldfile:
        worldreader = csv.reader(worldfile, delimiter=';')
        for row in worldreader:
            points = []
            for point in row:
                points.append((float(point.split(',')[0]), float(point.split(',')[1])))
            
            world_points.extend(points)
            points.append(points[0])
            world_lines.extend([[points[i], points[i+1]] for i,unused in enumerate(points[:-1])])
            world_polys.append(Polygon(points[0:-1]))

    return (world_points, world_lines, world_polys)

def dedupe_adjacent(alist):
    for i in xrange(len(alist) - 1, 0, -1):
        if alist[i] - alist[i-1] < 0.000009:
            del alist[i]

def lines_from_points(points):
    return [(points[i], points[i+1]) for i in range(len(list(points))-1)]

def lines_from_poly(poly):
    poly_coords = poly.exterior.coords
    return lines_from_points(poly_coords)

def lines_from_linestring(linestring):
    linestring_coords = linestring.coords
    return line_from_points(linestring_coords)

def visibility(T, C, O, S, phi):
    start = current_milli_time()
    debug = False

    world_points = O[0]
    world_lines = O[1]
    camera_position = (C[0], C[1])
    fov = phi
    # visibility algorithm. See
    # http://www.redblobgames.com/articles/visibility/
    # http://ncase.me/sight-and-light/

    # we need to build a polygon that represents all the 2D area that the agent can see.
    # this will then be intersected with the task volume.
    # 
    S_centroid = np.array([b for b in S.centroid.coords[0]])
    primary_angle = angle_to(C, S_centroid)

    cone_min = primary_angle - phi/2
    cone_max = primary_angle + phi/2
    
    #print(camera_position, world_points[0])
    #print (angle_to(camera_position, world_points[0]))
    indexed_points = [(angle_to(camera_position, world_points[i]), world_points[i]) for i in range(len(world_points))  if (angle_to(camera_position, world_points[i]) < cone_max and angle_to(camera_position, world_points[i]) > cone_min)]
    sorted_points_indexed = sorted(indexed_points, key=lambda pts : pts[0])

    # offset in each rotational direction so we hit both to the left and right of each corner
    ray_angles_minus = [row[0]-0.00001 for row in sorted_points_indexed]
    ray_angles_center = [row[0] for row in sorted_points_indexed]
    ray_angles_plus = [row[0]+0.00001 for row in sorted_points_indexed]

    # sort angles in increasing order
    ray_angles = [None]*(len(sorted_points_indexed)*3)
    ray_angles[::3] = ray_angles_minus
    ray_angles[1::3] = ray_angles_center
    ray_angles[2::3] = ray_angles_plus

    ray_angles = sorted(ray_angles)

    dedupe_adjacent(ray_angles)

    ray_lines = []
    view_volume_points = []
    for ray_angle in ray_angles:
        # build ray vars
        r_dx = math.cos(ray_angle)
        r_dy = math.sin(ray_angle)
        r_px = camera_position[0]
        r_py = camera_position[1]

        # find the closest intersection when comparing all lines
        best_t1 = 1e308
        best_intersect = []
        intersect = False
        for line in world_lines:
            # build line segment vars
            s_px = line[0][0]
            s_py = line[0][1]
            line_dir = np.array(line[1]) - np.array(line[0])
            s_dx = line_dir[0]
            s_dy = line_dir[1]

            # check for parallel, and if so, stop
            if ((abs(s_dx - r_dx) < 0.000001 and abs(s_dy - r_dy) < 0.000001) or 
                    (abs(s_dx + r_dx) < 0.000001 and abs(s_dy + r_dy) < 0.000001)):
                #print("lines are parallel")
                continue

            # Y parametric value
            t2 = (r_dx*(s_py-r_py) + r_dy*(r_px-s_px))/(s_dx*r_dy - s_dy*r_dx)
            if t2 >= 0 and t2 <= 1:
                # X parametric value
                t1 = (s_px+s_dx*t2-r_px)/r_dx
                if t1 >= 0:
                    # check and update best (closest) intersection
                    intersect = True
                    if t1 < best_t1:
                        best_t1 = t1
                        best_intersect = [s_px + s_dx * t2, s_py + s_dy * t2]
        
        if intersect:
            view_volume_points.append(best_intersect)
            ray_lines.append([camera_position, best_intersect])

    view_volume_points.append((C[0], C[1]))

    end = current_milli_time()

    try:
        view_poly = Polygon(view_volume_points)
    except:
        print "can't create view polygon"
        print view_volume_points
    view_lines = lines_from_poly(view_poly)

    if not view_poly.is_valid:
        if(debug):
            print("view polygon is invalid")
            print(shapely.validation.explain_validity(view_poly))
            print(view_lines)
        return 0

    combo = view_poly.intersection(S)

    if(debug):
        # plot the world
        intersection_lines = []
        if isinstance(combo, Polygon):
            intersection_lines = lines_from_poly(combo)
        else:
            for combo_item in list(combo):
                if isinstance(combo_item, Polygon):
                    intersection_lines.extend(lines_from_poly(combo_item))
                elif isinstance(combo_item, LineString):
                    intersection_lines.extend(lines_from_linestring(combo_item))

        fig, ax = pl.subplots()
        ax.add_collection(mc.LineCollection(intersection_lines, colors=[(1,0.5,0.5,1)], linewidths=0.25))
        ax.add_collection(mc.LineCollection(ray_lines, colors=[(0.5,0.5,0.5,0.25)], linewidths=0.25))
        ax.add_collection(mc.LineCollection(O[1], colors=np.array([(0.5,0.5,0.5,1)]), linewidths=1))
        ax.add_collection(mc.LineCollection(view_lines, colors=np.array([(1,0,0,1)]), linewidths=2))

        ax.set_xlim([-0.1, 3.1])
        ax.set_ylim([-0.1, 3.1])
        ax.margins(0.1)
        print("time elapsed for visibility check", end - start)
        pl.show()
    
    if combo.is_valid:
       return combo.area, primary_angle
    else:
       return 0, primary_angle

def main():
    T = (3,9)
    C = (7,3)
    O = load_world("test_world.csv")
    S = (0,0)
    visibility(T, C, O, S, 1.5)

if __name__ == '__main__':
    main()