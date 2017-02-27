#!/usr/bin/env python

# optimal control problem to maximize visible task area while taking task performer occlusion into account
# 
# Adam Allevato
# October 2016

import cvxpy
import numpy
import math
from scipy import interpolate
from matplotlib import collections as mc
import visibility
import pylab as pl
import shapely
from shapely import affinity, geometry


def generate_camera_candidates(center):
    delta_theta = 0.1 # radians
    distance = 1 # meters

    points = []
    theta = 0
    while theta < math.pi * 2:
        points.append([float(center[0]+math.cos(theta)*distance), float(center[1]+math.sin(theta)*distance)])
        theta += delta_theta
    print points

    return points

t_start = 0
t_final = 20
t_step  = 1.0
K = numpy.arange(t_start, t_final, t_step)

d_t_max = 4
d_t_min = 0.5
H_w = 0.4572  # human width in m
H_h = 0.24384 # human depth in m
standoff = 0.381 # meters, distance human stands away from wall while interacting with it
phi = 1.5 # camera FOV

desired_camera_standoff = 1 # in m

T_waypoints = numpy.array([[1.5,2.5-standoff], [1.5,2.5-standoff]])
T_waypoints_x = T_waypoints[:,0]
T_waypoints_y = T_waypoints[:,1]

T_x = []
T_y = []
waypoint_quantization = len(K)/(len(T_waypoints)-1)
for i in range(numpy.size(T_waypoints, 0)-1):
    T_x = numpy.append(T_x, numpy.linspace(T_waypoints[i,0], T_waypoints[i+1,0], waypoint_quantization))
    T_y = numpy.append(T_y, numpy.linspace(T_waypoints[i,1], T_waypoints[i+1,1], waypoint_quantization))
T = numpy.vstack([T_x, T_y])
T_start = numpy.array([[T[0,0]],[T[1,0]],[math.pi/2]], dtype=float)

candidates = generate_camera_candidates(T_start[0:2])

C = numpy.zeros([2, len(K)+1])
# initial camera position
C_start = numpy.array([0.51, 1.5], dtype=float)
C[0,0] = C_start[0]
C[1,0] = C_start[1]

c = numpy.zeros([2, len(K)+1])
t = numpy.zeros([2, len(K)])

O = visibility.load_world("wall_with_person.csv")

S = [None] * len(K)
S_w = 1.0
S_h = 0.5
for i,pt in enumerate(T.T):
    S_offset = shapely.geometry.Polygon([(-S_w/2.0,0), (S_w/2.0, 0), (S_w/2.0, S_h), (-S_w/2.0, S_h)])
    S_offset = shapely.affinity.translate(S_offset, pt[0], pt[1])
    S[i] = S_offset

H = shapely.geometry.Polygon([(-H_w/2,-H_h/2), (H_w/2, -H_h/2), (H_w/2, H_h/2), (-H_w/2, H_h/2)])
H = shapely.affinity.translate(H, pt[0], pt[1])
points = H.exterior.coords
O[0].extend(points)
O[1].extend(visibility.lines_from_poly(H))

v = 0.1 # max velocity
directions = numpy.array([[0, 0], [-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]], dtype=float)
S_lines = []
angles = numpy.arange(0, 2*math.pi, math.pi/100)
for k in range(0,len(K)):
    best_visibility = 0
    best_next_point = C[:,k]
    for direction in directions:
        offset = [x * v * t_step for x in direction]
        this_vis = visibility.visibility(T[:,k], C[:,k] + offset, O, S[k], phi)
        if this_vis > best_visibility:
            best_visibility = this_vis
            best_next_point = C[:,k] + offset
    C[:,k+1] = best_next_point
    print("visibility score:",  best_visibility)

    S_lines.extend(visibility.lines_from_poly(S[k]))

fig, ax = pl.subplots()

ax.plot(C[0, :], C[1, :], 'b')
ax.plot(T[0, :], T[1, :], 'r')

green = numpy.array([(0,0.5,0,1)])
black = numpy.array([(0,0,0,1)])
blue = numpy.array([(0,0,1,1)])

lines_between = [[C[:, i], T[:, i]] for i in range(0, numpy.size(T,1))]
lc_between = mc.LineCollection(lines_between, colors=green, linewidths=1)
ax.add_collection(lc_between)

lc_H = mc.LineCollection(visibility.lines_from_poly(H), colors=blue, linewidths=2)
ax.add_collection(lc_H)

lc_S = mc.LineCollection(S_lines, colors=green, linewidths=1)
ax.add_collection(lc_S)

lc_walls = mc.LineCollection(O[1], colors=black, linewidths=2)
ax.add_collection(lc_walls)

ax.set_xlim([-0.1, 3.1])
ax.set_ylim([-0.1, 3.1])
ax.margins(0.1)
pl.show()

# from cvxpy import *
# x = Variable(n, T+1)
# u = Variable(m, T)

# states = []
# for t in range(T):
#     cost = sum_squares(x[:,t+1]) + sum_squares(u[:,t])
#     constr = [x[:,t+1] == A*x[:,t] + B*u[:,t],
#               norm(u[:,t], 'inf') <= 1]
#     states.append( Problem(Minimize(cost), constr) )
# # sums problem objectives and concatenates constraints.
# prob = sum(states)
# prob.constraints += [x[:,T] == 0, x[:,0] == x_0]
# prob.solve()