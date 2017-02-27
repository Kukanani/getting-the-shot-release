#!/usr/bin/env python

# optimal control problem to maximize visible task area while taking task performer occlusion into account
# 
# Adam Allevato
# October 2016

import cvxpy
import numpy
import math
from scipy import interpolate
import visibility

waypoint_quantization = 10

t_start = 0
t_final = 10
t_step  = 0.1
K = numpy.arange(t_start, t_final, t_step)

d_t_max = 4
d_t_min = 0.5

T_waypoints = numpy.array([[5, 3], [2, 6], [7, 8]])
T_waypoints_x = T_waypoints[:,0]
T_waypoints_y = T_waypoints[:,1]

T_x = []
T_y = []
for i in range(numpy.size(T_waypoints, 0)-1):
    T_x = numpy.linspace(T_waypoints[i,0], T_waypoints[i+1,0], waypoint_quantization)
    T_y = numpy.linspace(T_waypoints[i,1], T_waypoints[i+1,1], waypoint_quantization)
print("T_x", T_x)
T = numpy.vstack([T_x, T_y])
T_start = numpy.array([[T[0,0]],[T[1,0]],[math.pi/2]], dtype=float)
print("T", T)
print("target start point", T_start)

C = cvxpy.Variable(3, len(K)+1)
C_start = numpy.array([[9],[3],[math.pi]], dtype=float)

c = numpy.zeros([2, len(K)+1])
t = numpy.zeros([2, len(K)])

O = visibility.load_world("world_1.csv")

S = numpy.zeros([2, len(K)+1])

phi = 1.5

states = []
for k in K:
    cost = visibility.visibility(T[:,k], C[:,k], O, S[k], phi)
    t[:,k] = numpy.array([T[0,k], T[1,k]])
    c[:,k] = numpy.array([C[0,k], C[1,k]])
    d_t = t - c
    constraints = [d_t > d_t_min,
                    d_t < d_t_max,
                    c[:, k+1] - c[:, k] < 0.2]
    states.append( cvxpy.Problem(cvxpy.Minimize(cost), constraints))

prob = sum(states)
prob.constraints += [C[:,0] == C_start]
prob.solve()


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


# # Problem data.
# m = 30
# n = 20
# numpy.random.seed(1)
# A = numpy.random.randn(m, n)
# b = numpy.random.randn(m)

# # Construct the problem.
# x = Variable(n)
# objective = Minimize(sum_squares(A*x - b))
# constraints = [0 <= x, x <= 1]
# prob = Problem(objective, constraints)

# # The optimal objective is returned by prob.solve().
# result = prob.solve()
# # The optimal value for x is stored in x.value.
# print(x.value)
# # The optimal Lagrange multiplier for a constraint
# # is stored in constraint.dual_value.
# print(constraints[0].dual_value)