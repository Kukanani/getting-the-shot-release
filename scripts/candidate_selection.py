#!/usr/bin/env python

# optimal control problem to maximize visible task area while taking task
# performer occlusion into account
# 
# Adam Allevato
# October 2016

import numpy as np
import math
from matplotlib import collections as mc
import visibility
import pylab as pl
import shapely
from shapely import affinity, geometry
from matplotlib.patches import Polygon
import matplotlib
import random
import copy

from rospkg import RosPack
from geometry_msgs.msg import PoseStamped
from tf import transformations

H_w = 0.4572  # human width in m
H_h = 0.24384  # human depth in m

phi = 1.5  # camera FOV

obs_lower_limit = 0.6
soc_distance = 0.7
phs_distance = 0.35

physical_force_strength = 1
social_force_strength = 0.5

# camera point generation
desired_camera_standoff = 0.75  # in m
delta_theta = math.pi / 20  # radians

sim_step = 0.2  # seconds
sim_time = 10.0  # seconds

window_width = 4
window_height = 3

m = 5.0  # mass of robot in kg


def generate_camera_candidates(center):
    points = []
    for i in range(0,200):
        points.append(np.array([random.random() * window_width,
                                random.random() * window_height]))

    return points


def generate_camera_candidates_circle(center):

    points = []
    theta = 0
    while theta < math.pi * 2:
        points.append(np.array([float(center[0]+math.cos(theta) *
                                      desired_camera_standoff),
                                float(center[1]+math.sin(theta) *
                                      desired_camera_standoff)]))
        theta += delta_theta

    return points


def unit(vec):
    return vec/np.linalg.norm(vec)


def heading_error(initial, final):
    # clamp values
    while initial < 0:
        initial += math.pi * 2
    while initial > math.pi *2:
        initial -= math.pi * 2

    while final < 0:
        final += math.pi * 2
    while final > math.pi * 2:
        final -= math.pi * 2

    diff = final - initial
    absDiff = abs(diff)

    if absDiff <= math.pi:
        if absDiff == math.pi:
            return absDiff
        else:
            return diff

    elif final > initial:
        return absDiff - math.pi * 2
    else:
        return math.pi * 2 - absDiff


def valid(candidate, O):
    cand_point = shapely.geometry.Point(candidate)
    return (cand_point.within(O[2][0]) and not 
            any([cand_point.within(x) for x in O[2][1:]]))


def find_best_candidate(candidates, T, O, S):
    best_cost = 1e308
    best_candidate = [0, 0]
    for candidate in candidates:
        # occlusion cost
        this_cost = (S.area -
                     visibility.visibility(T, candidate, O, S, phi))/S.area
        print("occlusion cost:", this_cost)

        cnt = 0
        # obstacle cost
        for obs in O[1]:
            distance = shapely.geometry.Point(candidate).distance(shapely.geometry.LineString(obs))
            if distance < obs_lower_limit:
                cnt += 1
                # print("obs cost:", 1 - math.exp(distance)
                #  /math.exp(obs_lower_limit))
                # this_cost += (1 - distance/obs_lower_limit)
        # print("candidate obstacles:", cnt, "cost", this_cost)

        # distance cost
        this_cost += np.linalg.norm(T[0:2] - candidate) * 0.25

        if this_cost < best_cost:
            best_cost = this_cost
            best_candidate = candidate
        print("cost:", this_cost)

    print("lowest cost:",  best_cost)
    return best_candidate


def sim_candidates_sfm(candidates, T, O, S):
    moved_candidates = []
    tracks = []
    for candidate in candidates:
        track = [np.copy(candidate)]
        vel = np.array([0,0], dtype=float)
        K = int(sim_time/sim_step)
        for k in range(0,K):
            F = np.array([0.0,0.0], dtype=float)
            t = T[0:-1]

            # personal space force (social)
            # avoid divide by 0
            if np.linalg.norm(t - candidate) > 0.001:
                # direction vector between objects
                v_i = (t - candidate) / np.linalg.norm(t - candidate)

                # calculate anisotropy
                other_to_me_angle = math.atan2(candidate[1] - T[1],
                                               candidate[0] - T[0])
                theta2 = heading_error(other_to_me_angle, T[2])
                dist_between = np.linalg.norm(candidate - t)
                anis_mod = (0.6 + 0.4 * abs(math.cos(theta2)))

                # if the other turtle is facing us, apply anisotropy to make
                # it seem like we're closer.
                # this will make the repulsive force stronger!
                modified_dist = soc_distance * anis_mod
                if dist_between < modified_dist:
                    # force ramps up as we get closer and closer
                    F += m * social_force_strength * \
                         (modified_dist-dist_between) * -v_i

            # obstacle avoidance force (physical)
            for poly in O[2][1:]:
                ring = shapely.geometry.LinearRing(poly.exterior.coords)
                point = shapely.geometry.Point(candidate)
                parametric_projection = ring.project(point)
                projection = ring.interpolate(parametric_projection)
                closest_point = np.array((list(projection.coords)[0]))

                v = unit(closest_point - candidate)
                d = np.linalg.norm(closest_point - candidate)
                if d < phs_distance:
                    F += m * physical_force_strength * (phs_distance - d) * -v

            vel += (F / m) * sim_step
            vel *= 0.8
            candidate += vel * sim_step
            # print(candidate)
            track.append(np.copy(candidate))
        # print("====")
        moved_candidates.append(candidate)
        tracks.append(track)
        # print(track)
    # print(tracks[0])
    return moved_candidates, tracks


def choose_viewpoint(S, H, O, T):
    # human position
    O_world = copy.deepcopy(O)

    H = shapely.affinity.rotate(H, T[2], 'center', use_radians=True)
    points = H.exterior.coords
    O[0].extend(points)
    O[1].extend(visibility.lines_from_poly(H))
    O[2].append(H)

    S = shapely.affinity.translate(S, T[0], T[1])
    S = shapely.affinity.rotate(S, T[2], (T[0], T[1]), use_radians=True)
    S_lines = visibility.lines_from_poly(S)

    angles = np.arange(0, 2*math.pi, math.pi/100)

    candidates = generate_camera_candidates([T[0], T[1]])
    candidates = [x for x in candidates if valid(x, O)]

    candidates, tracks = sim_candidates_sfm(candidates, T, O, S)
    best_candidate = find_best_candidate(candidates, T, O, S)
    # print("best candidate:", best_candidate)

    # plotting
    maxsize = int(max(window_width, window_height))  # we want integer division)
    fig, ax = pl.subplots(figsize=(window_width * (12/maxsize), window_height
                                   * (12/maxsize)))

    ax.plot([x[0] for x in candidates], [x[1] for x in candidates], 'ko')
    ax.plot(best_candidate[0], best_candidate[1],
            'g^', markersize=20, linewidth=0)

    green = np.array([(0,0.5,0,1)])
    black = np.array([(0,0,0,1)])
    blue = np.array([(0,0,1,1)])
    red = np.array([(1,0,0,1)])

    # ax.add_collection(mc.LineCollection(visibility.lines_from_poly(H),
    # colors=red, linewidths=2))
    ax.add_collection(mc.LineCollection(S_lines, colors=red, linewidths=2))
    for track in tracks:
        ax.add_collection(mc.LineCollection(visibility.lines_from_points(track),
                                            colors=black, linewidths=1))
    # ax.add_collection(mc.LineCollection(O[1], colors=black, linewidths=2))
    obstacles = mc.PatchCollection([Polygon(x.exterior.coords, True)
                                    for x in O[2][1:]], cmap=matplotlib.cm.jet,
                                   alpha=0.4, linewidths=0)
    obstacles.set_edgecolor([0,0,0,0])
    ax.add_collection(obstacles)

    ax.set_xlim([-0.1, window_width + 0.1])
    ax.set_ylim([-0.1, window_height + 0.1])
    ax.margins(0.1)

    ax.spines["top"].set_visible(False)    
    ax.spines["bottom"].set_visible(False)    
    ax.spines["right"].set_visible(False)    
    ax.spines["left"].set_visible(False)

    pl.yticks(fontsize=14)
    pl.xticks(fontsize=14)

    pl.show()


def do_test():
    # load world
    world_filename = "walls_u.csv"
    world_filename = "walls_basic.csv"
    O = visibility.load_world(world_filename)

    # human
    # standoff = 0.381 # meters, distance human stands away from wall while
    # interacting with it
    # T = np.array([1.5,1.5-standoff, 0])

    # vaultbot
    standoff = 1.1  # meters, distance human stands away from wall while
    # interacting with it
    T = np.array([2, 2.5-standoff, -math.pi/4])

    # load target volume
    # human
    # H = shapely.geometry.Polygon([(-H_w/2,-H_h/2), (H_w/2, -H_h/2),
    # (H_w/2, H_h/2), (-H_w/2, H_h/2)])
    # vaultbot
    H = visibility.load_poly("../geom/footprint_vb.csv")
    H = shapely.affinity.translate(H, T[0], T[1])

    # load task volume
    # human
    # task_region_filename="S_detail.csv"
    # vaultbot
    # task_region_filename="../geom/taskvol_vb_left.csv"
    # S = visibility.load_poly(task_region_filename)
    tvpoints = []
    res = 10
    for i in range(0, res+1):
        tvpoints.append((-0.25 + float(0.85*math.cos(math.pi/2 + i*math.pi/res))
                         , float(0.85*math.sin(math.pi/2 + i*math.pi/res))))
    S = shapely.geometry.Polygon(tvpoints)
    choose_viewpoint(S, H, O, T)


def get_viewpoint(le_map, is_right, target_location):
    # assert le_map.header.frame_id == target_location.header.frame_id,
    # "map frame id %s does not match location frame_id %s" %
    # (map.header.frame_id, target_location.header.frame_id)
    rospack = RosPack()
    package_path = rospack.get_path("camerabot")
    folder_path = package_path + "/geom/"

    S = None
    if is_right:
        S = visibility.load_poly(folder_path + "taskvol_vb_right.csv")
    else:
        S = visibility.load_poly(folder_path + "taskvol_vb_left.csv")

    H = visibility.load_poly(folder_path + "footprint_vb.csv")

    O = [[], [], []]
    for poly_msg in le_map.polygons:
        points = [(p.x, p.y) for p in poly_msg.points]
        poly = shapely.geometry.Polygon(points)
        O[0].append(points)
        O[1].append(visibility.lines_from_poly(poly))
        O[2].append(poly)

    T = np.array([target_location.pose.position.x,
                  target_location.pose.position.y])
    vp = choose_viewpoint(S, H, O, T)

    p = PoseStamped
    p.header = le_map.header
    p.pose.position.x = vp[0]
    p.pose.position.y = vp[1]
    p.pose.position.z = 0
    ori = transformations.quaternion_from_euler(0, 0, vp[2])
    p.pose.orientation.x = ori[0]
    p.pose.orientation.y = ori[1]
    p.pose.orientation.z = ori[2]
    p.pose.orientation.w = ori[3]

    return p


if __name__ == '__main__':
    do_test()