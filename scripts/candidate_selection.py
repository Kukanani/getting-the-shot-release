#!/usr/bin/env python

# optimal control problem to maximize visible task area while taking task
# performer occlusion into account
# 
# Adam Allevato
# October 2016

import numpy as np
import math
from matplotlib import collections as mc
from matplotlib import pyplot
import visibility
import pylab as pl
import shapely
from shapely import affinity, geometry
from matplotlib.patches import Polygon
import matplotlib
import random
import copy

import rospy
from camerabot.msg import PolygonArray
from rospkg import RosPack
from geometry_msgs.msg import PoseStamped
from tf import transformations

phi = 1.5  # camera FOV

obs_lower_limit = 0.6
soc_distance = 0.7
phs_distance = 0.35

physical_force_strength = 1
social_force_strength = 0.5

sim_step = 0.2  # seconds
sim_time = 10.0  # seconds

window_width = 10.0
window_height = 5.0

window_center_x = 0  # window_width/2
window_center_y = -2.5  # window_height/2

m = 5.0  # mass of robot in kg


def generate_camera_candidates(center):
    points = []
    for i in range(0,100):
        points.append(np.array([center[0] + random.random() * 3 - 1.5, center[1] + random.random() * 3 - 1.5]))
        # points.append(np.array([random.random() * window_width + (window_center_x - window_width/2),
        #                         random.random() * window_height + (window_center_y - window_height/2)]))

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
    return not any([cand_point.within(x) for x in O[2][1:]])


def find_best_candidate(candidates, T, O, S):
    best_cost = 1e308
    best_candidate = [0, 0, 0]
    primary_angle = 0
    for candidate in candidates:
        # occlusion cost
        cost, primary_angle = visibility.visibility(T, candidate, O, S, phi)
        this_cost = (S.area - cost)/S.area

        # distance cost
        this_cost += np.linalg.norm(np.array([b for b in S.centroid.coords[0]]) - candidate) * 0.4

        if this_cost < best_cost:
            best_cost = this_cost
            best_candidate = candidate
        best_candidate = np.append(best_candidate, primary_angle)
    return best_candidate


def sim_candidates_sfm(candidates, T, O, S):
    moved_candidates = []
    tracks = []
    for candidate in candidates:
        track = [np.copy(candidate)]
        vel = np.array([0,0], dtype=float)
        K = int(sim_time/sim_step)
        done = False
        for k in range(0,K):
            F = np.array([0.0,0.0], dtype=float)
            t = T[0:-1]

            if done:
                continue
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
                anis_mod = (0.6 + 0.4 * abs(math.sin(theta2)))

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
                #print ring
                point = shapely.geometry.Point(candidate)
                parametric_projection = ring.project(point)
                projection = ring.interpolate(parametric_projection)
                closest_point = np.array((list(projection.coords)[0]))

                v = unit(closest_point - candidate)
                d = np.linalg.norm(closest_point - candidate)
                if d < phs_distance:
                    F += m * physical_force_strength * (phs_distance - d) * -v

            if np.linalg.norm(F) < 0.001:
                done = True
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

nextfig = 0
def choose_viewpoint(S, H, O, T):
    global nextfig

    H = shapely.affinity.rotate(H, T[2], 'center', use_radians=True)
    H = shapely.affinity.translate(H, T[0], T[1])
    points = H.exterior.coords
    O[0].extend(points)
    O[1].extend(visibility.lines_from_poly(H))
    O[2].append(H)

    S = shapely.affinity.translate(S, T[0], T[1])
    S = shapely.affinity.rotate(S, T[2], (T[0], T[1]), use_radians=True)
    S_lines = visibility.lines_from_poly(S)

    angles = np.arange(0, 2*math.pi, math.pi/100)

    print("T: ", T)

    candidates = generate_camera_candidates([T[0], T[1]])

    print(str(len(candidates)) + " random candidates")
    candidates = [x for x in candidates if valid(x, O)]
    print(str(len(candidates)) + " valid candidates")

    candidates, tracks = sim_candidates_sfm(candidates, T, O, S)
    best_candidate = find_best_candidate(candidates, T, O, S)
    print("best candidate:", best_candidate)

    # plotting
    maxsize = int(max(window_width, window_height))  # we want integer division)
    fig, ax = pl.subplots(figsize=(window_width-window_center_x * (12/maxsize), window_height-window_center_y
                                   * (12/maxsize)))

    ax.plot([x[0] for x in candidates], [x[1] for x in candidates], 'ko')
    ax.plot(best_candidate[0], best_candidate[1],
            'g^', markersize=25, linewidth=0)

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

    #ax.set_xlim([-0.1 + (window_center_x-window_width/2), (window_center_x+window_width/2) + 0.1])
    #ax.set_ylim([-0.1 + (window_center_y-window_height/2), (window_center_y+window_height/2) + 0.1])
    
    ax.set_xlim([window_center_x-window_width/2-0.1, window_center_x+window_width/2+0.1])
    ax.set_ylim([window_center_y-window_height/2-0.1, window_center_y+window_height/2+0.1])
    ax.margins(0.1)

    pl.yticks(fontsize=14)
    pl.xticks(fontsize=14)


    pyplot.savefig('output' + str(nextfig) + '.png', bbox_inches='tight')
    nextfig = nextfig + 1
    pl.show()
    
    return best_candidate


def do_test():
    # load world
    world_filenames = ["walls_basic.csv", "walls_l.csv", "walls_u.csv"]
    orientations = [-math.pi/4, -math.pi/2, -3.0*math.pi/5, -5.0*math.pi/6]

    rotations = [math.pi/2, 0]
    standoffs = [0.381, 1]
    Hs = [ visibility.load_poly("../geom/footprint_human.csv"), visibility.load_poly("../geom/footprint_vb.csv")]

    tvpoints = []
    res = 10
    for q in range(0, res+1):
        tvpoints.append((-0.25 + float(0.85*math.cos(math.pi/2 + q*math.pi/res))
                         , float(0.85*math.sin(math.pi/2 + q*math.pi/res))))
    S_vb = shapely.geometry.Polygon(tvpoints)
    Ss = [visibility.load_poly("S_detail.csv"), S_vb]

    for i in range(0, len(world_filenames)):
        for j in range(0, len(orientations)):
            for k in range(0, len(standoffs)):
                O = visibility.load_world(world_filenames[i])
                standoff = standoffs[k]
                T = np.array([2, 2.5-standoff, orientations[j]+rotations[k]])
                H = copy.deepcopy(Hs[k])
                H = shapely.affinity.translate(H, T[0], T[1])
                S = Ss[k]
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
        tvpoints = []
        res = 10
        for q in range(0, res+1):
            tvpoints.append((-0.25 + float(0.85*math.cos(math.pi/2 + q*math.pi/res))
                             , float(0.85*math.sin(math.pi/2 + q*math.pi/res))))
        S = shapely.geometry.Polygon(tvpoints)
    else:
        #todo
        pass

    H = visibility.load_poly(folder_path + "footprint_vb.csv")

    O = [[], [], []]
    points = [(-5, -5), (5, -5), (5,5), (-5, 5)]
    poly = shapely.geometry.Polygon(points)
    O[0].extend(points)
    O[1].extend(visibility.lines_from_poly(poly))
    O[2].append(poly)


    for poly_msg in le_map.polygons:
        points = [(p.x, p.y) for p in poly_msg.points]
        poly = shapely.geometry.Polygon(points)
        O[0].extend(points)
        O[1].extend(visibility.lines_from_poly(poly))
        O[2].append(poly)
    # print(O)

    quat = (target_location.pose.orientation.x,
        target_location.pose.orientation.y,
        target_location.pose.orientation.z,
        target_location.pose.orientation.w)
    rot = transformations.euler_from_quaternion(quat)[2]+math.pi/2
    #print("rot", rot)

    T = np.array([target_location.pose.position.x,
                  target_location.pose.position.y,
                  rot])
    vp = choose_viewpoint(S, H, O, T)

    p = PoseStamped()
    p.header.frame_id = "map"
    p.pose.position.x = vp[0]
    p.pose.position.y = vp[1]
    p.pose.position.z = 0
    ori = transformations.quaternion_from_euler(0, 0, vp[2])
    p.pose.orientation.x = ori[0]
    p.pose.orientation.y = ori[1]
    p.pose.orientation.z = ori[2]
    p.pose.orientation.w = ori[3]

    return p

def cb_map(poly_arr):
    print("received vector map")
    vb_pose = PoseStamped()
    vb_pose.header.frame_id = "map"
    vb_pose.pose.position.x = 1.88540077209
    vb_pose.pose.position.y = -3.12148666382
    vb_pose.pose.position.z = 0.0

    vb_pose.pose.orientation.x = 0.0
    vb_pose.pose.orientation.y = 0.0
    vb_pose.pose.orientation.z = -0.0517053350395
    vb_pose.pose.orientation.w = 0.998662384557

    print("gettng viewpoint...")
    spot =get_viewpoint(poly_arr, True, vb_pose)
    print("viewpiioonnt get! " + str(spot))
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    rospy.sleep(1)
    pub.publish(spot)
    rospy.sleep(1)

if __name__ == '__main__':
    #do_test()
    
    rospy.init_node("candidate_selection")
    sub = rospy.Subscriber("/vector_map", PolygonArray, cb_map)
    rospy.spin()