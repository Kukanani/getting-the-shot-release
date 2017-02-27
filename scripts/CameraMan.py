#!/usr/bin/env python
"""This program is designed to wait for the a base link to stop moving and then
 provide its location, left or right arm, and the map to the TurtleBot so it
 can function as a cameraman."""
__author__ = "Andrew Sharp"
__maintainer__ = "Andrew Sharp"
__email__ = "asharp@utexas.edu"
__credits__ = "Andrew Sharp"
__license__ = "BSD"
__copyright__ = """Copyright The University of Texas at Austin, 2014-20XX.
                All rights reserved. This software and documentation
                constitute an unpublished work and contain valuable trade
                secrets and proprietary information belonging to the
                University. None of the foregoing material may be copied or
                duplicated or disclosed without the express, written
                permission of the University. THE UNIVERSITY EXPRESSLY
                DISCLAIMS ANY AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND
                DOCUMENTATION, INCLUDING ANY WARRANTIES OF MERCHANTABILITY
                AND/OR FITNESS FOR A PARTICULAR PURPOSE, AND WARRANTIES OF
                PERFORMANCE, AND ANY WARRANTY THAT MIGHT OTHERWISE ARISE FROM
                COURSE OF DEALING OR USAGE OF TRADE. NO WARRANTY IS EITHER
                EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF THE SOFTWARE OR
                DOCUMENTATION. Under no circumstances shall the University be
                liable for incidental, special, indirect, direct or
                consequential damages or loss of profits, interruption of
                business, or related expenses which may arise from use of
                software or documentation, including but not limited to those
                resulting from defects in software and/or documentation, or
                loss or inaccuracy of data of any kind."""
__project__ = "Support Utilities"
__version__ = "1.0.2"
__status__ = "Development"
__description__ = """This program is designed to wait for the a base link to
                  stop moving and then provide its location, left or right arm,
                  and the map to the TurtleBot so it can function as a
                  cameraman."""


from sys import path
from math import sqrt
from copy import deepcopy
from rospy import init_node, is_shutdown, signal_shutdown, Time, get_param, \
sleep, Duration, Subscriber
from rospkg import RosPack
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import TransformListener, Exception as TfExcept
from camerabot.msg import PolygonArray

from MessagesExits import MessagesExits as Me
ROSPACK = RosPack()
PACKAGE_PATH = ROSPACK.get_path("camerabot")
FOLDER_PATH = PACKAGE_PATH + "/scripts/"
PATH = path.append(FOLDER_PATH)
from candidate_selection import get_viewpoint


class CameraMan:
    """This class is designed to provide a wrapper for Adam's cameraman."""
    def __init__(self, params):
        """Initialize transform listener, subscribers, and action servers."""
        self.params = params
        self.tfl = TransformListener()
        self.map_sub = Subscriber("/vector_map", PolygonArray, self.map_callback)
        self.turtlebot_ac = SimpleActionClient("turtlebot/move_base",
                                               MoveBaseAction)
        # self.turtlebot_ac.wait_for_server()
        self.map = PolygonArray()
        return

    def loop(self):
        """Running loop to determine if the base link is moving."""
        old_pose = PoseStamped()
        old_pose.header.frame_id = self.params['base_frame']
        old_pose.pose.orientation.w = 1.0
        new_pose = PoseStamped()
        new_pose.header.frame_id = self.params['base_frame']
        new_pose.pose.orientation.w = 1.0
        num_stationary = 0
        total_time = self.params['loop_rate'] * self.params['num_stationary']
        nav_goal = MoveBaseGoal()
        Me.info_message("Starting loop")
        while not is_shutdown():
            try:
                self.tfl.waitForTransform(self.params['world_frame'],
                                          self.params['base_frame'],
                                          Time(0), Duration(2))
                new_pose = PoseStamped()
                new_pose.header.frame_id = self.params['base_frame']
                new_pose.pose.orientation.w = 1.0
                new_pose = self.tfl.transformPose(self.params['world_frame'],
                                                  new_pose)
                dist = \
                    sqrt((new_pose.pose.position.x -
                          old_pose.pose.position.x) ** 2.0 +
                         (new_pose.pose.position.y -
                          old_pose.pose.position.y) ** 2.0)

                if self.params['debug']:
                    Me.info_message("New Pose: " + str(new_pose))
                    Me.info_message("Old Pose: " + str(old_pose))
                    Me.info_message("Dist: " + str(dist))

                if dist <= self.params['tolerance']:
                    num_stationary += 1
                elif dist > self.params['tolerance']:
                    # Movement occurred, zero number
                    num_stationary = 0
                else:
                    Me.error_message("Error with distance/tolerance "
                                     "comparision.")
                    raise KeyboardInterrupt
                if num_stationary >= self.params['num_stationary']:
                    Me.info_message(str(self.params['base_frame']) + " has "
                                    "been stationary for required " +
                                    str(total_time) + " seconds.")
                    if self.params['debug']:
                        Me.info_message("Polygons in map: " +
                                        str(len(self.map.polygons)))
                        Me.info_message("Right_left: " +
                                        str(self.params['right_left']))
                    # Call Adam's function (self.map, self.params['right_left'],
                    #  new_pose)
                    # self.turtlebot_ac.send_goal(nav_goal)
                    nav_goal.target_pose = \
                        get_viewpoint(self.map, self.params['right_left'],
                                      new_pose)
                    self.turtlebot_ac.send_goal(goal=nav_goal)
                else:
                    current_time = self.params['loop_rate'] * num_stationary
                    Me.info_message(str(self.params['base_frame']) + " has been"
                                    " stationary for " + str(current_time) +
                                    " seconds but " + str(total_time) +
                                    " seconds are needed.")
                # Store old pose
                old_pose = deepcopy(new_pose)
                # Wait before getting next pose
                sleep(self.params['loop_rate'])
            except TfExcept:
                if self.params['debug']:
                    Me.info_message('Transform exception')
                raise TfExcept
            except KeyboardInterrupt:
                break
        return

    def map_callback(self, data):
        """Retain the most current map data."""
        self.map = data
        return self.map


if __name__ == "__main__":
    NAME = 'cameraman'
    init_node(NAME)
    Me.info_message('Starting ' + NAME + ' node.')
    PARAMS = {'debug': get_param('~debug'),
              'world_frame': get_param('~world_frame'),
              'base_frame': get_param('~base_frame'),
              'right_left': get_param('~right_left'),
              'num_stationary': get_param('~num_stationary'),
              'loop_rate': get_param('~loop_rate'),
              'tolerance': get_param('~tolerance')}

    CAMERAMAN = CameraMan(params=PARAMS)
    Me.info_message(NAME + ' node initialized.')
    CAMERAMAN.loop()

    # Exit the program to not cause an error.
    signal_shutdown("KeyboardInterrupt")
    Me.shutdown_node()
