#!/usr/bin/env python3

# jplaced@unizar.es
# 2022, Universidad de Zaragoza

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rospy
import tf
import actionlib
import numpy as np

import numba as nb
from numba import cuda

import networkx as nx
import scipy

from numpy.linalg import norm

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from nptyping import ndarray

cuda.select_device(0)


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Classes~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class robot:
    goal = MoveBaseGoal()
    start = PoseStamped()
    end = PoseStamped()

    def __init__(self, name):
        self.assigned_point = []
        self.name = name
        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.plan_service = rospy.get_param('~plan_service', '/move_base_node/NavfnROS/make_plan')
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(self.global_frame, self.robot_frame, rospy.Time(0), rospy.Duration(1.0))
        cond = 0
        while cond == 0:
            try:
                rospy.loginfo('Robot Class is waiting for the robot transform.')
                (trans, rot) = self.listener.lookupTransform(self.global_frame, self.robot_frame, rospy.Time(0))
                self.position = np.array([trans[0], trans[1]])
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # rospy.logerr(tf.LookupException);
                cond = 0
        rospy.loginfo('Robot Class received the robot transform.')
        self.assigned_point = self.position
        # self.client         = actionlib.SimpleActionClient(self.name+'/move_base', MoveBaseAction)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.client.wait_for_server()
        robot.goal.target_pose.header.frame_id = self.global_frame
        robot.goal.target_pose.header.stamp = rospy.Time.now()

        # rospy.wait_for_service(self.name+self.plan_service)
        # self.make_plan = rospy.ServiceProxy(self.name+self.plan_service, GetPlan)
        rospy.wait_for_service(self.plan_service)
        self.make_plan = rospy.ServiceProxy(self.plan_service, GetPlan)
        robot.start.header.frame_id = self.global_frame
        robot.end.header.frame_id = self.global_frame

    def getPosition(self):
        cond = 0
        while cond == 0:
            try:
                (trans, rot) = self.listener.lookupTransform(self.global_frame, self.robot_frame, rospy.Time(0))
                self.position = np.array([trans[0], trans[1]])
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond = 0
        return self.position

    def sendGoal(self, point):
        robot.goal.target_pose.pose.position.x = point[0]
        robot.goal.target_pose.pose.position.y = point[1]
        current_pos = self.getPosition()
        theta = np.arctan2(robot.goal.target_pose.pose.position.y - current_pos[1],
                           robot.goal.target_pose.pose.position.x - current_pos[0])
        robot.goal.target_pose.pose.orientation.x, robot.goal.target_pose.pose.orientation.y, \
        robot.goal.target_pose.pose.orientation.z, robot.goal.target_pose.pose.orientation.w = \
            euler2quaternion(0.0, 0.0, theta)
        self.client.send_goal(robot.goal)
        self.assigned_point = np.array(point)
        self.client.wait_for_result()

    def sendGoalNoWait(self, point):
        robot.goal.target_pose.pose.position.x = point[0]
        robot.goal.target_pose.pose.position.y = point[1]
        current_pos = self.getPosition()
        theta = np.arctan2(robot.goal.target_pose.pose.position.y - current_pos[1],
                           robot.goal.target_pose.pose.position.x - current_pos[0])
        robot.goal.target_pose.pose.orientation.x, robot.goal.target_pose.pose.orientation.y, \
        robot.goal.target_pose.pose.orientation.z, robot.goal.target_pose.pose.orientation.w = \
            euler2quaternion(0.0, 0.0, theta)
        self.client.send_goal(robot.goal)
        self.assigned_point = np.array(point)

    def cancelGoal(self):
        self.client.cancel_goal()
        self.assigned_point = self.getPosition()

    def getState(self):
        return self.client.get_state()

    def makePlan(self, start, end):
        robot.start.pose.position.x = start[0]
        robot.start.pose.position.y = start[1]
        robot.end.pose.position.x = end[0]
        robot.end.pose.position.y = end[1]
        start = self.listener.transformPose('map', robot.start)
        end = self.listener.transformPose('map', robot.end)
        plan = self.make_plan(start=start, goal=end, tolerance=0.0)
        return plan.plan.poses


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Returns the mapData's index of a given point (x,y) in the map
def index_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    index = int((np.floor((Xp[1] - Xstarty) // resolution) * width) + (np.floor((Xp[0] - Xstartx) // resolution)))
    return index


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# No difference in computation time from above.
@nb.jit(nb.int64(nb.float64, nb.float64, nb.float64, nb.int64, nb.float64[:]), nopython=True)
def index_of_point_NUMBA(resolution, Xstartx, Xstarty, width, Xp):
    index = (np.floor((Xp[1] - Xstarty) // resolution) * width) + (np.floor((Xp[0] - Xstartx) // resolution))
    return int(index)


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Returns the point (x,y) of a given mapData's index
def point_of_index(mapData, i):
    y = mapData.info.origin.position.y + (i // mapData.info.width) * mapData.info.resolution
    x = mapData.info.origin.position.x + (
            i - (i // mapData.info.width) * mapData.info.width) * mapData.info.resolution
    return np.array([x, y])


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# It is defined as the area of unknown region expected to be explored for a given
# frontier point.The information gain is quantified by counting the number of
# unknown cells surrounding a frontier point within a user defined radius. This
# radius is referred to as the information gain radius, which should be set to a
# value equal to the perception sensor range. The area is then calculated by
# multi-plying the number of cells within the information gain radius,by the
# area of each cell (which is computed from the map resolution).
def informationGain(mapData, point, r):
    infoGain = 0
    index = index_of_point(mapData, point)
    r_region = int(r // mapData.info.resolution)
    init_index = index - r_region * (mapData.info.width + 1)
    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        limit = ((start // mapData.info.width) + 2) * mapData.info.width
        for i in range(start, end + 1):
            if 0 <= i < len(mapData.data) and i < limit:
                if mapData.data[i] == -1 and norm(np.array(point) - point_of_index(mapData, i)) <= r:
                    infoGain += 1
    return infoGain * (mapData.info.resolution ** 2)


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
@nb.jit(nb.float64(nb.float64, nb.int64, nb.float64, nb.float64, nb.int64[:], nb.float64, nb.float64, nb.int64),
        nopython=True)
def informationGain_NUMBA(resolution, width, Xstartx, Xstarty, data, pointx, pointy, r):
    infoGain = 0
    index = int((np.floor((pointy - Xstarty) // resolution) * width) + (np.floor((pointx - Xstartx) // resolution)))
    r_region = int(r // resolution)
    init_index = index - r_region * (width + 1)
    for n in range(0, 2 * r_region + 1):
        start = n * width + init_index
        end = start + 2 * r_region
        limit = ((start // width) + 2) * width
        for i in range(start, end + 1):
            if 0 <= i < len(data) and i < limit:
                y = Xstarty + (i // width) * resolution
                x = Xstartx + (i - (i // width) * width) * resolution
                poi = np.array([x, y])
                if data[i] == -1 and norm(np.array([pointx, pointy]) - poi) <= r:
                    infoGain += 1
    return infoGain * (resolution ** 2)


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# USED IN FRONTIER FILTERING
# Returns grid value at "Xp" location
# Map data:  100 occupied      -1 unknown       0 free
def gridValue(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    Data = mapData.data
    index = (np.floor((Xp[1] - Xstarty) // resolution) * width) + (np.floor((Xp[0] - Xstartx) // resolution))
    return [100, Data[int(index)]][int(index) < len(Data)]


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Graph wrapper
def getGraph(filename):
    nodes = []
    edges = []

    with open(filename) as fp:
        lines = fp.readlines()
        for x in lines:
            edge_type = x.split(' ')[0]
            if edge_type == "VERTEX_SE2":
                node = np.float_(x.split(' ')[1])
                pose = np.float_(x.split(' ')[2:5])
                nodes.append(np.concatenate([np.array([node]), pose]).ravel())
            elif edge_type == "EDGE_SE2":
                node1 = np.float_(x.split(' ')[1])
                node2 = np.float_(x.split(' ')[2])
                etype = 0 if (abs(node1 - node2) == 1) else 1
                delta = np.float_(x.split(' ')[3:6])
                FIM = np.float_(x.split(' ')[7:13])
                edges.append(np.concatenate([np.array([node1, node2, etype]), delta, FIM]).ravel())

    return nodes, edges


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def wait_enterKey():
    input("Press Enter to continue...")


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def euler2quaternion(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return qx, qy, qz, qw


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def createMarker(mtype: str = "point", frame: str = "/map", ns: str = "marker_ns", lifetime: float = 0.12,
                 colors: ndarray = None, alpha: float = 1.0, scale: float = 0.3) -> Marker:
    """
    Initializes a ROS visualization_msgs Marker
    """
    if colors is None:
        colors = [255, 0, 0]

    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = 0
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.color.a = alpha
    marker.color.r = colors[0] / 255
    marker.color.g = colors[1] / 255
    marker.color.b = colors[2] / 255
    marker.lifetime = rospy.Duration(lifetime)

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    if mtype == "point":
        marker.type = Marker.POINTS
        marker.scale.x = marker.scale.y = scale
    elif mtype == "sphere":
        marker.type = Marker.SPHERE
        marker.scale.x = marker.scale.y = marker.scale.z = scale  # Diameter
    elif mtype == "arrow":
        marker.type = Marker.ARROW
        marker.scale.x = scale  # Arrow length
        marker.scale.y = marker.scale.z = 0.05  # Arrow head diameter and length
    elif mtype == "cube":
        marker.type = Marker.CUBE
        marker.scale.x = marker.scale.y = marker.scale.z = scale
    elif mtype == "circumference":
        marker.type = Marker.SPHERE
        marker.scale.x = marker.scale.y = scale
        marker.scale.z = 0.05
        marker.pose.position.z = 0.0
    elif mtype == "lines":
        marker.type = Marker.LINE_STRIP
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.scale.x = scale

    return marker


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def cellInformation(mapData, point, r):
    cells = [0, 0, 0]  # every cell, unknown cells, occupied cells
    index = index_of_point(mapData, point)
    r_region = int(r // mapData.info.resolution)
    init_index = index - r_region * (mapData.info.width + 1)

    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        limit = start + 2 * mapData.info.width
        for i in range(start, end + 1):
            if 0 <= i < np.min([limit, len(mapData.data)]) and norm(
                    np.array(point) - point_of_index(mapData, i)) <= r:
                cells[0] += 1
                if mapData.data[i] == -1:
                    cells[1] += 1  # Unknown
                elif mapData.data[i] == 100:
                    cells[2] += 1  # Occupied

    # Normalized information gain due to unknown region
    unknown_area_gain = (float(cells[1]) / float(cells[0]))
    # Normalized information gain due to LC candidates
    LC_gain = (float(cells[2]) / float(cells[0]))

    return unknown_area_gain, LC_gain


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
@nb.jit(nb.types.Tuple((nb.float64, nb.float64))(nb.int64[:], nb.float64, nb.int64, nb.float64, nb.float64, nb.float64,
                                                 nb.float64, nb.int64), nopython=True)
def cellInformation_NUMBA(data, resolution, width, Xstartx, Xstarty, pointx, pointy, r):
    cells = [0, 0, 0]  # every cell, unknown cells, occupied cells
    index = int((np.floor((pointy - Xstarty) // resolution) * width) + (np.floor((pointx - Xstartx) // resolution)))
    r_region = int(r // resolution)
    init_index = index - r_region * (width + 1)

    for n in range(0, 2 * r_region + 1):
        start = n * width + init_index
        end = start + 2 * r_region
        limit = start + 2 * width
        for i in range(start, end + 1):
            if 0 <= i < len(data) and i < limit:
                y = Xstarty + (i // width) * resolution
                x = Xstartx + (i - (i // width) * width) * resolution
                poi = np.array([x, y])
                if norm(np.array([pointx, pointy]) - poi) <= r:
                    cells[0] += 1
                    if data[i] == -1:
                        cells[1] += 1  # Unknown
                    elif data[i] == 100:
                        cells[2] += 1  # Occupied

    # Normalized information gain due to unknown region
    unknown_area_gain = (float(cells[1]) / float(cells[0]))
    # Normalized information gain due to LC candidates
    LC_gain = (float(cells[2]) / float(cells[0]))

    return unknown_area_gain, LC_gain
