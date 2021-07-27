#!/usr/bin/env python3

# jplaced@unizar.es
# 2021, Universidad de Zaragoza

# This node recieve target exploration goals, which are the filtered frontier
# points published by the filter node, and commands the robot accordingly. The
# assigner node commands the robot through the move_base_node.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rospy
import networkx as nx
import numpy as np
import heapq

from copy import deepcopy

from constants import GRAPH_PATH_, PENALTY_, PLAN_POINT_TH_, EXPLORING_TIME_, USE_GPU_

from functions import robot, wait_enterKey, getGraph
from weighted_pose_graph_class import weighted_pose_graph
from graph_d_exploration.msg import PointArray

if USE_GPU_:
    from functions import cellInformation_NUMBA
else:
    from functions import cellInformation

from nav_msgs.msg import OccupancyGrid, Path

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Callbacks~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
mapData_ = OccupancyGrid()
frontiers_ = []


def frontiersCallBack(data):
    global frontiers_
    frontiers_ = []
    for point in data.points:
        frontiers_.append(np.array([point.x, point.y]))


def mapCallBack(data):
    global mapData_
    mapData_ = data


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Hallucinates graph along the path to a frontier and computes its utility
def hallucinateGraph(G, p_frontier, info_radius):
    # Initialize hallucinated graph
    G_frontier = weighted_pose_graph()
    G_frontier.copy_graph(G.graph)
    n = float(G.get_no_nodes())
    id_last = n - 1
    id_new = id_last

    # Pose of last known node
    temp = nx.get_node_attributes(G.graph, 'pose')
    p_last = np.array(temp[n - 1])
    temp = nx.get_node_attributes(G.graph, 'theta')
    th_last = temp[n - 1]

    # Hallucinate path
    plan = Path()
    plan = robot_.makePlan(robot_.getPosition(), p_frontier)
    n_plan = len(plan)

    # Add new nodes & edges along the hallucinated path to the new frontier
    plan_nodes = np.ceil(n_plan // PLAN_POINT_TH_)
    new_nodes = np.sort(np.random.choice(np.arange(0, n_plan - 1), int(plan_nodes), replace=False))

    for i in new_nodes:

        id_new += 1
        p_path = np.array([plan[i].pose.position.x, plan[i].pose.position.y])
        th_path = np.arctan2(p_path[1] - p_last[1], p_path[0] - p_last[0])
        G_frontier.graph.add_node(id_new, pose=p_path, theta=th_path)

        delta_path_p = p_path - p_last
        delta_path_th = th_path - th_last
        edges_Info = G_frontier.graph.edges([id_last], 'information')

        if len(list(edges_Info)[0]) == 3:
            fim_path = np.array(list(edges_Info)[0][2]) * PENALTY_  # The longer the path the smaller the FIM
            # Account for the expected unknown region that will be seen in the hallucinated path
            if USE_GPU_:
                normalized_unk_region_info_i, LC_info_i = cellInformation_NUMBA(np.array(mapData_.data),
                                                                                mapData_.info.resolution,
                                                                                mapData_.info.width,
                                                                                mapData_.info.origin.position.x,
                                                                                mapData_.info.origin.position.y,
                                                                                p_path[0], p_path[1], info_radius)
            else:
                normalized_unk_region_info_i, LC_info_i = cellInformation(mapData_, [p_path[0], p_path[1]], info_radius)

            # Account for the expected unknown region that exists in that frontier
            fim_path *= (1 + normalized_unk_region_info_i)
            # Account for potential LC's
            fim_path *= (1 + LC_info_i / 2)
            G_frontier.addEdge(id_last, id_new, 0, [delta_path_p, delta_path_th], fim_path)
            # Update variables
            id_last = id_new
            p_last = p_path
            th_last = th_path
        else:
            rospy.logwarn("Error in Information matrix - Check graph.")

    # Add new hallucinated frontier node with corresponding constraint
    id_frontier = id_last + 1
    th_frontier = np.arctan2(p_frontier[1] - p_last[1], p_frontier[0] - p_last[0])
    G_frontier.graph.add_node(id_frontier, pose=p_frontier, theta=th_frontier)
    delta_p = p_frontier - p_last
    delta_th = th_frontier - th_last
    frontier_Info = G_frontier.graph.edges([id_last], 'information')
    fim_frontier = np.array(list(frontier_Info)[0][2]) * PENALTY_  # The longer the path the smaller the FIM

    if USE_GPU_:
        normalized_unk_region_info, LC_info = cellInformation_NUMBA(np.array(mapData_.data), mapData_.info.resolution,
                                                                    mapData_.info.width,
                                                                    mapData_.info.origin.position.x,
                                                                    mapData_.info.origin.position.y,
                                                                    p_frontier[0], p_frontier[1], info_radius)
    else:
        normalized_unk_region_info, LC_info = cellInformation(mapData_, [p_frontier[0], p_frontier[1]], info_radius)

    # Account for the expected unknown region that exists in that frontier
    fim_frontier *= (1 + normalized_unk_region_info)
    # Account for potential LC's
    fim_frontier *= (1 + LC_info / 2)

    G_frontier.addEdge(id_last, id_frontier, 0, [delta_p, delta_th], fim_frontier)

    return G_frontier


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Node~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def node():
    global frontiers_, mapData_, robot_

    rospy.init_node('assigner', anonymous=False)

    map_topic = rospy.get_param('~map_topic', '/map')
    info_radius = rospy.get_param('~info_radius', 1.0)
    frontiers_topic = rospy.get_param('~frontiers_topic', '/filtered_points')
    namespace = rospy.get_param('~namespace', '')
    rateHz = rospy.get_param('~rate', 1)
    namespace_init_count = rospy.get_param('namespace_init_count', 1)
    delay_after_assignement = rospy.get_param('~delay_after_assignement', 0.5)

    rate = rospy.Rate(rateHz)
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(frontiers_topic, PointArray, frontiersCallBack)

    # Wait if no frontier is received yet
    while len(frontiers_) < 1:
        pass

    # Wait if map is not received yet
    while len(mapData_.data) < 1:
        pass

    robot_name = namespace + str(namespace_init_count)
    robot_ = robot(robot_name)
    robot_.sendGoal(robot_.getPosition())

    # Get ROS time in seconds
    t_0 = rospy.get_time()

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    while not rospy.is_shutdown():

        rospy.sleep(1)
        centroids = deepcopy(frontiers_)

        originalLen = len(centroids)
        for ip in range(0, originalLen):
            i = ip - originalLen + len(centroids)
            if np.linalg.norm(robot_.getPosition() - centroids[i]) < 0.25:
                rospy.logwarn("Deleted a frontier too close to the robot.")
                del centroids[i]

        n_centroids = len(centroids)

        # Get SLAM graph
        nodes, edges = getGraph(GRAPH_PATH_)
        G = weighted_pose_graph(nodes, edges, 'd_opt')

        # If no nodes (starting step) build graph with one edge at origin.
        n = float(G.get_no_nodes())
        m = float(G.get_no_edges())
        if n < 1:
            G.graph.add_node(0, pose=[0, 0], theta=0)

        infoGain = []
        closer_goal = False

        # If only one frontier no need to evaluate anything. Select that frontier
        if n_centroids == 1:
            rospy.logwarn("Only one frontier detected. Selecting it.")
            infoGain.append(np.random.rand(1, 1))
        # If no edges (starting step) do not evaluate D-opt. Select random frontier
        elif m < 1:
            rospy.logwarn("Graph not started yet, m < 1. Selecting goal +=[0.1,0.1].")
            closer_goal = True
        else:  # Otherwise
            rospy.loginfo("Computing information gain of every frontier candidate.")
            for ip in range(0, n_centroids):
                # Get frontier goal
                p_frontier = np.array([centroids[ip][0], centroids[ip][1]])

                # Compute hallucinated pose graph
                G_frontier = hallucinateGraph(G, p_frontier, info_radius)

                # Compute no. of spanning trees <=> D-opt(FIM)
                n_frontier = float(G_frontier.get_no_nodes())
                L_anch = G_frontier.compute_anchored_L()
                _, t = np.linalg.slogdet(L_anch.todense())
                spann = n_frontier ** (1 / n_frontier) * np.exp(t / n_frontier)
                infoGain.append(spann)

        if robot_.getState() == 1:
            rospy.logwarn("Robot is not available.")
        elif closer_goal:
            robot_.sendGoal(robot_.getPosition() + [0.1, 0.1])
        else:
            infoGain_record = []
            centroid_record = []

            for ip in range(0, len(centroids)):
                infoGain_record.append(infoGain[ip])
                centroid_record.append(centroids[ip])

            winner_id = infoGain_record.index(np.max(infoGain_record))

            rospy.loginfo("Information record: " + str(infoGain_record))
            rospy.loginfo("Centroids record: " + str(centroid_record))
            rospy.loginfo(robot_name + " assigned to " + str(centroid_record[winner_id]))

            initial_plan_position = robot_.getPosition()
            robot_.sendGoal(centroid_record[winner_id])

            # If plan fails near to starting position, send new goal to the next best frontier
            if robot_.getState() != 3:
                norm = np.linalg.norm(robot_.getPosition() - initial_plan_position)
                if norm <= 2.0:
                    try:
                        second_max = heapq.nlargest(2, infoGain_record)[1]
                        winner_id = infoGain_record.index(second_max)
                        rospy.logwarn("Goal aborted near previous pose (eucl = " + str(norm)
                                      + "). Sending new goal to: " + str(centroid_record[winner_id]))
                        robot_.sendGoal(centroid_record[winner_id])
                    finally:
                        pass
                else:
                    rospy.logwarn("Goal aborted away from previous pose (eucl = " + str(norm) + "). Recomputing.")

            rospy.sleep(delay_after_assignement)

        t_f = rospy.get_time() - t_0  # Get ROS time in seconds
        if t_f >= EXPLORING_TIME_:
            wait_enterKey()
        rate.sleep()


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Main~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
