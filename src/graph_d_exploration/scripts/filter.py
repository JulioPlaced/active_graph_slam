#!/usr/bin/env python3

# jplaced@unizar.es
# 2021, Universidad de Zaragoza

# The filter nodes receives the detected frontier points from all the detectors,
# filters the points, and passes them to the assigner node to command the robots.
# Filtration includes the delection of old and invalid points, and it also
# discards redundant points.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rospy
import tf

import numpy as np
from sklearn.cluster import MeanShift
from copy import copy, deepcopy

from constants import USE_GPU_
from functions import gridValue, createMarker

if USE_GPU_:
    from functions import informationGain_NUMBA
else:
    from functions import informationGain

from graph_d_exploration.msg import PointArray

from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PointStamped
from dynamic_reconfigure.server import Server
from graph_d_exploration.cfg import informationGainConfig

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Callbacks~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
mapData_ = OccupancyGrid()
frontiers_ = []
f_timestamps_ = []
globalmaps_ = []
INFORMATION_THRESHOLD_ = 0.45


def reconfigureCallback(config, level):
    global INFORMATION_THRESHOLD_
    rospy.logwarn("""Reconfigure Request! InfoGain threshold changed to: {ig_threshold}""".format(**config))
    INFORMATION_THRESHOLD_ = config["ig_threshold"]
    return config


def frontiersCallBack(data, args):
    global frontiers_, f_timestamps_

    transformedPoint = args[0].transformPoint(args[1], data)
    x = np.array([transformedPoint.point.x, transformedPoint.point.y])
    x_t = data.header.stamp.to_sec()

    # Only add if not already there
    # Use temp variables to avoid errors due to global variables dimension (multiple ROS cb same time)
    temp_black = []
    temp_black = copy(x.tolist())
    temp_array = []
    temp_array = np.asarray(copy(frontiers_)).tolist()
    temp_time = []
    temp_time = copy(f_timestamps_)

    if len(temp_array) == len(temp_time):
        assert (len(temp_array) == len(temp_time))
        if temp_black in temp_array:
            repeated_idx = temp_array.index(temp_black)
            temp_time[repeated_idx] = x_t
        else:  # Otherwise, update timestamp
            temp_array.append(x)
            temp_time.append(x_t)

        # Delete too old points
        originalLen = len(temp_array)
        assert originalLen == len(temp_time)
        for ip in range(0, originalLen):
            i = ip - originalLen + len(temp_array)
            t_diff = np.abs(temp_time[i] - rospy.get_time())
            if t_diff > 5.0:
                rospy.logdebug("Deleted a frontier with timestamp diff = " + str(t_diff))
                del temp_array[i]
                del temp_time[i]

        frontiers_ = copy(temp_array)
        f_timestamps_ = copy(temp_time)
        assert (len(frontiers_) == len(f_timestamps_))
    else:
        rospy.logerr("Frontier callback failed due to dimension mismatch of " + str(
            len(temp_array) - len(temp_time)) + ". Skipping callback.")


def mapCallBack(data):
    global mapData_
    mapData_ = data


def globalMapCallback(data):
    global globalmaps_, litraIndx_, namespace_init_count_, n_robots_
    if n_robots_ > 1:
        indx = int(data._connection_header['topic'][litraIndx_]) - namespace_init_count_
    elif n_robots_ == 1:
        indx = 0
    globalmaps_[indx] = data


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Node~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def node():
    global frontiers_, mapData_, globalmaps_, litraIndx_, n_robots_, namespace_init_count_
    rospy.init_node('filter', anonymous=False)

    # Fetch all parameters
    map_topic = rospy.get_param('~map_topic', '/map')
    threshold = rospy.get_param('~costmap_clearing_threshold', 70)
    goals_topic = rospy.get_param('~goals_topic', '/detected_points')
    n_robots_ = rospy.get_param('~n_robots', 1)
    namespace = rospy.get_param('~namespace', '')
    rateHz = rospy.get_param('~rate', 50)
    robot_frame = rospy.get_param('~robot_frame', 'base_link')
    global_costmap_topic = rospy.get_param('~global_costmap_topic', '/move_base_node/global_costmap/costmap')
    namespace_init_count_ = rospy.get_param('namespace_init_count', 1)

    srv = Server(informationGainConfig, reconfigureCallback)

    litraIndx_ = len(namespace)
    rate = rospy.Rate(rateHz)
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)

    for i in range(0, n_robots_):
        globalmaps_.append(OccupancyGrid())

    if len(namespace) > 0:
        for i in range(0, n_robots_):
            rospy.Subscriber(global_costmap_topic, OccupancyGrid, globalMapCallback)
    elif len(namespace) == 0:
        rospy.Subscriber(global_costmap_topic, OccupancyGrid, globalMapCallback)

    # Wait if map is not received yet
    while len(mapData_.data) < 1:
        rospy.loginfo('Filter is waiting for the map.')
        rospy.sleep(0.5)
        pass

    # Wait if any of robots' global costmap map is not received yet
    for i in range(0, n_robots_):
        while len(globalmaps_[i].data) < 1:
            rospy.loginfo('Filter is waiting for the global costmap.')
            rospy.sleep(0.5)
            pass

    rospy.loginfo("Filter received local and global costmaps.")

    global_frame = "/" + mapData_.header.frame_id
    tfLisn = tf.TransformListener()
    if len(namespace) > 0:
        for i in range(0, n_robots_):
            tfLisn.waitForTransform(global_frame[1:], robot_frame, rospy.Time(0), rospy.Duration(10.0))
    elif len(namespace) == 0:
        tfLisn.waitForTransform(global_frame[1:], robot_frame, rospy.Time(0), rospy.Duration(10.0))

    rospy.Subscriber(goals_topic, PointStamped, callback=frontiersCallBack, callback_args=[tfLisn, global_frame[1:]])

    # Publishers
    pub_frontiers = rospy.Publisher('frontiers', Marker, queue_size=10)
    pub_centroids = rospy.Publisher('centroids', Marker, queue_size=10)
    pub_filtPoints = rospy.Publisher('filtered_points', PointArray, queue_size=10)

    # Wait if no frontier is received yet
    counter = 0
    while len(frontiers_) < 1:
        if counter == 0:
            rospy.loginfo("Filter is waiting for frontiers.")
            counter = 1

    rospy.loginfo("Filter received frontiers.")

    points = createMarker(frame=mapData_.header.frame_id, ns="raw_frontiers", colors=[1.0, 1.0, 0.0], scale=0.2)
    points_clust = createMarker(frame=mapData_.header.frame_id, ns="filtered_frontiers", colors=[0.0, 1.0, 0.0])

    p = Point()
    p.z = 0

    tempPointStamped = PointStamped()
    tempPointStamped.header.frame_id = mapData_.header.frame_id
    tempPointStamped.header.stamp = rospy.Time(0)
    tempPointStamped.point.z = 0.0

    tempPointArray = PointArray()
    tempPoint = Point()
    tempPoint.z = 0.0

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    while not rospy.is_shutdown():
        # Clustering frontier points
        centroids = []
        front = deepcopy(frontiers_)
        if len(front) > 1:
            ms = MeanShift(bandwidth=1.5)
            ms.fit(front)
            centroids = ms.cluster_centers_
        elif len(front) == 1:  # If there is only one frontier no need for clustering
            centroids = front

        # frontiers_ = deepcopy(centroids)

        # Clearing frontiers
        originalCentroidsLen = len(centroids)
        for zp in range(0, originalCentroidsLen):
            z = zp - originalCentroidsLen + len(centroids)

            cond = False
            tempPointStamped.point.x = centroids[z][0]
            tempPointStamped.point.y = centroids[z][1]

            # Remove any frontier inside an occupied cell (threshold)
            for i in range(0, n_robots_):
                transformedPoint = tfLisn.transformPoint(globalmaps_[i].header.frame_id, tempPointStamped)
                x = np.array([transformedPoint.point.x, transformedPoint.point.y])
                cond = (gridValue(globalmaps_[i], x) >= threshold) or cond

            # Remove frontiers with low information gain
            if USE_GPU_:
                ig = informationGain_NUMBA(mapData_.info.resolution, mapData_.info.width,
                                           mapData_.info.origin.position.x,
                                           mapData_.info.origin.position.y, np.array(mapData_.data), centroids[z][0],
                                           centroids[z][1], 1.0)
            else:
                ig = informationGain(mapData_, [centroids[z][0], centroids[z][1]], 1.0)

            cond = (ig < INFORMATION_THRESHOLD_) or cond
            # Since radius=1 -> maximum ig=2pi. If less than 15%, delete frontier
            if cond:
                centroids = np.delete(centroids, z, axis=0)
                rospy.logdebug("Deleted a frontier with information gain = " + str(ig))

        rospy.logdebug("Frontier centroids len=" + str(len(centroids)) + ", frontiers len=" + str(
            len(front)) + ". Centroids: \n" + str(centroids))

        # Publishing
        tempPointArray.points = []
        pp = []
        for i in centroids:
            tempPoint.x = i[0]
            tempPoint.y = i[1]
            tempPointArray.points.append(copy(tempPoint))
            pp.append(copy(tempPoint))
        points_clust.id += 1
        points_clust.points = pp
        pub_centroids.publish(points_clust)
        pub_filtPoints.publish(tempPointArray)

        pp = []
        for q in front:
            p.x = q[0]
            p.y = q[1]
            pp.append(copy(p))
        points.id += 1
        points.points = pp
        pub_frontiers.publish(points)

        rate.sleep()


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Main~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
