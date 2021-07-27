#!/usr/bin/env python3

# jplaced@unizar.es
# 2021, Universidad de Zaragoza

# This node initializes the frontier detectors with a pre defined starting map
# size.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rospy
import os

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def init_point_selector(idx):
    points = [[-25, 25], [-25, -25], [25, -25], [25, 25], [0.140955, -0.0519512]]

    FivePoints = []
    for (x, y) in points:
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.0
        FivePoints.append(p)

    init_points = Marker()
    init_points.header.frame_id = "map"
    init_points.header.stamp = rospy.get_rostime()
    init_points.points = FivePoints[0:idx]

    return init_points


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Node~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def node():
    rospy.init_node('initializer', anonymous=False)

    rateHz = rospy.get_param('~rate', 1)
    rate = rospy.Rate(rateHz)
    marker_pub = rospy.Publisher('/init_points', Marker, queue_size=5)

    iteration = 0
    while not rospy.is_shutdown() and iteration <= 5:
        if iteration > 5:
            iteration = 5
        init_points = init_point_selector(iteration)
        marker_pub.publish(init_points)
        iteration += 1
        rate.sleep()

    print("Shutting down initializer node.")
    try:
        os.system("rosnode kill /point_init")
    except KeyError:
        pass


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Main~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
