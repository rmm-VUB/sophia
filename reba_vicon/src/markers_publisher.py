#!/usr/bin/env python


import rospy

from math import *
import numpy as np


from mocap_vicon.msg import Markers
from mocap_vicon.msg import Marker

import geometry_msgs.msg
from geometry_msgs.msg import Point

import std_msgs.msg
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

import sys

def draw_markers(markers):
    # Drawing of the markers
    # Set chart title.
    fig = plt.figure()
    ax = plt.axes(projection='3d')

    plt.title("Markers positions", fontsize=19)

    for letter in enumerate(markers):
        #plt.scatter(markers[letter[1]][0], markers[letter[1]][1], s=1000)
        ax.scatter3D(markers[letter[1]][0], markers[letter[1]][1], markers[letter[1]][2], cmap='Greens')

    # Display the plot in the matplotlib's viewer.
    #plt.draw()
    plt.show()
    #plt.waitforbuttonpress(0)
    plt.close(fig)

if __name__ == '__main__':
    pub_markers = rospy.Publisher('/vicon/labeled_markers', Markers, queue_size=0)

    rospy.init_node('markers_publisher')

    list_dot = ['lower_arm_r1','lower_arm_r2','upper_arm_r1','shoulder_r1','shoulder_l2','upper_arm_l2','lower_arm_l2','neck2','hip_r2','hip_l2','knee_r2','knee_l2','ankle_r2','ankle_l2']

    pos_markers = {'lower_arm_r1':[5,0,20],'lower_arm_r2':[0,0,0],'upper_arm_r1':[10,0,40],'shoulder_r1':[25,0,40],'shoulder_l2':[40,0,40],'upper_arm_l2':[45,0,20],'lower_arm_l2':[50,0,0],'neck2':[25,0,50],
                   'hip_r2':[15,0,10],'hip_l2':[35,0,10],'knee_r2':[15-5,0,-10], 'knee_l2':[35+5,0,-10], 'ankle_r2':[15-5,0,-30], 'ankle_l2':[35+5,0,-30]}

    draw_markers(pos_markers)

    fr_nb = 0
    print "Publishing markers.."
    while not rospy.is_shutdown():
        msg = Markers()
        msg.frame_number = fr_nb
        msg.header = std_msgs.msg.Header()
        msg.header.stamp = rospy.Time.now()

        msg.markers = []
        for name in list_dot:
            m = Marker()
            m.marker_name = name
            m.subject_name = 'human'
            m.segment_name = 'seg'
            m.translation.x = pos_markers[name][0]
            m.translation.y = pos_markers[name][1]
            m.translation.z = pos_markers[name][2]
            msg.markers.append(m)

        #print msg.markers
        pub_markers.publish(msg)
        fr_nb = fr_nb + 1
        rospy.sleep(0.01)
