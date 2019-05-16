#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Martin Guenther
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

'''This is a converter for the Intel Research Lab SLAM dataset
   ( http://kaspar.informatik.uni-freiburg.de/~slamEvaluation/datasets/intel.clf )
   to rosbag'''

import rospy
import rosbag
from sensor_msgs.msg import LaserScan
from math import pi
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf

def make_tf_msg(x, y, theta, t):
    trans = TransformStamped()
    trans.header.stamp = t
    trans.header.frame_id = '/odom'
    trans.child_frame_id = '/base_footprint'
    trans.transform.translation.x = x
    trans.transform.translation.y = y
    q = tf.transformations.quaternion_from_euler(0, 0, theta)
    trans.transform.rotation.x = q[0]
    trans.transform.rotation.y = q[1]
    trans.transform.rotation.z = q[2]
    trans.transform.rotation.w = q[3]

    msg = TFMessage()
    msg.transforms.append(trans)
    return msg

def mak_odom_msg(x, y, theta, t):
    msg = Odometry()
    msg.header.stamp = t
    msg.header.frame_id = "/odom"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0
    q = tf.transformations.quaternion_from_euler(0, 0, theta)
    msg.pose.pose.orientation.x = q[0]
    msg.pose.pose.orientation.y = q[1]
    msg.pose.pose.orientation.z = q[2]
    msg.pose.pose.orientation.w = q[3]

    return msg

with open('intel.clf') as dataset:
    with rosbag.Bag('aces.bag', 'w') as bag:
        for line in dataset.readlines():
            line = line.strip()
            tokens = line.split(' ')
            if len(tokens) <= 2:
                continue
            if tokens[0] == 'FLASER':
                msg = LaserScan()
                num_scans = int(tokens[1])

                if num_scans != 180 or len(tokens) < num_scans + 9:
                    rospy.logwarn("unsupported scan format")
                    continue

                msg.header.frame_id = 'laser'
                t = rospy.Time(float(tokens[(num_scans + 8)]))
                msg.header.stamp = t
                msg.angle_min = -90.0 / 180.0 * pi
                msg.angle_max = 90.0 / 180.0 * pi
                msg.angle_increment = pi / (num_scans - 1.0)
                msg.time_increment = 0.2 / 360.0
                msg.scan_time = 0.2
                msg.range_min = 0.001
                msg.range_max = 50.0
                msg.ranges = [float(r) for r in tokens[2:(num_scans + 2)]]

                bag.write('scan', msg, t)

                #odom_x, odom_y, odom_theta = [float(r) for r in tokens[(num_scans + 2):(num_scans + 5)]]
                #tf_msg = make_tf_msg(odom_x, odom_y, odom_theta, t)
                #bag.write('tf', tf_msg, t)
                #odom_msg = mak_odom_msg(odom_x, odom_y, odom_theta,t)
                #bag.write("odom", odom_msg, t)

            elif tokens[0] == 'ODOM':
                odom_x, odom_y, odom_theta = [float(t) for t in tokens[1:4]]
                t = rospy.Time(float(tokens[7]))
                tf_msg = make_tf_msg(odom_x, odom_y, odom_theta, t)
                bag.write('tf', tf_msg, t)
                odom_msg = mak_odom_msg(odom_x, odom_y, odom_theta, t)
                bag.write("odom", odom_msg, t)