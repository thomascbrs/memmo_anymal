#!/usr/bin/env python3
#
# Copyright 2022 University of Edinburgh
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of  nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import numpy as np
import pinocchio as pin
import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class WalkgenVisualizationPublisher():

    def __init__(self, topic_marker, topic_marker_array, queue_size=10):
        # Initializing the publisher
        self._marker_pub = rospy.Publisher(topic_marker, Marker, queue_size=queue_size)
        self._marker_array_pub = rospy.Publisher(topic_marker_array, MarkerArray, queue_size=queue_size)

    def publish_world(self, worldMesh, worldPose, frame_id="map"):
        """ Publish mesh as marker message.

        Args:
            - worldMesh (str): mesh.
            - worldPose (list,array x7): Position, Orientation (Quaternion).
            - frame_id (str): Frame.
        """
        if len(worldPose) != 7:
            raise ArithmeticError("worldPose should be size 7 (position, Orientation)")
        msg = Marker()
        msg.header.frame_id = frame_id
        msg.header.stamp = rospy.Time.now()
        msg.ns = "map"
        msg.id = 0
        msg.type = msg.MESH_RESOURCE
        msg.pose.position.x = worldPose[0]
        msg.pose.position.y = worldPose[1]
        msg.pose.position.z = worldPose[2]
        msg.pose.orientation.x = worldPose[3]
        msg.pose.orientation.y = worldPose[4]
        msg.pose.orientation.z = worldPose[5]
        msg.pose.orientation.w = worldPose[6]
        msg.lifetime = rospy.Duration(0)
        msg.scale.x = 1
        msg.scale.y = 1
        msg.scale.z = 1
        msg.color.a = 1.0
        msg.color.r = 0.7
        msg.color.g = 0.7
        msg.color.b = 0.7
        msg.mesh_resource = "file://" + worldMesh
        self._marker_pub.publish(msg)

    def publish_fsteps(self, all_feet_pos, lifetime = 0., frame_id="map"):
        """ Publish the foostep optimised by sl1m.

        Args:
            - all_feet_pos (list) : List containing the footstep optimised by the MIP.
            - lifetime (float): Duration in [s].
            - frame_id (str): Frame.
        """
        msg = MarkerArray()
        color = [[1.,0.,0.,1.],[0.,0.,1.,1.],[0.,1.,0.,1.],[0.,1.,1.,1.]]
        for foot_id,all_foot_pos in enumerate(all_feet_pos):
            for id_,foot_pos in enumerate(all_foot_pos):
                if foot_pos is not None:
                    marker_x = Marker()

                    marker_x.header.frame_id = frame_id
                    marker_x.header.stamp = rospy.Time.now()
                    marker_x.lifetime = rospy.Duration(
                        lifetime)  # How long the object should last before being automatically deleted.  0 means forever
                    marker_x.ns = "arrow_config_" + str(id) + str(id_)
                    marker_x.id = 50*foot_id + id_
                    marker_x.type = 2 # Sphere
                    marker_x.action = 0  # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
                    # Pose
                    marker_x.pose.position.x = foot_pos[0]
                    marker_x.pose.position.y = foot_pos[1]
                    marker_x.pose.position.z = foot_pos[2]
                    # Color
                    marker_x.color.r = color[foot_id][0]
                    marker_x.color.g = color[foot_id][1]
                    marker_x.color.b = color[foot_id][2]
                    marker_x.color.a = color[foot_id][3]
                    marker_x.scale.x = 0.06
                    marker_x.scale.y = 0.06
                    marker_x.scale.z = 0.06
                    marker_x.frame_locked = True
                    msg.markers.append(marker_x)
        self._marker_array_pub.publish(msg)

    def publish_config(self, configs, lifetime = 0., frame_id="map"):
        """ Publish the configuration of each phase of contact of the MIP.

        Args:
            - configs (list): List of config (array x7 Positon and Orientation)
            - lifetime (float): Duration in [s].
            - frame_id (str): Frame.
        """
        msg = MarkerArray()
        rpy = np.array([0,0,np.pi/2])
        mat_y = pin.rpy.rpyToMatrix(rpy)
        for id,config in enumerate(configs):
            marker_x = Marker()
            color = [1., 0., 0., 1.]

            marker_x.header.frame_id = frame_id
            marker_x.header.stamp = rospy.Time.now()
            marker_x.lifetime = rospy.Duration(
                lifetime)  # How long the object should last before being automatically deleted.  0 means forever
            marker_x.ns = "arrow_config"
            marker_x.id = id
            marker_x.type = 0  # Arrow
            marker_x.action = 0  # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
            # Pose
            marker_x.pose.position.x = config[0]
            marker_x.pose.position.y = config[1]
            marker_x.pose.position.z = config[2]
            marker_x.pose.orientation.x = config[3]
            marker_x.pose.orientation.y = config[4]
            marker_x.pose.orientation.z = config[5]
            marker_x.pose.orientation.w = config[6]
            # Color
            marker_x.color.r = color[0]
            marker_x.color.g = color[1]
            marker_x.color.b = color[2]
            marker_x.color.a = color[3]
            marker_x.scale.x = 0.1
            marker_x.scale.y = 0.01
            marker_x.scale.z = 0.01
            marker_x.frame_locked = True
            msg.markers.append(marker_x)


            marker_y = Marker()
            color = [0., 1., 0., 1.]
            quat = pin.Quaternion(np.dot(mat_y, pin.Quaternion(config[3:]).toRotationMatrix()))
            marker_y.header.frame_id = frame_id
            marker_y.header.stamp = rospy.Time.now()
            marker_y.lifetime = rospy.Duration(
                lifetime)  # How long the object should last before being automatically deleted.  0 means forever
            marker_y.ns = "arrow_config"
            marker_y.id = id + 50
            marker_y.type = 0  # Arrow
            marker_y.action = 0  # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
            # Pose
            marker_y.pose.position.x = config[0]
            marker_y.pose.position.y = config[1]
            marker_y.pose.position.z = config[2]
            marker_y.pose.orientation.x = quat.x
            marker_y.pose.orientation.y = quat.y
            marker_y.pose.orientation.z = quat.z
            marker_y.pose.orientation.w = quat.w
            # Color
            marker_y.color.r = color[0]
            marker_y.color.g = color[1]
            marker_y.color.b = color[2]
            marker_y.color.a = color[3]
            marker_y.scale.x = 0.1
            marker_y.scale.y = 0.01
            marker_y.scale.z = 0.01
            marker_y.frame_locked = True
            msg.markers.append(marker_y)
        self._marker_array_pub.publish(msg)

    def publish_surfaces(self, surfaces, lifetime=0, frame_id="map"):
        """ Publish the surfaces computed by the MIP.
        Args:
            - surfaces (list): List of array 3xn vertices positions:
                        array([[x0, x1, ... , xn],
                                [y0, y1, ... , yn],
                                [z0, z1, ... , zn]])
            - lifetime (float): Duration in [s].
            - frame_id (str): Frame.
        """
        if surfaces[0].shape[0] != 3:
            raise ArithmeticError("Vertices should be an array of size 3xn")

        msg = MarkerArray()
        for id, vertices in enumerate(surfaces):
            marker = Marker()
            color = [1., 0., 0., 1.]

            marker.header.frame_id = frame_id
            marker.header.stamp = rospy.Time.now()
            marker.lifetime = rospy.Duration(
                lifetime)  # How long the object should last before being automatically deleted.  0 means forever
            marker.ns = "hull"
            marker.id = id
            marker.type = 4  # 4 LINE_STRIP, 8 POINTS
            marker.action = 0  # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
            # Pose
            marker.pose.position.x = 0
            marker.pose.position.y = 0
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            # Color
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]
            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03
            marker.frame_locked = True
            for k in range(vertices.shape[1] - 1):
                point_a = Point()
                point_a.x = vertices[0, k]
                point_a.y = vertices[1, k]
                point_a.z = vertices[2, k]
                marker.points.append(point_a)
                point_b = Point()
                point_b.x = vertices[0, k + 1]
                point_b.y = vertices[1, k + 1]
                point_b.z = vertices[2, k + 1]
                marker.points.append(point_b)
            # Add end line
            point_a = Point()
            point_a.x = vertices[0, 0]
            point_a.y = vertices[1, 0]
            point_a.z = vertices[2, 0]
            marker.points.append(point_a)
            point_b = Point()
            point_b.x = vertices[0, -1]
            point_b.y = vertices[1, -1]
            point_b.z = vertices[2, -1]
            marker.points.append(point_b)
            msg.markers.append(marker)
        self._marker_array_pub.publish(msg)