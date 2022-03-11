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

import rospy
from footstep_msgs.msg import SetSurfaces, FootSurfaces, ConvexSurface, GaitStatusOnNewPhase
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import numpy as np

from walkgen_ros.tools.Surface import Surface


class SurfacePublisher():

    def __init__(self, topic, queue_size=10):
        # Initializing the publisher
        self._pub = rospy.Publisher(topic, SetSurfaces, queue_size=queue_size)
        self._surfacePlanner_iface = SurfacePlannerInterface()

    def publish(self, t, set_surfaces):
        msg = self._surfacePlanner_iface.writeToMessage(t, set_surfaces)
        self._pub.publish(msg)


class StepManagerPublisher():

    def __init__(self, topic, queue_size=10):
        # Initializing the publisher
        self._pub = rospy.Publisher(topic, GaitStatusOnNewPhase, queue_size=queue_size)
        self._stepmanager_iface = StepManagerInterface()

    def publish(self, t, gait, target_foostep):
        msg = self._stepmanager_iface.writeToMessage(t, gait, target_foostep)
        self._pub.publish(msg)


class SurfacePlannerInterface():

    def __init__(self):
        self._msg = SetSurfaces()

    def writeToMessage(self, t, set_surfaces):
        """ Write data to SurfacesSL1M message.

        Args:
            - t (float): time.
            - set_surfaces (dict): Dictionnary object containing the next surfaces for each foot.
        """
        self._msg.header.stamp = rospy.Time(t)
        self._msg.set_surfaces = []
        for key in set_surfaces:
            msg_tmp = FootSurfaces()
            msg_tmp.name = key
            for sf in set_surfaces[key]:
                surface = ConvexSurface()
                # Inequality matrix.
                A = Float64MultiArray()
                A.layout.dim.append(MultiArrayDimension())
                A.layout.dim.append(MultiArrayDimension())
                A.layout.dim[0].label = "height"
                A.layout.dim[1].label = "width"
                A.layout.dim[0].size = sf.A.shape[0]
                A.layout.dim[1].size = sf.A.shape[1]
                A.data = np.ravel(sf.A, order="C").tolist()
                surface.A = A

                # Inequality vector
                surface.b = sf.b.tolist()

                # Vertices matrix.
                vert = Float64MultiArray()
                vert.layout.dim.append(MultiArrayDimension())
                vert.layout.dim.append(MultiArrayDimension())
                vert.layout.dim[0].label = "height"
                vert.layout.dim[1].label = "width"
                vert.layout.dim[0].size = sf.vertices.shape[0]
                vert.layout.dim[1].size = sf.vertices.shape[1]
                vert.data = np.ravel(sf.vertices, order="C").tolist()
                surface.vertices = vert

                msg_tmp.surfaces.append(surface)

            self._msg.set_surfaces.append(msg_tmp)

        return self._msg

    def writeFromMessage(self, msg):
        """ Extract data from SurfacesSL1M message
        """
        set_surfaces = dict()
        for foot_surfaces in msg.set_surfaces:
            L = []
            for sf in foot_surfaces.surfaces:
                ia = sf.A.layout.dim[0].size
                ja = sf.A.layout.dim[1].size
                iv = sf.vertices.layout.dim[0].size
                jv = sf.vertices.layout.dim[1].size
                L.append(
                    Surface(
                        np.array(sf.A.data).reshape((ia, ja), order="C"), np.array(sf.b),
                        np.array(sf.vertices.data).reshape((iv, jv), order="C")))
            set_surfaces[foot_surfaces.name] = L

        return set_surfaces


class StepManagerInterface():

    def __init__(self):
        self._msg = GaitStatusOnNewPhase()
        self._contact_names = ['LF_FOOT', 'RF_FOOT', 'LH_FOOT', 'RH_FOOT']  # Order of the feet in the surface planner.

    def writeToMessage(self, t, gait, foot_pos):
        """ Write data to ROS message FootStepStateSL1M.
        Args:
            - t (time): Current timing.
            - gait (Array n_gait x 4): Next walking gait pattern.
            - foot_pos (Array 3x4): Foot position for the next phase. (For the next phase of contact)
        """
        self._msg.header.stamp = rospy.Time(t)

        # Gait matrix
        gait_mat = Float64MultiArray()
        gait_mat.layout.dim.append(MultiArrayDimension())
        gait_mat.layout.dim.append(MultiArrayDimension())
        gait_mat.layout.dim[0].label = "height"
        gait_mat.layout.dim[1].label = "width"
        gait_mat.layout.dim[0].size = gait.shape[0]
        gait_mat.layout.dim[1].size = 4
        gait_mat.data = np.ravel(gait, order="C").tolist()
        self._msg.gait = gait_mat

        # Target foosteps
        for k in range(4):
            self._msg.foot_pos[k].name = self._contact_names[k]
            self._msg.foot_pos[k].position.x = foot_pos[0, k]
            self._msg.foot_pos[k].position.y = foot_pos[1, k]
            self._msg.foot_pos[k].position.z = foot_pos[2, k]
        return self._msg

    def writeFromMessage(self, msg):
        """ Extract data from ROS message FootStepStateSL1M.
        """
        i = msg.gait.layout.dim[0].size
        j = msg.gait.layout.dim[1].size
        gait = np.reshape(msg.gait.data, ((i, j)), order="C")

        foot_pos = np.zeros((3, 4))
        for k in range(4):
            c = self._contact_names.index(msg.foot_pos[k].name)
            foot_pos[0, c] = msg.foot_pos[k].position.x
            foot_pos[1, c] = msg.foot_pos[k].position.y
            foot_pos[2, c] = msg.foot_pos[k].position.z

        return gait, foot_pos
