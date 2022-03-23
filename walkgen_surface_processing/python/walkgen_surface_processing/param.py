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

import pinocchio as pin
import numpy as np

rom_names = ['anymal_LFleg_rom', 'anymal_RFleg_rom',
             'anymal_LHleg_rom', 'anymal_RHleg_rom']


class SurfaceDetector:
    """ Class to extract convex surfaces from an URDF file.
    """

    def __init__(self, path, margin=0.01, q0=None, initial_height=0.):
        """ Initialize the surface detector.

        Args:
            - path (str): Path of the urdf file.
            - margin (float): Margin in [m] inside the surfaces.
            - initial_height (float): Height of the ground.
            - q0 (array x7): Initial position and orientation in world frame.
            - params (WalkgenParams): Parameter class.
        """

        if q0 is None:
            self._q0 = np.zeros(7)
            self._q0[-1] = 1
        else:
            if len(q0) != 7:
                raise AttributeError(
                    "Initial configuration should be size 7, [position, quaternion]")
            self._q0 = q0

        # Import hpp rbprm here to avoid dependency problems with the module.
        from hpp.corbaserver.affordance.affordance import AffordanceTool
        from hpp.corbaserver.rbprm.tools.surfaces_from_path import getAllSurfacesDict
        from hpp.corbaserver.problem_solver import ProblemSolver
        from hpp.gepetto import ViewerFactory
        from walkgen_surface_planner.tools.geometry_utils import getAllSurfacesDict_inner
        from anymal_rbprm.anymal_abstract import Robot as AnymalAbstract

        self._anymal_abstract = AnymalAbstract()
        self._anymal_abstract.setJointBounds(
            "root_joint", [-5., 5., -5., 5., 0.241, 1.5])
        self._anymal_abstract.boundSO3([-3.14, 3.14, -0.01, 0.01, -0.01, 0.01])
        self._anymal_abstract.setFilter(rom_names)
        for limb in rom_names:
            self._anymal_abstract.setAffordanceFilter(limb, ['Support'])
        self._ps = ProblemSolver(self._anymal_abstract)
        self._vf = ViewerFactory(self._ps)
        self._afftool = AffordanceTool()
        self._afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])

        self._afftool.loadObstacleModel(path, "environment", self._vf)
        self._ps.selectPathValidation("RbprmPathValidation", 0.05)

        # Height matrix is expressed in o frame, the position and orientation of the robot is known.
        # pos = (0,0,height_feet = 0.), quat = (0,0,0,1)
        self._wRo = pin.Quaternion(self._q0[3:]).toRotationMatrix()
        self._wTo = np.zeros(3)
        self._wTo[:2] = self._q0[:2]
        self._wTo[2] = initial_height
        world_pose = np.zeros(7)
        world_pose[:3] = self._wTo[:]
        world_pose[3:] = self._q0[3:]

        # Move the entire environment to follow the initial configuration.
        all_names = self._afftool.getAffRefObstacles("Support")
        for name in all_names:
            self._vf.moveObstacle(name, world_pose.tolist())

        self._all_surfaces = getAllSurfacesDict_inner(
            getAllSurfacesDict(self._afftool), margin=margin)

    def extract_surfaces(self):
        """ Extract surfaces from the URDF file.

        Retruns:
            - param1 (dict): Dictionnary type containing all the surfaces ("unique id" : [vertices]).
        """
        return dict(zip(self._all_surfaces.keys(), [value[0] for value in self._all_surfaces.values()]))
