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
import unittest
from walkgen_surface_planner.SurfacePlanner import SurfacePlanner
from walkgen_surface_planner.params import SurfacePlannerParams

params = SurfacePlannerParams()
params.planeseg = False
# path = os.path.dirname(os.path.abspath(__file__)) + "/../data/"
# params.heightmap = path + "lab_scene.dat"
# params.urdf = path + "urdf/lab_scene.urdf"

# State of the robot
q = np.array([0., 0., 0.4792, 0., 0., 0., 1., -0.1, 0.7, -1., -0.1, -0.7, 1., 0.1, 0.7, -1., 0.1, -0.7, 1.])
# Reference velocity
bvref = np.zeros(6)
bvref[0] = 0.05
bvref[5] = 0.

# Order : [LF, RF, LH, RH]
GAITS = {}
GAITS["walk"] = np.array([[0., 1., 1., 1.], [1., 0., 1., 1.], [1., 1., 0., 1.], [1., 1., 1., 0.]])
GAITS["trot"] = np.array([[1., 0., 1., 0.], [0., 1., 0., 1.]])

params.N_phase = 6
params.typeGait = "walk"
params.com = False
gait_pattern = GAITS[params.typeGait]

surface_planner = SurfacePlanner(params)


class SurfacePlannerTest(unittest.TestCase):

    def test_planner_walk(self):
        # parameters for the test
        params.typeGait = "walk"
        params.com = False
        gait_pattern = GAITS[params.typeGait]

        # update gait parameters
        surface_planner._set_gait_param(params)

        # Run MIP problem.
        selected_surfaces = surface_planner.run(q, gait_pattern, bvref, surface_planner._current_position)
        results = surface_planner.pb_data
        self.assertTrue(results.success)

    def test_planner_walk_com(self):
        # parameters for the test
        params.typeGait = "walk"
        params.com = True
        gait_pattern = GAITS[params.typeGait]

        # update gait parameters
        surface_planner._set_gait_param(params)

        # Run MIP problem.
        selected_surfaces = surface_planner.run(q, gait_pattern, bvref, surface_planner._current_position)
        results = surface_planner.pb_data
        self.assertTrue(results.success)

    def test_planner_trot(self):
        # parameters for the test
        params.typeGait = "trot"
        params.com = False
        gait_pattern = GAITS[params.typeGait]

        # update gait parameters
        surface_planner._set_gait_param(params)

        # Run MIP problem.
        selected_surfaces = surface_planner.run(q, gait_pattern, bvref, surface_planner._current_position)
        results = surface_planner.pb_data
        self.assertTrue(results.success)

    def test_planner_trot_com(self):
        # parameters for the test
        params.typeGait = "trot"
        params.com = False
        gait_pattern = GAITS[params.typeGait]

        # update gait parameters
        surface_planner._set_gait_param(params)

        # Run MIP problem.
        selected_surfaces = surface_planner.run(q, gait_pattern, bvref, surface_planner._current_position)
        results = surface_planner.pb_data
        self.assertTrue(results.success)


if __name__ == '__main__':
    unittest.main()