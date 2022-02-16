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

import os
import sys
import pickle
import numpy as np
import unittest
from walkgen.SurfacePlanner import SurfacePlanner
from walkgen.params import WalkgenParams
from walkgen.tools.geometry_utils import reduce_surfaces, remove_overlap_surfaces

# Load Marker array class example extracted from rosbag.
path = os.path.dirname(os.path.abspath(__file__)) + "/../data/"
fileObject = path + "example_marker_array.pickle"
with open(fileObject, 'rb') as file2:
    data = pickle.load(file2)

params = WalkgenParams()
params.planeseg = True

# State of the robot
q = np.array([0., 0., 0.4792, 0., 0., 0., 1., -0.1, 0.7, -1., -0.1, -0.7, 1., 0.1, 0.7, -1., 0.1, -0.7, 1.])
# Reference velocity
bvref = np.zeros(6)
bvref[0] = 0.5
bvref[5] = 0.

# Order : [LF, RF, LH, RH]
GAITS = {}
GAITS["Walk"] = np.array([[0., 1., 1., 1.], [1., 0., 1., 1.], [1., 1., 0., 1.], [1., 1., 1., 0.]])
GAITS["Trot"] = np.array([[1., 0., 1., 0.], [0., 1., 0., 1.]])

params.N_phase = 4
params.typeGait = "Walk"
params.com = False
gait_pattern = GAITS[params.typeGait]

surface_planner = SurfacePlanner(params)


class SurfacePlannerTest(unittest.TestCase):

    def test_planner_walk(self):
        # parameters for the test
        params.typeGait = "Walk"
        params.com = False
        gait_pattern = GAITS[params.typeGait]

        # update gait parameters
        surface_planner._set_gait_param(params)

        # Reduce and sort incoming data
        surfaces_reduced = reduce_surfaces(data, margin=params.margin, n_points=params.n_points)

        # Apply proccess to filter and decompose the surfaces to avoid overlap
        set_surfaces = remove_overlap_surfaces(surfaces_reduced,
                                               polySize=params.poly_size,
                                               method=params.method_id,
                                               min_area=params.min_area,
                                               initial_floor=surface_planner._init_surface.vertices)

        # Run MIP problem.
        selected_surfaces = surface_planner.run(q, gait_pattern, bvref, surface_planner._current_position,
                                                set_surfaces)
        results = surface_planner.pb_data
        self.assertTrue(results.success)

    def test_planner_walk_com(self):
        # parameters for the test
        params.typeGait = "Walk"
        params.com = True
        gait_pattern = GAITS[params.typeGait]

        # update gait parameters
        surface_planner._set_gait_param(params)

        # Reduce and sort incoming data
        surfaces_reduced = reduce_surfaces(data, margin=params.margin, n_points=params.n_points)

        # Apply proccess to filter and decompose the surfaces to avoid overlap
        set_surfaces = remove_overlap_surfaces(surfaces_reduced,
                                               polySize=params.poly_size,
                                               method=params.method_id,
                                               min_area=params.min_area,
                                               initial_floor=surface_planner._init_surface.vertices)

        # Run MIP problem.
        selected_surfaces = surface_planner.run(q, gait_pattern, bvref, surface_planner._current_position,
                                                set_surfaces)
        results = surface_planner.pb_data
        self.assertTrue(results.success)

    def test_planner_trot(self):
        # parameters for the test
        params.typeGait = "Trot"
        params.com = False
        gait_pattern = GAITS[params.typeGait]

        # update gait parameters
        surface_planner._set_gait_param(params)

        # Reduce and sort incoming data
        surfaces_reduced = reduce_surfaces(data, margin=params.margin, n_points=params.n_points)

        # Apply proccess to filter and decompose the surfaces to avoid overlap
        set_surfaces = remove_overlap_surfaces(surfaces_reduced,
                                               polySize=params.poly_size,
                                               method=params.method_id,
                                               min_area=params.min_area,
                                               initial_floor=surface_planner._init_surface.vertices)

        # Run MIP problem.
        selected_surfaces = surface_planner.run(q, gait_pattern, bvref, surface_planner._current_position,
                                                set_surfaces)
        results = surface_planner.pb_data
        self.assertTrue(results.success)

    def test_planner_trot_com(self):
        # parameters for the test
        params.typeGait = "Trot"
        params.com = True
        gait_pattern = GAITS[params.typeGait]

        # update gait parameters
        surface_planner._set_gait_param(params)

        # Reduce and sort incoming data
        surfaces_reduced = reduce_surfaces(data, margin=params.margin, n_points=params.n_points)

        # Apply proccess to filter and decompose the surfaces to avoid overlap
        set_surfaces = remove_overlap_surfaces(surfaces_reduced,
                                               polySize=params.poly_size,
                                               method=params.method_id,
                                               min_area=params.min_area,
                                               initial_floor=surface_planner._init_surface.vertices)

        # Run MIP problem.
        selected_surfaces = surface_planner.run(q, gait_pattern, bvref, surface_planner._current_position,
                                                set_surfaces)
        results = surface_planner.pb_data
        self.assertTrue(results.success)


if __name__ == '__main__':
    unittest.main()