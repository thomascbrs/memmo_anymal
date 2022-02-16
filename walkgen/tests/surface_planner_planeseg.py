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
params.heightmap = path + "lab_scene.dat"
params.urdf = path + "urdf/lab_scene.urdf"

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