#!/usr/bin/env python3
#
# Copyright 2021 University of Edinburgh
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
import os
from time import perf_counter as clock
import yaml
import copy

from walkgen.tools.heightmap import load_heightmap
from walkgen.tools.geometry_utils import reduce_surfaces, remove_overlap_surfaces

from sl1m.problem_definition import Problem
from sl1m.generic_solver import solve_MIP

# --------------------------------- PROBLEM DEFINITION ----------------------------------------------------
paths = [
    os.environ["INSTALL_HPP_DIR"] + "/anymal-rbprm/com_inequalities/feet_quasi_flat/anymal_",
    os.environ["INSTALL_HPP_DIR"] + "/anymal-rbprm/relative_effector_positions/anymal_"
]
suffix_com = "_effector_frame_quasi_static_reduced.obj"
limbs = ['LFleg', 'RFleg', 'LHleg', 'RHleg']
others = ['LF_ADAPTER_TO_FOOT', 'RF_ADAPTER_TO_FOOT', 'LH_ADAPTER_TO_FOOT', 'RH_ADAPTER_TO_FOOT']
suffix_feet = "_reduced.obj"
rom_names = ['anymal_LFleg_rom', 'anymal_RFleg_rom', 'anymal_LHleg_rom', 'anymal_RHleg_rom']


class SurfacePlanner():

    def __init__(self, T_gait, n_gait, filename):
        """ Initialize the surface planner.

        Args:
            - filename (str): Path to the config file.
            - T_gait (float): The period of a gait.
            - n_gait (int): Number of phases in one gait.
        """

        self._config = yaml.load(open(filename, 'r'), Loader=yaml.FullLoader)
        self.planeseg = self._config["walkgen_params"]["planeseg"]

        # PARAMETERS FOR SURFACES PROCESSING
        self._margin = self._config["walkgen_params"]["params"]["margin"]
        if self.planeseg:
            self._n_points = self._config["walkgen_params"]["params"]["n_points"]
            self._method_id = self._config["walkgen_params"]["params"]["method_id"]
            self._poly_size = self._config["walkgen_params"]["params"]["poly_size"]
            self._min_area = self._config["walkgen_params"]["params"]["min_area"]

        else:  # Use URDF and heightmap environment
            from hpp.corbaserver.affordance.affordance import AffordanceTool
            from hpp.corbaserver.rbprm.tools.surfaces_from_path import getAllSurfacesDict
            from hpp.corbaserver.problem_solver import ProblemSolver
            from hpp.gepetto import ViewerFactory
            from walkgen.tools.geometry_utils import getAllSurfacesDict_inner
            from anymal_rbprm.anymal_abstract import Robot as AnymalAbstract

            self._path = os.environ["DEVEL_DIR"] + "/memmo_anymal/"  # Temp path
            self._urdf_path = self._path + self._config["walkgen_params"]["world"]["urdf"]
            self._heightmap_path = self._path + self._config["walkgen_params"]["world"]["heightmap"]
            self.heightmap = load_heightmap(self._heightmap_path)  # Heightmap

            self._anymal_abstract = AnymalAbstract()
            self._anymal_abstract.setJointBounds("root_joint", [-5., 5., -5., 5., 0.241, 1.5])
            self._anymal_abstract.boundSO3([-3.14, 3.14, -0.01, 0.01, -0.01, 0.01])
            self._anymal_abstract.setFilter(rom_names)
            for limb in rom_names:
                self._anymal_abstract.setAffordanceFilter(limb, ['Support'])
            self._ps = ProblemSolver(self._anymal_abstract)
            self._vf = ViewerFactory(self._ps)
            self._afftool = AffordanceTool()
            self._afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])

            self._afftool.loadObstacleModel(self._urdf_path, "environment", self._vf)
            self._ps.selectPathValidation("RbprmPathValidation", 0.05)

            self._all_surfaces = getAllSurfacesDict_inner(getAllSurfacesDict(self._afftool), margin=self._margin)

        # SL1M Problem initialization
        self.pb = Problem(limb_names=limbs,
                          other_names=others,
                          constraint_paths=paths,
                          suffix_com=suffix_com,
                          suffix_feet=suffix_feet)

        # SL1M parameters
        self._T_gait = T_gait  # Period of a gait.
        self._n_gait = n_gait  # Number of phases in a gait.
        self._step_duration = self._T_gait / n_gait
        self._N_phase = self._config["walkgen_params"]["params"]["N_phase"]
        self._N_phase_return = self._config["walkgen_params"]["params"]["N_phase_return"]
        if self._N_phase_return > self._N_phase :
            raise ArithmeticError("Cannot return more surfaces than step done in SL1M")
        self._N_total = self._n_gait * self._N_phase
        self._com = self._config["walkgen_params"]["params"]["com"]

        self._contact_names = ['LF_FOOT', 'RF_FOOT', 'LH_FOOT', 'RH_FOOT']  # Order of the feet in the surface planner.
        # Shoulder position in base frame
        self._shoulders = np.array([[0.37, 0.37, -0.37, -0.37], [0.2, -0.2, 0.2, -0.2], [0., 0., 0., 0.]])
        self._current_position = np.array([[0.37, 0.37, -0.37, -0.37], [0.2, -0.2, 0.2, -0.2], [0., 0., 0., 0.]])
        self._reference_height = 0.4792

        # Initial selected surface
        A = [[-1., 0., 0.], [0., -1., 0.], [0., 1., 0.], [1., 0., 0.], [0., 0., 1.], [-0., -0., -1.]]
        b = [1., 1., 1., 1., 0., 0.]
        vertices = [[-1., 1., 0.], [-1., -1., 0.], [1., -1., 0.], [1., 1., 0.]]
        self._init_surface = Surface(np.array(A), np.array(b), np.array(vertices).T)

        # Add initial surface to the result structure.
        self._selected_surfaces = dict()
        # Dictionary type :
        # For each foot is associated a list of surfaces, one foot is moving one time for each gait/phase
        for foot in range(4):
            L = []
            for k in range(self._N_phase_return):
                L.append(copy.deepcopy(self._init_surface))
            self._selected_surfaces[self._contact_names[foot]] = L

        # Debug and plot purpose
        self.pb_data = None
        self.surfaces_processed = None

    def _compute_gait(self, gait_in):
        """
        Get a gait matrix with only one line per phase
        :param gait_in: gait matrix with several line per phase
        :return: gait matrix
        """
        gait = []

        for i in range(0, gait_in.shape[0]):
            if np.any(1-gait_in[i,:]): # no [1,1,1,1]
                if not gait_in[i,:].tolist() in gait :
                    gait.append(gait_in[i,:].tolist())
        gait = np.array(gait)
        gait = np.roll(gait, -2, axis=0)

        return gait

    def _compute_effector_positions(self, configs, bvref):
        """Compute the desired effector positions with an approximation
        of the Raibert's heuristic.

        Args:
         - configs (list): List of configurations (Array x7, [position, orientation]).
         - bvref (array x6): The desired velocity in base frame.
        """
        t_stance = self._T_gait / self._n_gait
        effector_positions = np.zeros((4, self.pb.n_phases, 2))

        for phase in self.pb.phaseData:
            for foot in phase.moving:
                rpy = pin.rpy.matrixToRpy(pin.Quaternion(configs[phase.id][3:7]).toRotationMatrix())
                yaw = rpy[2]  # Get yaw for the predicted configuration
                shoulders = np.zeros(2)
                # Compute heuristic position in horizontal frame
                rpy[2] = 0.  # Yaw = 0. in horizontal frame
                Rp = pin.rpy.rpyToMatrix(rpy)[:2, :2]
                heuristic = 0.5 * t_stance * Rp @ bvref[:2] + Rp @ self._shoulders[:2, foot]

                # Compute heuristic in world frame, rotation
                shoulders[0] = heuristic[0] * np.cos(yaw) - heuristic[1] * np.sin(yaw)
                shoulders[1] = heuristic[0] * np.sin(yaw) + heuristic[1] * np.cos(yaw)
                effector_positions[foot][phase.id] = np.array(configs[phase.id][:2] + shoulders)

        return effector_positions

    def _compute_shoulder_positions(self, configs):
        """Compute the shoulder positions.

        Args:
            - configs (list): List of configurations (Array x7, [position, orientation]).
        """
        shoulder_positions = np.zeros((4, self.pb.n_phases, 3))

        for phase in self.pb.phaseData:
            for foot in phase.moving:
                R = pin.Quaternion(configs[phase.id][3:7]).toRotationMatrix()
                shoulder_positions[foot][phase.id] = R @ self._shoulders[:, foot] + configs[phase.id][:3]

        return shoulder_positions

    def _compute_configuration(self, q, bvref):
        """ Compute configuration for the next phases.

        Args :
            - q (array x7): Cureent state [pos x3 , quaternion x4]
            - bvref (array x6): The desired velocity in base frame.
        """
        if len(q) != 7:
            raise ArithmeticError("State q should be size 7.")
        if len(bvref) != 6:
            raise ArithmeticError("Reference velocity should be size 6.")

        yaw_init = pin.rpy.matrixToRpy(pin.Quaternion(q[3:7]).toRotationMatrix())[2]
        # List of configurations in planned horizon, using the reference velocity.
        configs = []

        rpyMap_, fit_ = np.zeros(3), np.zeros(3)
        if not self.planeseg:
            fit_ = self.heightmap.fit_surface(q[0], q[1])
            rpyMap_[0] = -np.arctan2(fit_[1], 1.)
            rpyMap_[1] = -np.arctan2(fit_[0], 1.)

        for i in range(self._N_total):
            config = np.zeros(7)
            dt_config = self._step_duration * (i + 1)  # Delay of 1 phase of contact for MIP

            if bvref[5] >= 0.001:
                config[0] = (bvref[0] * np.sin(bvref[5] * dt_config) + bvref[1] *
                             (np.cos(bvref[5] * dt_config) - 1.0)) / bvref[5]
                config[1] = (bvref[1] * np.sin(bvref[5] * dt_config) - bvref[0] *
                             (np.cos(bvref[5] * dt_config) - 1.0)) / bvref[5]
            else:
                config[0] = bvref[0] * dt_config
                config[1] = bvref[1] * dt_config

            config[0] = np.cos(yaw_init) * config[0] - np.sin(yaw_init) * config[1]  # Yaw rotation for dx
            config[1] = np.sin(yaw_init) * config[0] + np.cos(yaw_init) * config[1]  # Yaw rotation for dy
            config[:2] += q[:2]  # Add initial 2D position

            # Recompute the orientation according to the heightmap for each configuration.
            # if not self.planeseg:
            #     fit_ = self.heightmap.fit_surface(config[0], config[1])
            #     rpyMap_[0] = -np.arctan2(fit_[1], 1.)
            #     rpyMap_[1] = -np.arctan2(fit_[0], 1.)

            config[2] = fit_[0] * config[0] + fit_[1] * config[1] + fit_[2] + self._reference_height
            yaw = yaw_init + bvref[5] * dt_config
            roll = rpyMap_[0] * np.cos(yaw) - rpyMap_[1] * np.sin(yaw)
            pitch = rpyMap_[0] * np.sin(yaw) + rpyMap_[1] * np.cos(yaw)
            config[3:] = pin.Quaternion(pin.rpy.rpyToMatrix(roll, pitch, yaw)).coeffs()
            configs.append(config)

        return configs

    def _compute_com_positions(self, configs):
        """ Compute the com positions

        Args:
            - configs (list): List of configurations (Array x7, [position, orientation]).
        """
        com_positions = []
        for phase in self.pb.phaseData:
            com_positions.append(configs[phase.id][:3])

        return com_positions

    def _get_potential_surfaces(self, configs, gait, all_surfaces):
        """ Get the rotation matrix and surface condidates for each configuration in configs.
        Without a guide path, gives all surfaces as potential surfaces for each feet.

        Args:
         - configs (list): List of configurations (Array x7, [position, orientation]).
         - gait (array nx4): gait matrix.
         - all_surfaces (list): List of the surface reshaped, defined using the vertices positions:
                                array([[x0, x1, ... , xn],
                                       [y0, y1, ... , yn],
                                       [z0, z1, ... , zn]]).
        """
        surfaces_list = []
        empty_list = False
        for id, config in enumerate(configs):
            foot_surfaces = []
            stance_feet = np.nonzero(gait[id % len(gait)] == 1)[0]
            previous_swing_feet = np.nonzero(gait[(id - 1) % len(gait)] == 0)[0]
            moving_feet = stance_feet[np.in1d(stance_feet, previous_swing_feet, assume_unique=True)]

            for elt in moving_feet:
                foot_surfaces.append(all_surfaces)
            surfaces_list.append(foot_surfaces)

        return surfaces_list, empty_list

    def _get_potential_surfaces_RBPRM(self, configs, gait):
        """ Get the rotation matrix and surface condidates for each configuration in configs using
        the guide path RBPRM.

        Args:
         - configs (list): List of configurations (Array x7, [position, orientation]).
         - gait (array nx4): gait matrix.
        """
        surfaces_list = []
        empty_list = False
        for id, config in enumerate(configs):
            stance_feet = np.nonzero(gait[id % len(gait)] == 1)[0]
            previous_swing_feet = np.nonzero(gait[(id - 1) % len(gait)] == 0)[0]
            moving_feet = stance_feet[np.in1d(stance_feet, previous_swing_feet, assume_unique=True)]
            roms = np.array(rom_names)[moving_feet]

            foot_surfaces = []
            for rom in roms:
                surfaces = []
                surfaces_names = self._anymal_abstract.clientRbprm.rbprm.getCollidingObstacleAtConfig(
                    config.tolist(), rom)
                for name in surfaces_names:
                    surfaces.append(self._all_surfaces[name][0])

                if not len(surfaces_names):
                    empty_list = True

                # Sort and then convert to array
                surfaces = sorted(surfaces)
                surfaces_array = []
                for surface in surfaces:
                    surfaces_array.append(np.array(surface).T)

                # Add to surfaces list
                foot_surfaces.append(surfaces_array)
            surfaces_list.append(foot_surfaces)

        return surfaces_list, empty_list

    def _retrieve_surfaces(self, surfaces, indices):
        """ Update the structure containing the surfaces selected for each foot.

        Args:
            -  surfaces (list): The list of potential surfaces. The surfaces are defined using the vertices positions:
                                array([[x0, x1, ... , xn],
                                        [y0, y1, ... , yn],
                                        [z0, z1, ... , zn]])
            - indices (list): For each step in the horizon, list of the selected surfaces.
        """
        for k in range(self._N_phase_return):  # in one phase each foot move one time.
            for i in range(self._n_gait): # The number of step in one phase
                for id,foot in enumerate(self.pb.phaseData[k*self._n_gait + i].moving):
                    id_sf = indices[k*self._n_gait + i][id] # Index of surface choosen in the potential surfaces.
                    self._selected_surfaces.get(self._contact_names[foot])[k].A = self.pb.phaseData[k*self._n_gait + i].S[id][id_sf][0][:,:]
                    self._selected_surfaces.get(self._contact_names[foot])[k].b = self.pb.phaseData[k*self._n_gait + i].S[id][id_sf][1][:]
                    self._selected_surfaces.get(self._contact_names[foot])[k].vertices = surfaces[k*self._n_gait + i][id][id_sf][:,:]

    def run(self, q, gait_in, bvref, target_foostep, array_markers=None):
        """ Select the nex surfaces to use.

        Args:
            - q (array x19): Current state [pos x3 , quaternion x4, joint pos x12].
            - gait (Array n_gait x 4): Next walking gait.
            - bvref (Array x3): Reference velocity.
            - target_foostep (Array 3x4): The next foot step position. (For the next phase of contact)
            - array_markers (ArrayMarker): MarkerArray from visualization_msgs.msg. (list of surfaces)

        Return:
            - param1 (dict): Next surfaces for each foot.
        """
        if len(q) != 19:
            raise ArithmeticError("Current state should be size 19, [pos x3 , quaternion x4, joint pos x12]")
        if array_markers == None and self.planeseg == True:
            raise ArithmeticError("No surfaces provided, SL1M running without the env URDF.")

        t0 = clock()

        configs = self._compute_configuration(q[:7], bvref)  # Compute configurations.

        R = [pin.XYZQUATToSE3(np.array(config)).rotation for config in configs]  # Orientation for each configurations.

        gait = self._compute_gait(gait_in) # remove redundancies + roll the matrix of 1.

        # Initial contact at the beginning of the next phase.
        initial_contacts = [np.array(target_foostep[:, i].tolist()) for i in range(4)]

        if self.planeseg:
            # Reduce and sort incoming data
            surfaces_reduced = reduce_surfaces(array_markers, margin=self._margin, n_points=self._n_points)

            # Apply proccess to filter and decompose the surfaces to avoid overlap
            self.surfaces_processed = remove_overlap_surfaces(surfaces_reduced,
                                                              polySize=self._poly_size,
                                                              method=self._method_id,
                                                              min_area=self._min_area,
                                                              initial_floor=self._init_surface.vertices)
            surfaces, empty_list = self._get_potential_surfaces(configs, gait, self.surfaces_processed)

        else:
            surfaces, empty_list = self._get_potential_surfaces_RBPRM(configs, gait)

        self.pb.generate_problem(R, surfaces, gait, initial_contacts, configs[0][:3], com=self._com)

        if empty_list:
            raise ArithmeticError("One step has no pentential surface to use")

        # Compute the costs
        effector_positions = self._compute_effector_positions(configs, bvref)
        com_positions = self._compute_com_positions(configs)

        costs = {
            "effector_positions": [1.0, effector_positions],
            "coms_xy": [0.5, com_positions],
            "coms_z": [0.05, com_positions]
        }

        # Solve MIP
        self.pb_data = solve_MIP(self.pb, costs=costs, com=self._com)

        # Process result SL1M
        if self.pb_data.success:
            surface_indices = self.pb_data.surface_indices

            self._retrieve_surfaces(surfaces, surface_indices)
            return self._selected_surfaces

        else:
            # TODO : Return a list of potential surfaces, so that the footsteplanner chooses
            # the next surface accordingonly to the Raibert's heuristic.
            print("The MIP problem did NOT converge")
            return self._selected_surfaces


class Surface():

    def __init__(self, A, b, vertices):
        """Initialize the surface.

        Args :
        - A (array nx3): Inequality matrix.
        - b (array x n): Inequality vector.
        - vertices (array 3 x n): Vertices with the format:
                                array([[x0, x1, ... , xn],
                                       [y0, y1, ... , yn],
                                       [z0, z1, ... , zn]])
        """
        if A.shape[1] != 3:
            raise ArithmeticError("Number column of the inequality array should be 3.")
        if vertices.shape[0] != 3:
            raise ArithmeticError("Number of rows of the vertice array should be 3.")

        self.A = A
        self.b = b
        self.vertices = vertices


if __name__ == "__main__":
    """ Run a simple example of SurfacePlanner.
    """
    import pickle
    # Load Marker array class example from ROS simulation
    fileObject = os.getcwd() + "/data/example_marker_array.pickle"

    with open(fileObject, 'rb') as file2:
        array_markers = pickle.load(file2)

    filepath = os.path.dirname(os.path.abspath(__file__)) + "/config/params.yaml"
    surface_planner = SurfacePlanner(0.6, 2, filepath)

    q = np.array([0., 0., 0.4792, 0., 0., 0., 1., -0.1, 0.7, -1., -0.1, -0.7, 1., 0.1, 0.7, -1., 0.1, -0.7, 1.])
    q[0] = 0.
    bvref = np.zeros(6)
    bvref[0] = 0.3
    bvref[5] = 0.1
    # gait_in = np.array([[1, 0, 0, 1], [0, 1, 1, 0]])
    gait_in = np.array([[0, 1, 1, 1], [1, 0, 1, 1], [1,1,0,1], [1,1,1,0]])

    selected_surfaces = surface_planner.run(q, gait_in, bvref, surface_planner._current_position, array_markers)

    import matplotlib.pyplot as plt
    import sl1m.tools.plot_tools as plot

    if not surface_planner.planeseg :
        ax = plot.draw_whole_scene(surface_planner._all_surfaces)
        plot.plot_planner_result(surface_planner.pb_data.all_feet_pos, coms=surface_planner.pb_data.coms, ax=ax, show=True)

    else:
        from walkgen.tools.plot_tools import plot_marker_surface
        # Plot initial surfaces from markerArray
        fig = plt.figure(figsize=(10, 6))
        ax = plt.axes(projection='3d')
        plt.title("Initial surfaces")
        plot_marker_surface(array_markers,ax)

        # Plot SL1M results
        fig = plt.figure(figsize=(10, 6))
        ax = plt.axes(projection='3d')
        plt.title("SL1M result")
        for sf in surface_planner.surfaces_processed :
            plot.plot_surface(sf,ax=ax)
        plot.plot_planner_result(surface_planner.pb_data.all_feet_pos, coms=surface_planner.pb_data.coms, ax=ax, show=True)
