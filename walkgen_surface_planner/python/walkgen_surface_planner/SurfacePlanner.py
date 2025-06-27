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
import hppfcl
import numpy as np
import os
try:
    from time import perf_counter as clock
except ImportError:
    from time import time as clock
import copy
import warnings
import trimesh

from walkgen_surface_planner.tools.Surface import Surface
from walkgen_surface_planner.params import SurfacePlannerParams
from walkgen_surface_planner.tools.collisions_utils import convert_to_convexFcl, distance
from .libwalkgen_surface_planner_pywrap import TerrainSlope, StdVec_MatrixXd

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
rom_names = ['LFleg_vN_Rom.stl', 'RFleg_vN_Rom.stl', 'LHleg_vN_Rom.stl', 'RHleg_vN_Rom.stl']


class SurfacePlanner():
    def __init__(self, params=None, RECORDING=False):
        """ Initialize the surface planner.

        Args:
            - initial_height (float): Height of the ground.
            - q0 (array x7): Initial position and orientation in world frame.
            - params (WalkgenParams): Parameter class.
            - RECORDING (bool): records computing timings in a dict().
        """
        if params is not None:
            self._params = copy.deepcopy(params)
        else:
            self._params = SurfacePlannerParams()

        # SL1M Problem initialization
        self.pb = Problem(limb_names=limbs,
                          other_names=others,
                          constraint_paths=paths,
                          suffix_com=suffix_com,
                          suffix_feet=suffix_feet)

        # Slope of the terrain given a set of convex surfaces.
        self._box = hppfcl.Box(np.array([4., 2., 4]))  # Reduce number of surfaces for the terrain evaluation.
        self._tf = hppfcl.Transform3f()
        self._terrain = TerrainSlope(self._params.fitsize_x, self._params.fitsize_y, self._params.fitlength)
        self._recompute_slope = self._params.recompute_slope

        # Usual order LF LH RF RH
        # SL1M order : LF RF LH RH
        self._contact_names = []
        self._contact_names.append(self._params.contact_names[0])
        self._contact_names.append(self._params.contact_names[2])
        self._contact_names.append(self._params.contact_names[1])
        self._contact_names.append(self._params.contact_names[3])
        offsets = self._params.shoulder_offsets  # In usual order.
        self._shoulders = np.zeros((3, 4))
        self._shoulders[:2, 0] = offsets[0]
        self._shoulders[:2, 1] = offsets[2]
        self._shoulders[:2, 2] = offsets[1]
        self._shoulders[:2, 3] = offsets[3]
        # self._shoulders = np.array([[0.37, 0.37, -0.37, -0.37], [0.2, -0.2, 0.2, -0.2], [0., 0., 0., 0.]]) # Base frame
        self._reference_height = 0.4792

        # Load rom .stl objects for collision tools to select only the relevant surfaces.
        path = os.environ["INSTALL_HPP_DIR"] + "/anymal-rbprm/meshes/"
        # obj_stl = [trimesh.load_mesh(path + rom) for rom in rom_names]
        obj_stl = []
        for i, rom in enumerate(rom_names):
            obj = trimesh.load_mesh(path + rom)
            obj.apply_translation(-self._shoulders[:, i])
            obj.apply_scale(1.2)
            obj.apply_translation(self._shoulders[:, i])
            obj_stl.append(obj)

        # Dictionnary containing the convex set of roms for collisions.
        self.roms_collision = dict(zip(self._contact_names, [convert_to_convexFcl(obj.vertices) for obj in obj_stl]))

        # Planner parameters.
        self.HORIZON = self._params.horizon  # Number of fsteps optimised.
        self._com = self._params.com  # CoM taken into account in the formulation.
        self._ratio_recompute_slope = 3  # Compute the slope of terrain 1 configuration over self._ratio_recompute_slope
        self._N_phase_return = self._params.N_phase_return  # Number of surfaces returned for each foot.
        self._step_duration = 0.  # Average duration of a step for visualization purpose.
        self._RECORDING = RECORDING  # Record computing timings

        # Store data.
        self.all_surfaces = None
        self.all_surfaces_collision = None
        self._selected_surfaces = dict()
        self.surfaces_processed = None
        self.configs = None
        self.pb_data = None  # Debug and plot purpose

        # Profiling performances
        if self._RECORDING:
            self.profiler = {"potential_number": [], "timing_potential": 0, "timing_MIP": 0, "timing_configuration": 0}
        else:
            self.profiler = {}

    def get_profiler(self):
        return self.profiler

    def _compute_gait(self, gait_in):
        """
        Remove phases with no foot moving. [1,1,1,1]
        :param gait_in: gait matrix with several line per phase
        :return: gait matrix
        """
        gait = []
        for i in range(0, gait_in.shape[0]):
            if np.any(1 - gait_in[i, :]):  # no [1,1,1,1]
                gait.append(gait_in[i, :].tolist())
        gait = np.array(gait)

        return gait

    def _compute_shoulder_positions(self, configs):
        """Compute the shoulder positions, keep heuristic for back foot.
        TODO: Maybe add a way to penalize only some feet in SL1M.

        Args:
            - configs (list): List of configurations (Array x7, [position, orientation]).
        """
        shoulder_positions = np.zeros((4, self.pb.n_phases, 3))

        for phase in self.pb.phaseData:
            for foot in phase.moving:
                R = pin.Quaternion(configs[phase.id][3:7]).toRotationMatrix()
                sh = np.dot(R, self._shoulders[:, foot]) + configs[phase.id][:3]
                shoulder_positions[foot][phase.id] = sh
        return shoulder_positions

    def _compute_configuration(self, q, bvref, gait, timings):
        """ Compute configuration for the next phases and the 2D heuristics.

        Args :
            - q (array x7): Cureent state [pos x3 , quaternion x4]
            - bvref (array x6): The desired velocity in base frame.
            - gait (array nx4) : The next movings feet.
            - timings (list) : For each row in the gait matrix, time spent in this configuration.

        Returns:
            -  (list) : configurations
            - (array) : (4 x len(configs) x 2) End-effector heuristic.
        """
        if len(q) != 7:
            raise ArithmeticError("State q should be size 7.")
        if len(bvref) != 6:
            raise ArithmeticError("Reference velocity should be size 6.")

        # Compute effector positions at the same time
        effector_positions = [[], [], [], []]  # 4 feet, Nb phase, 2D position

        yaw_init = pin.rpy.matrixToRpy(pin.Quaternion(q[3:7]).toRotationMatrix())[2]

        # Select the relevant convex surfaces to approximate the slope of the terrain.
        self._tf.setTranslation(q[:3])
        rotation = pin.rpy.rpyToMatrix(np.array([0., 0., yaw_init]))
        collision_points = StdVec_MatrixXd()
        for key, surface_collision in self.all_surfaces_collision.items():
            if distance(surface_collision, self._box, hppfcl.Transform3f(), self._tf) < 0:
                collision_points.append(np.array(self.all_surfaces[key]))
        # Get the slope of the terrain.
        fit_ = self._terrain.get_slope(q[:3], rotation, collision_points)
        rpyMap_ = np.zeros(3)
        rpyMap_[0] = np.arctan2(fit_[1], 1.)
        rpyMap_[1] = -np.arctan2(fit_[0], 1.)

        #   GAIT         TIMELINE
        # [1,0,1,1]         0         --> Current phase. Feet no1 START to move.
        # [0,1,1,1]        0.70         --> Delay for SL1M. Initial phase. 700ms from row 0 to row 1
        # [1,1,1,0]        0.70         --> 70ms from row 1 to row 2
        # [1,1,1,1]        0.70         --> Not considered as a phaseData since no moving feet.
        # [1,1,0,1]        0.90
        # [1,0,1,1]        0.70

        # print("Gait config : ", gait)
        # print("timing : ", timings)

        # List of configurations in planned horizon, using the reference velocity.
        configs = []

        horizonTmp = 0  # Total number of optimised variables.
        i = 0
        index_gait = 1  # Delay. Start at row 1.
        dt_config = timings[1]  # Cumulative time over timings.

        # Compute the slope of terrain 1 configuration over self._ratio_recompute_slope
        while horizonTmp < self.HORIZON:
            config = np.zeros(7)

            dt_config += timings[index_gait + 1]
            while np.sum(gait[index_gait, :] == 1) == 4:  # row = [1,1,1,1]
                index_gait += 1
                # If taking into account the stance phase.
                # we can obtain error during initialisation since the stance phase is long.
                dt_config += timings[index_gait + 1]

            # Compute the number of optimised variables:
            horizonTmp += np.sum(gait[index_gait, :] == 0)
            # print("horizonTmp : ", horizonTmp)

            # WARNING HERE
            # The Raibert heuristic takes into account the position of the shoulder (or center of the base)
            # when the foot hits the ground. Hence, dt_config = time to arrived here + flying phase.
            if abs(bvref[5]) >= 0.01:
                dx_ = (bvref[0] * np.sin(bvref[5] * dt_config) + bvref[1] *
                       (np.cos(bvref[5] * dt_config) - 1.0)) / bvref[5]
                dy_ = (bvref[1] * np.sin(bvref[5] * dt_config) - bvref[0] *
                       (np.cos(bvref[5] * dt_config) - 1.0)) / bvref[5]
            else:
                dx_ = bvref[0] * dt_config
                dy_ = bvref[1] * dt_config

            config[0] = np.cos(yaw_init) * dx_ - np.sin(yaw_init) * dy_  # Yaw rotation for dx
            config[1] = np.sin(yaw_init) * dx_ + np.cos(yaw_init) * dy_  # Yaw rotation for dy
            config[:2] += q[:2]  # Add initial 2D position

            # Recompute the orientation according to the heightmap each configuration over self._ratio_recompute_slope
            if self._recompute_slope and i % self._ratio_recompute_slope == 0:
                rotation = np.dot(pin.rpy.rpyToMatrix(np.array([0., 0., bvref[5] * dt_config])), rotation)
                fit_ = self._terrain.get_slope(config[:2], rotation, collision_points)
                rpyMap_ = np.zeros(3)
                rpyMap_[0] = np.arctan2(fit_[1], 1.)
                rpyMap_[1] = -np.arctan2(fit_[0], 1.)


            config[2] = fit_[0] * config[0] + fit_[1] * \
                config[1] + fit_[2] + self._reference_height
            yaw = yaw_init + bvref[5] * dt_config
            roll = rpyMap_[0] * np.cos(bvref[5] * dt_config) - rpyMap_[1] * np.sin(bvref[5] * dt_config)
            pitch = rpyMap_[0] * np.sin(bvref[5] * dt_config) + rpyMap_[1] * np.cos(bvref[5] * dt_config)

            Rp = pin.rpy.rpyToMatrix(np.array([roll, pitch, 0.]))
            Ryaw = pin.rpy.rpyToMatrix(np.array([0., 0., yaw]))
            config[3:] = pin.Quaternion(np.dot(Rp, Ryaw)).coeffs()

            for foot_id in range(4):
                if gait[index_gait, foot_id] == 0:
                    t_stance = self.get_stance(gait, timings, foot_id, index_gait)
                    rpy = pin.rpy.matrixToRpy(pin.Quaternion(config[3:7]).toRotationMatrix())
                    yaw = rpy[2]  # Get yaw for the predicted configuration
                    foot_pos = np.zeros(2)
                    # Compute heuristic position in horizontal frame
                    rpy[2] = 0.  # Yaw = 0. in horizontal frame
                    Rp = pin.rpy.rpyToMatrix(rpy)[:2, :2]
                    heuristic = 1.35 * t_stance * \
                        np.dot(Rp, bvref[:2]) + np.dot(Rp, self._shoulders[:2, foot_id])

                    # Compute heuristic in world frame, rotation
                    foot_pos[0] = heuristic[0] * \
                        np.cos(yaw) - heuristic[1] * np.sin(yaw)
                    foot_pos[1] = heuristic[0] * \
                        np.sin(yaw) + heuristic[1] * np.cos(yaw)
                    effector_positions[foot_id].append(np.array(config[:2] + foot_pos))
                else:
                    effector_positions[foot_id].append(np.zeros(2))

            configs.append(config)

            i += 1
            index_gait += 1

        return configs, np.array(effector_positions)

    def get_stance(self, gait, timings, foot, index):
        if gait[index, foot] != 0:
            raise AttributeError("Evaluation of stance phase start with 0")
        id = copy.copy(index)  # to not modify index
        t_stance = 0
        # Find next index in the gait where the foot is on the ground
        while gait[id % gait.shape[0], foot] != 1:
            id += 1
        # Find the next index in the gait where the foot is flying
        # Foot id = 1. Index = 0. in means the timings counts for the t_stance
        #   GAIT         TIMELINE
        # [1,0,1,1]         0     -
        # [0,1,1,1]        0.70   -
        # [1,1,1,0]        0.70   in
        # [1,1,0,1]        0.70   in
        # [1,1,1,1]        0.90   in
        # [1,0,1,1]        0.70   in
        # [0,1,1,1]        0.70   -
        while gait[id % gait.shape[0], foot] != 0:
            # if np.any(1 - gait[id % gait.shape[0], :]): # not taking into account [1,1,1,1]
            t_stance += timings[(id + 1) % gait.shape[0]]
            id += 1
        return t_stance

    def _compute_com_positions(self, configs):
        """ Compute the com positions

        Args:
            - configs (list): List of configurations (Array x7, [position, orientation]).
        """
        com_positions = []
        for phase in self.pb.phaseData:
            com_positions.append(configs[phase.id][:3])

        return com_positions

    def _compute_sl1m_position(self, pb_data):
        """ Get the position of the feet from the previous optimisation problem.

        Args:
            - pb_data : Sl1m structure containing the results of the previous optimisation
        """
        previous_position = pb_data.all_feet_pos
        for foot in range(4):
            previous_position[foot].pop(0)
            previous_position.append(None)
        return previous_position

    def get_potential_surfaces(self, configs, gait):
        """ Get the rotation matrix and surface condidates for each configuration in configs using
        the collision tool hppfcl.

        Args:
            - configs (list): List of configurations (Array x7, [position, orientation]).
            - gait (array nx4): gait matrix without static phases. (same as sl1m).
        """
        if self._RECORDING:
            # Reset the list of potential surfaces
            self.profiler["potential_number"] = []
        surfaces_list = []
        empty_list = False
        for id, config in enumerate(configs):
            tf = hppfcl.Transform3f()
            tf.setTranslation(config[:3])
            tf.setRotation(pin.Quaternion(config[3:]).toRotationMatrix())

            # Previous SL1M method to compute the moving feet.
            # stance_feet = np.nonzero(gait[id % len(gait)] == 1)[0]
            # previous_swing_feet = np.nonzero(gait[(id - 1) % len(gait)] == 0)[0]
            # moving_feet = stance_feet[np.in1d(stance_feet, previous_swing_feet, assume_unique=True)]

            # New method to compute the moving feet.
            previous_stance = np.nonzero(gait[id % len(gait)] == 1)[0]
            next_moving = np.nonzero(gait[(id + 1) % len(gait)] == 0)[0]
            moving_feet = previous_stance[np.in1d(previous_stance, next_moving, assume_unique=True)]

            foot_surfaces = []
            for foot_id in moving_feet:
                name_foot = self._contact_names[foot_id]
                surfaces = []
                surface_name = []  # to remove
                for key, surface_collision in self.all_surfaces_collision.items():
                    if distance(surface_collision, self.roms_collision[name_foot], hppfcl.Transform3f(), tf) < 0:
                        surfaces.append(self.all_surfaces[key])
                        surface_name.append(key)
                if not len(surfaces):
                    # In case there are not any potential surface to use, gives all the surfaces as potential surfaces.
                    print("Warning : no potential surfaces.")
                    surfaces = [value for value in self.all_surfaces.values()]
                if self._RECORDING:
                    # Save the number of potential surfaces
                    self.profiler["potential_number"].append(len(surfaces))

                # Sort and then convert to array
                surfaces = sorted(surfaces)
                surfaces_array = []
                for surface in surfaces:
                    surfaces_array.append(np.array(surface).T)

                # Add to surfaces list
                foot_surfaces.append(surfaces_array)
            surfaces_list.append(foot_surfaces)

        return surfaces_list, empty_list

    def set_surfaces(self, all_surfaces):
        """ Set the surfaces.

        Args:
            - surfaces (dict): Dictionnary containing the surfaces.
        """
        # self.all_surfaces = all_surfaces

        # Dictionnary type containing the convex FCl object for collision checking
        # self.all_surfaces_collision = dict(
        # zip(all_surfaces.keys(), [convert_to_convexFcl(value) for value in all_surfaces.values()]))

        # Convert to Fcl might return error (qhull)
        self.all_surfaces_collision = dict()
        self.all_surfaces = dict()
        for key in all_surfaces.keys():
            try:
                if len(all_surfaces[key]) > 2:
                    collision = convert_to_convexFcl(all_surfaces[key])
                    self.all_surfaces_collision[key] = collision
                    self.all_surfaces[key] = all_surfaces[key]
            except:
                print("Could not create Convex model of the surface. Skip this surfaces.")

    def _retrieve_surfaces(self, surfaces, indices):
        """ Update the structure containing the surfaces selected for each foot. Trying to be independant from hyper-parameters.

        Args:
            -  surfaces (list): The list of potential surfaces. The surfaces are defined using the vertices positions:
                                array([[x0, x1, ... , xn],
                                        [y0, y1, ... , yn],
                                        [z0, z1, ... , zn]])
            - indices (list): For each step in the horizon, list of the selected surfaces.
        """
        self._selected_surfaces.clear()
        for name in self._contact_names:
            self._selected_surfaces[name] = []  # empty list
        for phase_id, phase in enumerate(self.pb.phaseData):
            for id, foot in enumerate(phase.moving):
                id_sf = indices[phase_id][id]
                self._selected_surfaces.get(self._contact_names[foot]).append(
                    Surface(phase.S[id][id_sf][0][:, :], phase.S[id][id_sf][1][:],
                            surfaces[phase_id][id][id_sf][:, :]))

        # Add until N_phase_return for each foot if the MPC horizon is longer.
        # Should not happen. horizon_sl1m >> horizon_mpc.
        # A security mechanism should be setup on mpc side as well.
        for name in self._contact_names:
            while len(self._selected_surfaces.get(name)) < self._N_phase_return:
                self._selected_surfaces.get(name).append(self._selected_surfaces.get(name)[-1])

    def run(self, q, gait_in, timings_in, bvref, target_foostep):
        """ Select the nex surfaces to use.

        Args:
            - q (array x7): Current state [pos x3 , quaternion x4].
            - gait (Array n_gait x 4): Next walking gait.
            - bvref (Array x6): Reference velocity.
            - target_foostep (Array 3x4): The next foot step position. (For the next phase of contact)

        Return:
            - param1 (dict): Next surfaces for each foot.
        """
        if len(q) != 7:
            raise ArithmeticError("Current state should be size 7, [pos x3 , quaternion x4]")

        # Compute configurations.
        t0 = clock()
        configs, effector_positions = self._compute_configuration(q[:7], bvref, gait_in, timings_in)
        t1 = clock()
        # print("_compute_configuration [ms] : ", 1000 * (t1 - t0))
        if self._RECORDING:
            self.profiler["timing_configuration"] = t1 - t0

        self.configs = configs

        # Orientation for each configurations.
        R = [pin.XYZQUATToSE3(np.array(config)).rotation for config in configs]

        # Remove static phases.
        gait = self._compute_gait(gait_in)

        # Update the intern _step_duration parameters for visualisation.
        self._step_duration = np.mean(timings_in[1:])

        # Initial contact at the beginning of the next phase.
        initial_contacts = [np.array(target_foostep[:, i].tolist()) for i in range(4)]

        t0 = clock()
        surfaces, empty_list = self.get_potential_surfaces(configs, gait)
        t1 = clock()
        # print("get_potential_surfaces [ms] : ", 1000 * (t1 - t0))
        if self._RECORDING:
            self.profiler["timing_potential"] = t1 - t0

        self.pb.generate_problem(R, surfaces, gait, initial_contacts, configs[0][:3], com=self._com)

        if empty_list:
            raise ArithmeticError("One step has no pentential surface to use")

        # Compute the costs
        # effector_positions = self._compute_effector_positions(configs, bvref)
        shoulder_position = self._compute_shoulder_positions(configs)

        # New cost function tests
        # if bvref[0] > 0:
        #     feet_r = dict(zip([2, 3], [1, 0]))
        #     feet = [2,3]
        #     feet = [0,1]
        # else:
        #     feet_r = dict(zip([1,0],[2,3]))
        #     feet = [0,1]

        # Walking costs
        # costs = {
        # "height_first_phase_cost": [0.12, feet_r],
        # "height_first_phase_cost2": [
        #     1.0, pin.rpy.matrixToRpy(pin.Quaternion(configs[0][3:]).toRotationMatrix())[1]
        # ],
        # "effector_positions_xy": [1.0, effector_positions]
        # }

        # Walking costs (almost working but stay locked in the midlle)
        # costs = {
        #     "height_first_phase_cost2": [
        #         5.0, pin.rpy.matrixToRpy(pin.Quaternion(configs[0][3:]).toRotationMatrix())[1]
        #     ],
        #     "effector_position_cost_xy_selected": [2.5, [feet, shoulder_position]],
        #     "effector_positions_xy": [4.0, effector_positions],
        #     "height_first_phase_cost2": [
        #         5.0, pin.rpy.matrixToRpy(pin.Quaternion(configs[0][3:]).toRotationMatrix())[1]
        #     ]
        # }

        ############################################################
        # Walking cost with new potential surfaces
        costs = {"effector_positions_xy": [1.0, effector_positions]}
        # activate shoulder cost for going up (should work for going down)
        # if bvref[0] > 0:
        #     feet_selected = [2,3]
        #     costs["effector_positions_3D_select"] = [0.5, [feet_selected, shoulder_position]]

        # Walk : ! Depends on the forward sign vx > 0 here
        # feet_selected = [0,1]
        # costs["effector_positions_3D_select"] = [0.5, [feet_selected, shoulder_position]]

        pitch = pin.rpy.matrixToRpy(pin.Quaternion(q[3:7]).toRotationMatrix())[1]

        # Should work for going up & down
        feet_selected = [2, 3]
        if pitch > 0.05:
            feet_selected = [0, 1]  # Going down forward
            costs["effector_positions_3D_select"] = [0.25, [feet_selected, shoulder_position]]
        else:
            costs["effector_positions_3D_select"] = [0.2, [feet_selected, shoulder_position]]
        #############################################################

        #############################################################
        # Trotting cost:
        # costs = {
        #     "effector_positions_xy": [1.0, effector_positions]
        # }
        # For going down : vx > 0
        # feet_selected = [0,1]
        # costs["effector_positions_3D_select"] = [0.2, [feet_selected, shoulder_position]]

        # Going up
        # feet_selected = [2,3]
        # costs["effector_positions_3D_select"] = [0.2, [feet_selected, shoulder_position]]

        # # Regularization cost
        # if self.pb_data is not None and self.pb_data.success:
        #     previous_position = self._compute_sl1m_position(self.pb_data)
        #     # feet_selected = [0,1]
        #     costs["regularization_cost"] = [0.4, previous_position]
        ###############################################################

        # Trotting costs
        # costs = {
        # "height_first_phase_cost2": [
        #     3.0, pin.rpy.matrixToRpy(pin.Quaternion(configs[0][3:]).toRotationMatrix())[1]
        # ],
        # "effector_position_cost_xy_selected": [2.5, [feet, shoulder_position]],
        # "effector_positions_xy": [4.0, effector_positions]
        # }

        # CoM cost
        if self._com:
            com_positions = self._compute_com_positions(configs)
            costs["coms_xy"] = [0.5, com_positions]
            costs["coms_z"] = [0.05, com_positions]

        # Solve MIP
        t0 = clock()
        self.pb_data = solve_MIP(self.pb, costs=costs, com=self._com)
        t1 = clock()
        if self._RECORDING:
            self.profiler["timing_MIP"] = t1 - t0
        print("SL1M optimization took [ms] : ", 1000 * (t1 - t0))

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
