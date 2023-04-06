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

    def __init__(self, params=None):
        """ Initialize the surface planner.

        Args:
            - initial_height (float): Height of the ground.
            - q0 (array x7): Initial position and orientation in world frame.
            - params (WalkgenParams): Parameter class.
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

        # Gait parameters
        self._set_gait_param(self._params)

        # SL1M parameters
        self._com = self._params.com

        # Order of the feet in the surface planner.
        self._contact_names = ['LF_FOOT', 'RF_FOOT', 'LH_FOOT', 'RH_FOOT']
        # Shoulder position in base frame
        self._shoulders = np.array([[0.37, 0.37, -0.37, -0.37], [0.2, -0.2, 0.2, -0.2], [0., 0., 0., 0.]])
        self._reference_height = 0.4792

        # Dictionnary containing for each foot a list of surfaces (size N_phase_return)
        # One foot is moving one time per each gait/phase.
        self._selected_surfaces = dict(
            zip(self._contact_names, [[copy.deepcopy(Surface()) for k in range(3)] for _ in range(4)]))

        # Load rom .stl objects for collision tools to select only the relevant surfaces.
        path = os.environ["INSTALL_HPP_DIR"] + "/anymal-rbprm/meshes/"
        # obj_stl = [trimesh.load_mesh(path + rom) for rom in rom_names]
        obj_stl = []
        for i,rom in enumerate(rom_names):
            obj = trimesh.load_mesh(path + rom)
            obj.apply_translation(-self._shoulders[:,i])
            obj.apply_scale(1.2)
            obj.apply_translation( self._shoulders[:,i])
            obj_stl.append(obj)


        # Dictionnary containing the convex set of roms for collisions.
        self.roms_collision = dict(zip(self._contact_names, [convert_to_convexFcl(obj.vertices) for obj in obj_stl]))

        self.all_surfaces = None
        self.all_surfaces_collision = None
        # Debug and plot purpose
        self.pb_data = None
        self.surfaces_processed = None
        self.configs = None
        
        # Compute the slope of terrain 1 configuration over self._ratio_recompute_slope 
        self._ratio_recompute_slope = 3 

    def _set_gait_param(self, params):
        """ Initialize gait parameters.

        Args:
            - params (WalkgenParams): parameter class.
        """
        # Gait parameters
        self._typeGait = params.typeGait
        if self._typeGait == "trot":
            n_gait = 2
            N_total = 2 * params.N_ss
        elif self._typeGait == "walk":
            n_gait = 4
            N_total = params.N_ds + 4 * params.N_ss
        else:
            raise SyntaxError("Unknown gait type in the config file. Try Trot or Walk.")

        # SL1M parameters
        self._T_gait = N_total * params.dt  # Period of a gait.
        self._n_gait = n_gait  # Number of phases in a gait.
        self._step_duration = self._T_gait / n_gait
        self._N_phase = params.N_phase
        self._N_phase_return = params.N_phase_return
        if self._N_phase_return > self._N_phase:
            warnings.warn(
                "More phases in the MPC horizon than planned by the MIP. The last surface selected will be used multiple times."
            )
        self._N_total = self._n_gait * self._N_phase
        # Trotting in simulation with a lot of surfaces
        # self._N_total = 3
        # Walking in simulation with a lot of surfaces
        # self._N_total = 6     
        
    def _compute_gait(self, gait_in):
        """
        Get a gait matrix with only one line per phase
        :param gait_in: gait matrix with several line per phase
        :return: gait matrix
        """
        gait = []

        for i in range(0, gait_in.shape[0]):
            if np.any(1 - gait_in[i, :]):  # no [1,1,1,1]
                if not gait_in[i, :].tolist() in gait:
                    gait.append(gait_in[i, :].tolist())
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
                heuristic = 0.5 * t_stance * \
                    np.dot(Rp, bvref[:2]) + np.dot(Rp, self._shoulders[:2, foot])

                # Compute heuristic in world frame, rotation
                shoulders[0] = heuristic[0] * \
                    np.cos(yaw) - heuristic[1] * np.sin(yaw)
                shoulders[1] = heuristic[0] * \
                    np.sin(yaw) + heuristic[1] * np.cos(yaw)
                effector_positions[foot][phase.id] = np.array(configs[phase.id][:2] + shoulders)

        return effector_positions

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

        # Select the relevant convex surfaces to approximate the slope of the terrain.
        self._tf.setTranslation(q[:3])
        rotation = pin.rpy.rpyToMatrix(np.array([0.,0.,yaw_init]))
        collision_points = StdVec_MatrixXd()
        for key, surface_collision in self.all_surfaces_collision.items():
            if distance(surface_collision, self._box, hppfcl.Transform3f(), self._tf) < 0:
                collision_points.append(np.array(self.all_surfaces[key]))
        # Get the slope of the terrain.
        fit_ = self._terrain.get_slope(q[:3], rotation, collision_points)
        rpyMap_ = np.zeros(3)
        rpyMap_[0] = np.arctan2(fit_[1], 1.)
        rpyMap_[1] = -np.arctan2(fit_[0], 1.)

        # List of configurations in planned horizon, using the reference velocity.
        configs = []
        
        # Compute the slope of terrain 1 configuration over self._ratio_recompute_slope 
        for i in range(self._N_total):
            config = np.zeros(7)
            # Delay of 1 phase of contact for MIP
            dt_config = self._step_duration * (i + 2)

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
                rotation =  np.dot(pin.rpy.rpyToMatrix(np.array([0.,0.,bvref[5] * dt_config])) , rotation)
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
            config[3:] = pin.Quaternion(np.dot(Rp , Ryaw)).coeffs()
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
            - gait (array nx4): gait matrix.
        """
        surfaces_list = []
        empty_list = False
        for id, config in enumerate(configs):
            tf = hppfcl.Transform3f()
            tf.setTranslation(config[:3])
            tf.setRotation(pin.Quaternion(config[3:]).toRotationMatrix())

            stance_feet = np.nonzero(gait[id % len(gait)] == 1)[0]
            previous_swing_feet = np.nonzero(gait[(id - 1) % len(gait)] == 0)[0]
            moving_feet = stance_feet[np.in1d(stance_feet, previous_swing_feet, assume_unique=True)]

            foot_surfaces = []
            for foot_id in moving_feet:
                name_foot = self._contact_names[foot_id]
                surfaces = []
                surface_name = []  # to remove
                for key, surface_collision in self.all_surfaces_collision.items():
                    if distance(surface_collision, self.roms_collision[name_foot], hppfcl.Transform3f(), tf) < 0:
                        surfaces.append(self.all_surfaces[key])
                        surface_name.append(key)
                print("pot surfaces : " , len(surfaces))
                if not len(surfaces):
                    # In case there are not any potential surface to use, gives all the surfaces as potential surfaces.
                    print("Warning : no potential surfaces.")
                    surfaces = [value for value in self.all_surfaces.values()]

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
        self.all_surfaces = all_surfaces

        # Dictionnary type containing the convex FCl object for collision checking
        self.all_surfaces_collision = dict(
            zip(all_surfaces.keys(), [convert_to_convexFcl(value) for value in all_surfaces.values()]))

    def _retrieve_surfaces(self, surfaces, indices):
        """ Update the structure containing the surfaces selected for each foot.

        Args:
            -  surfaces (list): The list of potential surfaces. The surfaces are defined using the vertices positions:
                                array([[x0, x1, ... , xn],
                                        [y0, y1, ... , yn],
                                        [z0, z1, ... , zn]])
            - indices (list): For each step in the horizon, list of the selected surfaces.
        """
        for k in range(self._N_phase):  # in one phase each foot move one time.
            if k < self._N_phase_return:
                for i in range(self._n_gait):  # The number of step in one phase
                    try :
                        for id, foot in enumerate(self.pb.phaseData[k * self._n_gait + i].moving):
                            # Index of surface choosen in the potential surfaces.
                            id_sf = indices[k * self._n_gait + i][id]
                            self._selected_surfaces.get(
                                self._contact_names[foot])[k].A = self.pb.phaseData[k * self._n_gait +
                                                                                    i].S[id][id_sf][0][:, :]
                            self._selected_surfaces.get(
                                self._contact_names[foot])[k].b = self.pb.phaseData[k * self._n_gait +
                                                                                    i].S[id][id_sf][1][:]
                            self._selected_surfaces.get(
                                self._contact_names[foot])[k].vertices = surfaces[k * self._n_gait + i][id][id_sf][:, :]
                    except :
                        # Quick fix, should not work for walk
                        if self._typeGait == "trot" :
                            for id, foot in enumerate(self.pb.phaseData[k * self._n_gait + i - 2].moving):
                                self._selected_surfaces.get(
                                    self._contact_names[foot])[k].A =  self._selected_surfaces.get(self._contact_names[foot])[k-1].A
                                self._selected_surfaces.get(
                                    self._contact_names[foot])[k].b =  self._selected_surfaces.get(self._contact_names[foot])[k-1].b
                                self._selected_surfaces.get(
                                    self._contact_names[foot])[k].vertices = self._selected_surfaces.get(self._contact_names[foot])[k-1].vertices
                        else :
                            for id, foot in enumerate(self.pb.phaseData[k * self._n_gait + i - 4].moving):
                                self._selected_surfaces.get(
                                    self._contact_names[foot])[k].A =  self._selected_surfaces.get(self._contact_names[foot])[k-1].A
                                self._selected_surfaces.get(
                                    self._contact_names[foot])[k].b =  self._selected_surfaces.get(self._contact_names[foot])[k-1].b
                                self._selected_surfaces.get(
                                    self._contact_names[foot])[k].vertices = self._selected_surfaces.get(self._contact_names[foot])[k-1].vertices
                            

        # Fill with the last surface computed in case the number of phases in the MIP is lower than the one in the Walkgen.
        if self._N_phase_return > self._N_phase:
            for k in range(self._N_phase, self._N_phase_return):
                for foot in range(4):
                    self._selected_surfaces.get(self._contact_names[foot])[k].A = self._selected_surfaces.get(
                        self._contact_names[foot])[-2].A
                    self._selected_surfaces.get(self._contact_names[foot])[k].b = self._selected_surfaces.get(
                        self._contact_names[foot])[-2].b
                    self._selected_surfaces.get(self._contact_names[foot])[k].vertices = self._selected_surfaces.get(
                        self._contact_names[foot])[-2].vertices

    def run(self, q, gait_in, bvref, target_foostep):
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

        t0 = clock()

        # Compute configurations.
        configs = self._compute_configuration(q[:7], bvref)
        self.configs = configs

        t1 = clock()
        print("_compute_configuration [ms] : ", 1000 * (t1 - t0))

        # Orientation for each configurations.
        R = [pin.XYZQUATToSE3(np.array(config)).rotation for config in configs]

        # remove redundancies + roll the matrix of 21.
        gait = self._compute_gait(gait_in)

        # Initial contact at the beginning of the next phase.
        initial_contacts = [np.array(target_foostep[:, i].tolist()) for i in range(4)]

        t0 = clock()
        surfaces, empty_list = self.get_potential_surfaces(configs, gait)
        t1 = clock()
        print("get_potential_surfaces [ms] : ", 1000 * (t1 - t0))

        self.pb.generate_problem(R, surfaces, gait, initial_contacts, configs[0][:3], com=self._com)

        if empty_list:
            raise ArithmeticError("One step has no pentential surface to use")

        # Compute the costs
        effector_positions = self._compute_effector_positions(configs, bvref)
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
        costs = {
            "effector_positions_xy": [1.0, effector_positions]
        }
        # activate shoulder cost for going up (should work for going down)
        # if bvref[0] > 0:
        #     feet_selected = [2,3]
        #     costs["effector_positions_3D_select"] = [0.5, [feet_selected, shoulder_position]]

        # Walk : ! Depends on the forward sign vx > 0 here
        # feet_selected = [0,1]
        # costs["effector_positions_3D_select"] = [0.5, [feet_selected, shoulder_position]]

        pitch = pin.rpy.matrixToRpy(pin.Quaternion(q[3:7]).toRotationMatrix())[1]

        # Should work for going up & down
        feet_selected = [2,3]
        if pitch > 0.05:
            feet_selected = [0,1] # Going down forward
            costs["effector_positions_3D_select"] = [0.25, [feet_selected, shoulder_position]]
        else:
            costs["effector_positions_3D_select"] = [0.5, [feet_selected, shoulder_position]]
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
