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
import os
from time import perf_counter as clock
import copy

from walkgen_surface_planner.tools.heightmap import load_heightmap
from walkgen_surface_planner.tools.Surface import Surface
from walkgen_surface_planner.params import SurfacePlannerParams

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

    def __init__(self,  initial_height=0., q0=None, params=None):
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

        if q0 is None:
            self._q0 = np.zeros(7)
            self._q0[-1] = 1
        else:
            if len(q0) != 7:
                raise AttributeError("Initial configuration should be size 7, [position, quaternion]")
            self._q0 = q0

        self.planeseg = self._params.planeseg # Use URDF or planseg

        if not self.planeseg:  # Use URDF and heightmap environment
            from hpp.corbaserver.affordance.affordance import AffordanceTool
            from hpp.corbaserver.rbprm.tools.surfaces_from_path import getAllSurfacesDict
            from hpp.corbaserver.problem_solver import ProblemSolver
            from hpp.gepetto import ViewerFactory
            from walkgen_surface_planner.tools.geometry_utils import getAllSurfacesDict_inner
            from anymal_rbprm.anymal_abstract import Robot as AnymalAbstract

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

            self._afftool.loadObstacleModel(self._params.path + self._params.urdf, "environment", self._vf)
            self._ps.selectPathValidation("RbprmPathValidation", 0.05)

            # Height matrix is expressed in o frame, the position and orientation of the robot is known.
            # pos = (0,0,height_feet = 0.), quat = (0,0,0,1)
            self._wRo = pin.Quaternion(self._q0[3:]).toRotationMatrix()
            self._wTo = np.zeros(3)
            self._wTo[:2] = self._q0[:2]
            self._wTo[2] = initial_height
            self.heightmap = load_heightmap(self._params.path + self._params.heightmap, initial_height)  # in o frame.
            self.worldPose = np.zeros(7)
            self.worldPose[:3] = self._wTo[:]
            self.worldPose[3:] = self._q0[3:]

            # Move the entire environment to follow the initial configuration.
            all_names = self._afftool.getAffRefObstacles("Support")
            for name in all_names:
                self._vf.moveObstacle(name, self.worldPose.tolist())

            self._all_surfaces = getAllSurfacesDict_inner(getAllSurfacesDict(self._afftool), margin=self._params.margin)

        # SL1M Problem initialization
        self.pb = Problem(limb_names=limbs,
                          other_names=others,
                          constraint_paths=paths,
                          suffix_com=suffix_com,
                          suffix_feet=suffix_feet)

        # Gait parameters
        self._set_gait_param(self._params)

        # SL1M parameters
        self._com = self._params.com

        self._contact_names = ['LF_FOOT', 'RF_FOOT', 'LH_FOOT', 'RH_FOOT']  # Order of the feet in the surface planner.
        # Shoulder position in base frame
        self._shoulders = np.array([[0.37, 0.37, -0.37, -0.37], [0.2, -0.2, 0.2, -0.2], [0., 0., 0., 0.]])
        self._current_position = np.array([[0.37, 0.37, -0.37, -0.37], [0.2, -0.2, 0.2, -0.2], [0., 0., 0., 0.]])
        self._reference_height = 0.4792

        # Initial selected surface, rectangle of 4dxdy m2 around the initial position.
        dx = 1.5 # Distance on x-axis around the initial position.
        dy = 1.5 # Distance on y-axis around the initial position.
        # Assume 4 feet are on the ground
        epsilon = 10e-6
        A = [[-1., 0., 0.], [0., -1., 0.], [0., 1., 0.], [1., 0., 0.], [0., 0., 1.], [-0., -0., -1.]]
        b = [
            dx - self._q0[0], dy - self._q0[1], dy + self._q0[1], dx + self._q0[0], initial_height + epsilon,
            -initial_height + epsilon
        ]
        vertices = [[self._q0[0] - dx, self._q0[1] + dy, initial_height],
                    [self._q0[0] - dx, self._q0[1] - dy, initial_height],
                    [self._q0[0] + dx, self._q0[1] - dy, initial_height],
                    [self._q0[0] + dx, self._q0[1] + dy, initial_height]]
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
        self.configs = None

    def _set_gait_param(self,params):
        """ Initialize gait parameters.

        Args:
            - params (WalkgenParams): parameter class.
        """
        # Gait parameters
        self._typeGait = self._params.typeGait
        if self._typeGait == "trot":
            n_gait = 2
            N_total = 2 * self._params.N_ss
        elif self._typeGait == "walk":
            n_gait = 4
            N_total =  self._params.N_ds + 4 * self._params.N_ss
        else:
            raise SyntaxError("Unknown gait type in the config file. Try Trot or Walk.")

        # SL1M parameters
        self._T_gait = N_total * self._params.dt  # Period of a gait.
        self._n_gait = n_gait  # Number of phases in a gait.
        self._step_duration = self._T_gait / n_gait
        self._N_phase = self._params.N_phase
        self._N_phase_return = self._params.N_phase_return
        if self._N_phase_return > self._N_phase :
            raise ArithmeticError("Cannot return more surfaces than step done in SL1M")
        self._N_total = self._n_gait * self._N_phase

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

    def _compute_shoulder_positions_2D_tuned(self, configs, bvref):
        """Compute the shoulder positions, keep heuristic for back foot.
        TODO: Maybe add a way to penalize only some feet in SL1M.

        Args:
            - configs (list): List of configurations (Array x7, [position, orientation]).
        """
        t_stance = self._T_gait / self._n_gait
        shoulder_positions = np.zeros((4, self.pb.n_phases, 2))

        for phase in self.pb.phaseData:
            for foot in phase.moving:
                if foot == 0 or foot == 1:
                    R = pin.Quaternion(configs[phase.id][3:7]).toRotationMatrix()
                    sh = R @ self._shoulders[:, foot] + configs[phase.id][:3]
                    shoulder_positions[foot][phase.id] = sh[:2]
                else:
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
                    shoulder_positions[foot][phase.id] = np.array(configs[phase.id][:2] + shoulders)

        return shoulder_positions

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
                sh = R @ self._shoulders[:, foot] + configs[phase.id][:3]
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
        # List of configurations in planned horizon, using the reference velocity.
        configs = []

        rpyMap_, fit_o, fit_w = np.zeros(3), np.zeros(3), np.zeros(3)
        if not self.planeseg:
            q_o = self._wRo.T @ (np.array([q[0], q[1], 0.]) - self._wTo)
            # print("q_o : ", q_o)
            # In frame_o, a_o * x + b_o * y -z + c_o = 0, fit = [a,b,c]
            # [a b -1]^T [x, y, z]_o = - c, naming ab_o = [a, b, -1]
            fit_o = self.heightmap.fit_surface(q_o[0], q_o[1]) # rpy in in frame_o
            ab_o = np.array([fit_o[0], fit_o[1], -1])
            ab_w = ab_o @ self._wRo.T
            c_w = fit_o[2] - ab_o @ self._wRo.T @ self._wTo
            fit_w[:2] = ab_w[:2]
            fit_w[2] = c_w
            rpyMap_[0] = -np.arctan2(fit_w[1], 1.)
            rpyMap_[1] = -np.arctan2(fit_w[0], 1.)

            # print(rpyMap_)
            # rpyMap_ = self._wRo @ rpyMap_
            # print(rpyMap_)

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

            config[2] = fit_w[0] * config[0] + fit_w[1] * config[1] + fit_w[2] + self._reference_height
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

    def run(self, q, gait_in, bvref, target_foostep, set_surfaces=None):
        """ Select the nex surfaces to use.

        Args:
            - q (array x19): Current state [pos x3 , quaternion x4, joint pos x12].
            - gait (Array n_gait x 4): Next walking gait.
            - bvref (Array x6): Reference velocity.
            - target_foostep (Array 3x4): The next foot step position. (For the next phase of contact)
            - set_surfaces (list): The set of surfaces if not provided by the collision tool. (list of surfaces)

        Return:
            - param1 (dict): Next surfaces for each foot.
        """
        if len(q) != 19:
            raise ArithmeticError("Current state should be size 19, [pos x3 , quaternion x4, joint pos x12]")
        if set_surfaces == None and self.planeseg == True:
            raise ArithmeticError("No surfaces provided, SL1M running without the env URDF.")

        t0 = clock()

        configs = self._compute_configuration(q[:7], bvref)  # Compute configurations.
        self.configs = configs

        R = [pin.XYZQUATToSE3(np.array(config)).rotation for config in configs]  # Orientation for each configurations.

        gait = self._compute_gait(gait_in) # remove redundancies + roll the matrix of 21.

        # Initial contact at the beginning of the next phase.
        initial_contacts = [np.array(target_foostep[:, i].tolist()) for i in range(4)]

        if self.planeseg:
            self.surfaces_processed = set_surfaces # MarkerArray already processed.

            surfaces, empty_list = self._get_potential_surfaces(configs, gait, self.surfaces_processed)

        else:
            surfaces, empty_list = self._get_potential_surfaces_RBPRM(configs, gait)

        self.pb.generate_problem(R, surfaces, gait, initial_contacts, configs[0][:3], com=self._com)

        if empty_list:
            raise ArithmeticError("One step has no pentential surface to use")

        # Compute the costs
        effector_positions = self._compute_effector_positions(configs, bvref)
        shoulder_position_tuned = self._compute_shoulder_positions_2D_tuned(configs, bvref)
        shoulder_position = self._compute_shoulder_positions(configs)
        com_positions = self._compute_com_positions(configs)

        costs = {
            "effector_positions_3D": [0.1, shoulder_position],
            "effector_positions_xy": [1.0, effector_positions],
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
