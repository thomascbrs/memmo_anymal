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

from sl1m.problem_definition import Problem
from sl1m.generic_solver import solve_MIP

from anymal_rbprm.anymal_abstract import Robot as AnymalAbstract

from hpp.corbaserver.affordance.affordance import AffordanceTool
from hpp.corbaserver.rbprm.tools.surfaces_from_path import getAllSurfacesDict
from hpp.corbaserver.problem_solver import ProblemSolver
from hpp.gepetto import ViewerFactory

from example_robot_data.robots_loader import ANYmalLoader

from walkgen.tools.heightmap import load_heightmap
from walkgen.tools.geometry_utils import getAllSurfacesDict_inner
import copy
# --------------------------------- PROBLEM DEFINITION ---------------------------------------------------------------

# paths = [os.environ["INSTALL_HPP_DIR"] + "/solo-rbprm/com_inequalities/feet_quasi_flat/",
#          os.environ["INSTALL_HPP_DIR"] + "/solo-rbprm/relative_effector_positions/"]
# suffix_com = "_effector_frame_quasi_static_reduced.obj"

paths = [
    os.environ["INSTALL_HPP_DIR"] + "/anymal-rbprm/com_inequalities/feet_quasi_flat/anymal_",
    os.environ["INSTALL_HPP_DIR"] + "/anymal-rbprm/relative_effector_positions/anymal_"
]
suffix_com = "_effector_frame_quasi_static_reduced.obj"
limbs = ['LFleg', 'RFleg', 'LHleg', 'RHleg']
others = ['LF_ADAPTER_TO_FOOT', 'RF_ADAPTER_TO_FOOT', 'LH_ADAPTER_TO_FOOT', 'RH_ADAPTER_TO_FOOT']
suffix_feet = "_reduced.obj"
rom_names = ['anymal_LFleg_rom', 'anymal_RFleg_rom', 'anymal_LHleg_rom', 'anymal_RHleg_rom']


class SurfacePlannerURDF:
    """
    Choose the next surface to use by solving a MIP problem
    """

    def __init__(self, environment_URDF, heightmap_path, T_gait):
        """
        Initialize the affordance tool and save the solo abstract rbprm builder, and surface dictionary
        """
        self.anymal_abstract = AnymalAbstract()
        self.anymal_abstract.setJointBounds("root_joint", [-5., 5., -5., 5., 0.241, 1.5])
        self.anymal_abstract.boundSO3([-3.14, 3.14, -0.01, 0.01, -0.01, 0.01])
        self.anymal_abstract.setFilter(rom_names)
        for limb in rom_names:
            self.anymal_abstract.setAffordanceFilter(limb, ['Support'])
        self.ps = ProblemSolver(self.anymal_abstract)
        self.vf = ViewerFactory(self.ps)
        self.afftool = AffordanceTool()
        self.afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])

        self.afftool.loadObstacleModel(environment_URDF, "environment", self.vf)
        self.ps.selectPathValidation("RbprmPathValidation", 0.05)

        self.all_surfaces = getAllSurfacesDict_inner(getAllSurfacesDict(self.afftool), margin=0.06)

        self.potential_surfaces = []

        # SL1M Problem initialization
        self.pb = Problem(limb_names=limbs,
                          other_names=others,
                          constraint_paths=paths,
                          suffix_com=suffix_com,
                          suffix_feet=suffix_feet)

        self.T_gait = 0.6
        self.N_phase = 4  # Number of phases to predict for SL1M

        self.heightmap = load_heightmap(heightmap_path)

        # TODO, modify hand tune parameters
        self.step_duration = self.T_gait / 2
        self.reference_height = 0.4792

        # Load Anymal model to get the current feet position by forward kinematic.
        ANYmalLoader.free_flyer = True
        self.anymal = ANYmalLoader().robot

        # Get Anymal feet offset
        pin.centerOfMass(self.anymal.model, self.anymal.data, self.anymal.q0)
        pin.updateFramePlacements(self.anymal.model, self.anymal.data)
        pin.crba(self.anymal.model, self.anymal.data, self.anymal.q0)

        self.contact_names = ['LF_FOOT', 'RF_FOOT', 'LH_FOOT', 'RH_FOOT']
        self.shoulders = np.zeros((3, 4))
        self.current_position = np.zeros((3, 4))
        for i, idx in enumerate(self.contact_names):
            Id = self.anymal.model.getFrameId(idx)
            self.shoulders[:, i] = self.anymal.data.oMf[Id].translation
            self.current_position[:, i] = self.anymal.data.oMf[Id].translation

        # Initial selected surface
        A = [[-1.0000000, 0.0000000, 0.0000000], [0.0000000, -1.0000000, 0.0000000], [0.0000000, 1.0000000, 0.0000000],
             [1.0000000, 0.0000000, 0.0000000], [0.0000000, 0.0000000, 1.0000000],
             [-0.0000000, -0.0000000, -1.0000000]]

        b = [1.3946447, 0.9646447, 0.9646447, 0.5346446, 0.0000, 0.0000]

        vertices = [[-1.3946447276978748, 0.9646446609406726, 0.0], [-1.3946447276978748, -0.9646446609406726, 0.0],
                    [0.5346445941834704, -0.9646446609406726, 0.0], [0.5346445941834704, 0.9646446609406726, 0.0]]

        self.selected_surfaces = dict()  # Mimic asynchronous behaviour
        for foot in range(4):
            self.selected_surfaces[self.contact_names[foot]] = Surface(np.array(A), np.array(b), np.array(vertices))

        self.pb_data = None # For degub purpose, to get external access to the results.

    def compute_gait(self, gait_in):
        """
        Get a gait matrix with only one line per phase
        :param gait_in: gait matrix with several line per phase
        :return: gait matrix
        """
        gait = [gait_in[0, :]]

        for i in range(1, gait_in.shape[0]):
            new_phase = True
            for row in gait:
                if (gait_in[i, :] == row).any():
                    new_phase = False

            if new_phase:
                gait.append(gait_in[i, :])

        gait = np.roll(gait, -2, axis=0)

        return gait

    def compute_step_length(self, o_v_ref):
        """
        Compute the step_length used for the cost
        :param o_v_ref: desired velocity
        :return: desired step_length
        """
        # TODO: Divide by number of phases in gait
        step_length = o_v_ref * self.T_gait / 2

        return np.array([step_length[i] for i in range(2)])

    def compute_effector_positions(self, configs, bvref):
        """
        Compute the desired effector positions
        :param configs the list of configurations
        :param bvref, Array (x3) the desired velocity in base frame
        """
        # TODO: Divide by number of phases in gait
        t_stance = self.T_gait / 2
        effector_positions = np.zeros((4, self.pb.n_phases, 2))

        for phase in self.pb.phaseData:
            for foot in phase.moving:
                rpy = pin.rpy.matrixToRpy(pin.Quaternion(configs[phase.id][3:7]).toRotationMatrix())
                yaw = rpy[2]  # Get yaw for the predicted configuration
                shoulders = np.zeros(2)
                # Compute heuristic position in horizontal frame
                rpy[2] = 0.  # Yaw = 0. in horizontal frame
                Rp = pin.rpy.rpyToMatrix(rpy)[:2, :2]
                heuristic = 0.5 * t_stance * Rp @ bvref[:2] + Rp @ self.shoulders[:2, foot]

                # Compute heuristic in world frame, rotation
                shoulders[0] = heuristic[0] * np.cos(yaw) - heuristic[1] * np.sin(yaw)
                shoulders[1] = heuristic[0] * np.sin(yaw) + heuristic[1] * np.cos(yaw)
                effector_positions[foot][phase.id] = np.array(configs[phase.id][:2] + shoulders)

        return effector_positions

    def compute_shoulder_positions(self, configs):
        """
        Compute the shoulder positions
        :param configs the list of configurations
        """
        shoulder_positions = np.zeros((4, self.pb.n_phases, 3))

        for phase in self.pb.phaseData:
            for foot in phase.moving:
                R = pin.Quaternion(configs[phase.id][3:7]).toRotationMatrix()
                shoulder_positions[foot][phase.id] = R @ self.shoulders[:, foot] + configs[phase.id][:3]

        return shoulder_positions

    def _compute_configuration(self, q, bvref):
        """ Compute configuration for the next phases.

        Args :
            - q (array x6), rpy
            - bvref (array x6),rpy
        """
        yaw_init = pin.rpy.matrixToRpy(pin.Quaternion(q[3:7]).toRotationMatrix())[2]
        print("yaw_init : ", yaw_init)
        # List of configurations in planned horizon, using the reference velocity.
        configs = []

        fit_ = self.heightmap.fit_surface(q[0], q[1])
        rpyMap_ = np.zeros(3)
        rpyMap_[0] = -np.arctan2(fit_[1], 1.)
        rpyMap_[1] = -np.arctan2(fit_[0], 1.)

        for i in range(self.N_phase):
            config = np.zeros(7)
            dt_config = self.step_duration * (i + 1)  # Delay of 2 phase of contact for MIP

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

            config[2] = fit_[0] * config[0] + fit_[1] * config[1] + fit_[2] + self.reference_height
            yaw = yaw_init + bvref[5] * dt_config
            roll = rpyMap_[0] * np.cos(yaw) - rpyMap_[1] * np.sin(yaw)
            pitch = rpyMap_[0] * np.sin(yaw) + rpyMap_[1] * np.cos(yaw)
            config[3:] = pin.Quaternion(pin.rpy.rpyToMatrix(roll, pitch, yaw)).coeffs()
            configs.append(config)

        return configs

    def _update_current_position(self, q):
        """ Get current foot position by forward kinematics.
        Args:
            - q
            - vq
        """
        pin.forwardKinematics(self.anymal.model, self.anymal.data, q)
        for i, name in enumerate(self.contact_names):
            frame_id = self.anymal.model.getFrameId(name)
            oMf = pin.updateFramePlacement(self.anymal.model, self.anymal.data, frame_id)
            self.current_position[:, i] = oMf.translation[:]

    def compute_com_positions(self, configs):
        """
        Compute the com positions
        :param configs the list of configurations
        """
        com_positions = []
        for phase in self.pb.phaseData:
            com = configs[phase.id][:3]
            # com[2] += 0.5
            com_positions.append(configs[phase.id][:3])

        return com_positions

    def get_potential_surfaces(self, configs, gait):
        """
        Get the rotation matrix and surface condidates for each configuration in configs
        :param configs: a list of successive configurations of the robot
        :param gait: a gait matrix
        :return: a list of surface candidates
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
                surfaces_names = self.anymal_abstract.clientRbprm.rbprm.getCollidingObstacleAtConfig(
                    config.tolist(), rom)
                for name in surfaces_names:
                    surfaces.append(self.all_surfaces[name][0])

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

    def retrieve_surfaces(self, surfaces, indices=None):
        """
        Get the surface vertices, inequalities and selected surface indices if need be
        """
        vertices = []
        surfaces_inequalities = []
        if indices is not None:
            surface_indices = []
        else:
            surface_indices = None
        first_phase_i = 0
        second_phase_i = 0
        for foot in range(4):
            if foot in self.pb.phaseData[0].moving:
                vertices.append(surfaces[0][first_phase_i])
                surfaces_inequalities.append(self.pb.phaseData[0].S[first_phase_i])
                if indices is not None:
                    surface_indices.append(indices[0][first_phase_i])
                first_phase_i += 1
            elif foot in self.pb.phaseData[1].moving:
                vertices.append(surfaces[1][second_phase_i])
                surfaces_inequalities.append(self.pb.phaseData[1].S[second_phase_i])
                if indices is not None:
                    surface_indices.append(indices[1][second_phase_i])
                second_phase_i += 1
            else:
                print("Error : the foot is not moving in any of the first two phases")

        return vertices, surfaces_inequalities, surface_indices

    def run(self, q, gait_in, bvref, target_foostep):
        """
        Select the nex surfaces to use
        :param xref: successive states
        :param gait: a gait matrix
        :param current_contacts: the initial_contacts to use in the computation
        :param bvref: Array (x3) the desired velocity for the cost, in base frame
        :return: the selected surfaces for the first phase
        """
        t0 = clock()

        configs = self._compute_configuration(q, bvref)
        # print(configs)
        # print(target_foostep)
        self._update_current_position(q)

        # current_pos = np.zeros((3,4))
        # for foot in range(4):
        #     if gait_in[0,foot] == 0:
        #         current_pos[:,foot] = target_foostep[:,foot]
        #     else:
        #         current_pos[:,foot] = self.current_position[:,foot]

        current_pos = self.current_position

        R = [pin.XYZQUATToSE3(np.array(config)).rotation for config in configs]

        gait = self.compute_gait(gait_in)

        step_length = self.compute_step_length(bvref[:2])

        surfaces, empty_list = self.get_potential_surfaces(configs, gait)

        # from IPython import embed
        # embed()

        initial_contacts = [np.array(current_pos[:, i].tolist()) for i in range(4)]

        self.pb.generate_problem(R, surfaces, gait, initial_contacts, configs[0][:3], com=True)

        if empty_list:
            print("Surface planner: one step has no potential surface to use.")
            vertices, inequalities, indices = self.retrieve_surfaces(surfaces)
            return vertices, inequalities, indices, None, False

        effector_positions = self.compute_effector_positions(configs, bvref)
        shoulder_positions = self.compute_shoulder_positions(configs)
        # print(shoulder_positions)
        com_positions = self.compute_com_positions(configs)
        # costs = {"step_size": [1.0, step_length]}
        costs = {
            "effector_positions": [1., effector_positions],
            "shoulder_cost": [0.5, shoulder_positions],
            "height_effector": [0., 0.]
        }
        # costs = {"effector_positions": [1.0, effector_positions] , "effector_positions_3D": [0., shoulder_positions]}
        # from IPython import embed
        # embed()
        # costs = {"effector_positions": [1.0, effector_positions], "coms_3D": [0.1, com_positions]}
        # COSTS = {"posture": [1.0]}
        self.pb_data = solve_MIP(self.pb, costs=costs, com=False)

        # ax = plot.draw_whole_scene(self.all_surfaces)
        # plot.plot_planner_result(pb_data.all_feet_pos, coms=pb_data.coms, ax=ax, show=True)
        # R = pin.rpy.rpyToMatrix(q[3:6, 0])
        # o_com = q[:3, 0]
        # normal = np.array([0., 0., 1.])
        # transform = default_transform_from_pos_normal(np.zeros(3), normal, R)
        # K = []
        # foot_contact = []
        # for foot in range(4):
        #     if gait[foot] == 1.:
        #         foot_contact.append(foot)
        #         ine = rotate_inequalities(self.com_objects[foot], transform.copy())
        #         K.append((ine.A, ine.b))
        # flag = 0
        # for i in range(len(K)):
        #     A = K[i][0]
        #     b = K[i][1]
        #     ERR = A @ (o_com - o_feet[:, foot_contact[i]]) > b
        #     if np.any(ERR):
        #         idx = np.where(ERR)[0][0]
        #         c = A @ (o_com - o_feet[:, foot_contact[i]])

        #         print("PROBLEM WITH INEQUALITIES, FOOT : ", foot_contact[i])
        # from IPython import embed
        # embed()

        if self.pb_data.success:
            print("SUCCESSSSSSSSS")
            surface_indices = self.pb_data.surface_indices

            # Not used
            # selected_surfaces = []
            # for index_phase in range(len(surface_indices)):
            #     surface_tmp = []
            #     for foot, index in enumerate(surface_indices[index_phase]):
            #         surface_tmp.append(surfaces[index_phase][foot][index])
            #     selected_surfaces.append(surface_tmp)

            t1 = clock()
            if 1000. * (t1 - t0) > 150.:
                print("Run took ", 1000. * (t1 - t0))

            # import matplotlib.pyplot as plt
            # import sl1m.tools.plot_tools as plot

            # ax = plot.draw_whole_scene(self.all_surfaces)
            # plot.plot_planner_result(pb_data.all_feet_pos, step_size=step_length, ax=ax, show=True)

            vertices, inequalities, indices = self.retrieve_surfaces(surfaces, surface_indices)

            # TODO Definie what quantities to return
            # return vertices, inequalities, indices, pb_data.all_feet_pos, True
            selected_surfaces = copy.deepcopy(self.selected_surfaces)
            self.selected_surfaces.clear()
            for foot, foot_surfaces in enumerate(inequalities):
                i = indices[foot]
                S, s = foot_surfaces[i]
                self.selected_surfaces[self.contact_names[foot]] = Surface(S, s, vertices[foot][i].T)

            return selected_surfaces

        else:
            print("FAILLLLLLL")
            # ax = plot.draw_whole_scene(self.all_surfaces)
            # plot.plot_initial_contacts(initial_contacts, ax=ax)
            # ax.scatter([c[0] for c in configs], [c[1] for c in configs], [c[2] for c in configs], marker='o', linewidth=5)
            # ax.plot([c[0] for c in configs], [c[1] for c in configs], [c[2] for c in configs])

            # plt.show()

            print("The MIP problem did NOT converge")
            # TODO what if the problem did not converge ???

            vertices, inequalities, indices = self.retrieve_surfaces(surfaces)

            # return vertices, inequalities, indices, None, False
            return 0


class Surface():

    def __init__(self, A, b, vertices):

        self.A = A
        self.b = b
        self.vertices = vertices


if __name__ == "__main__":

    # Heightmap path
    heightmap_path = os.getcwd() + "/data/lab_scene.dat"
    env_urdf =  os.getcwd() + "/data/urdf/lab_scene.urdf"
    surface_planner = SurfacePlannerURDF(env_urdf, heightmap_path, 0.6)

    q = surface_planner.anymal.q0
    q[0] = 0.
    bvref = np.zeros(6)
    bvref[0] = 0.3
    bvref[5] = 0.1
    gait_in = np.array([[1, 0, 0, 1], [0, 1, 1, 0]])

    configs = [
        np.array([
            -7.51004948e-01, 3.39275763e-01, 4.79100000e-01, 3.98948975e-20, -1.66031243e-20, -4.12095976e-01,
            9.11140443e-01
        ]),
        np.array([
            9.56311165e-01, 1.29181564e+00, 4.79100000e-01, -3.70796316e-20, -2.21893556e-20, 8.03799124e-01,
            5.94900806e-01
        ]),
        np.array([
            -9.91835502e-01, 1.12754426e+00, 4.79100000e-01, -4.22398061e-21, -4.30049366e-20, -7.50970934e-01,
            6.60335261e-01
        ]),
        np.array([
            8.50879152e-01, 4.74349630e-01, 4.79100000e-01, -3.20537649e-20, 2.89796952e-20, 4.87196087e-01,
            8.73292604e-01
        ]),
        np.array([
            -5.58880351e-01, 1.82891180e+00, 4.79100000e-01, 4.23628748e-20, 8.52369756e-21, 9.56368711e-01,
            -2.92162435e-01
        ]),
        np.array([
            1.67354085e-01, 1.37412654e-02, 4.79100000e-01, 1.83513342e-20, 3.91215428e-20, 8.39990729e-02,
            9.96465833e-01
        ])
    ]

    selected_surfaces = surface_planner.run(q, gait_in, bvref, np.zeros((3,4)))

    import matplotlib.pyplot as plt
    import sl1m.tools.plot_tools as plot

    ax = plot.draw_whole_scene(surface_planner.all_surfaces)
    plot.plot_planner_result(surface_planner.pb_data.all_feet_pos, coms=surface_planner.pb_data.coms, ax=ax, show=True)

    # selected_surfaces = dict()
    # for foot, foot_surfaces in enumerate(inequalities):
    #     i = indices[foot]
    #     S, s = foot_surfaces[i]
    #     selected_surfaces[surface_planner.contact_names[foot]] = Surface(S, s, vertices[foot][i].T)

    # configs = surface_planner._compute_configuration(q,bvref)
    # surface_planner._update_current_position(q)
    # R = [pin.XYZQUATToSE3(np.array(config)).rotation for config in configs]

    # gait = surface_planner.compute_gait(gait_in)

    # surfaces, empty_list = surface_planner.get_potential_surfaces(configs, gait)

    # initial_contacts = [np.array(surface_planner.current_position[:, i].tolist()) for i in range(4)]

    # surface_planner.pb.generate_problem(R, surfaces, gait, initial_contacts, configs[0][:3], com=True)
    # effector_positions = surface_planner.compute_effector_positions(configs, bvref)
    # costs = {"effector_positions": [1.0, effector_positions]}
    # pb_data = solve_MIP(surface_planner.pb, costs=costs, com=False)
