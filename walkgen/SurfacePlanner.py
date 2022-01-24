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

from sl1m.problem_definition import Problem
from sl1m.generic_solver import solve_MIP
import os
import pinocchio as pin
from example_robot_data.robots_loader import ANYmalLoader
import numpy as np
from geometry_utils import reduce_surfaces, remove_overlap_surfaces

# -----Problem definition-----
paths = [
    os.environ["INSTALL_HPP_DIR"] + "/anymal-rbprm/com_inequalities/feet_quasi_flat/anymal_",
    os.environ["INSTALL_HPP_DIR"] + "/anymal-rbprm/relative_effector_positions/anymal_"
]
suffix_com = "_effector_frame_quasi_static_reduced.obj"
limbs = ['LFleg', 'RFleg', 'LHleg', 'RHleg']
others = ['LF_ADAPTER_TO_FOOT', 'RF_ADAPTER_TO_FOOT', 'LH_ADAPTER_TO_FOOT', 'RH_ADAPTER_TO_FOOT']
suffix_feet = "_reduced.obj"

INIT_VERTICES = np.array([[1., -1, 0.], [1., 1., 0.], [-1., 1., 0.], [-1., -1, 0.]])

class SurfacePlanner():
    """ Plan the next contact surfaces using SL1M algorithm using MarkerArray topic.
    """

    def __init__(self, N_phase, n_gait, T_gait, margin = 0., n_points=None, method_id = 0, polySize=10, min_area=0., init_vertices= INIT_VERTICES):

        # Initial surface, usefull for first run, if the ground is not detected.
        h_init = -0.06
        self.init_vertices = init_vertices

        # SL1M Problem initialization
        self.pb = Problem(limb_names=limbs,
                          other_names=others,
                          constraint_paths=paths,
                          suffix_com=suffix_com,
                          suffix_feet=suffix_feet)
        # Debug and plot purpose
        self.pb_data = None
        self.surfaces_processed = None

        # PARAMETERS FOR SURFACES PROCESSING
        self.margin = margin
        self.n_points = n_points
        self.method_id = method_id
        self.polySize = polySize
        self.min_area = min_area

        # WALKING PARAMETERS
        self.T_gait = T_gait
        self.n_gait = n_gait # Number of different phases in the gait

        # SL1M PARAMETERS
        self.N_phase = N_phase  # Number of phases

        # Load Anymal model to get the current feet position by forward kinematic.
        ANYmalLoader.free_flyer = True
        self.anymal = ANYmalLoader().robot

        # Get Anymal feet offset
        pin.centerOfMass(self.anymal.model, self.anymal.data, self.anymal.q0)
        pin.updateFramePlacements(self.anymal.model, self.anymal.data)
        pin.crba(self.anymal.model, self.anymal.data, self.anymal.q0)

        indexes = ['LF_FOOT', 'RF_FOOT', 'LH_FOOT', 'RH_FOOT']
        self.offsets_feet = np.zeros((3, 4))
        for i, idx in enumerate(indexes):
            Id = self.anymal.model.getFrameId(idx)
            self.offsets_feet[:, i] = self.anymal.data.oMf[Id].translation

    def _get_potential_surfaces(self,configs, gait, all_surfaces):
        """
        Get the rotation matrix and surface condidates for each configuration in configs
        :param configs: a list of successive configurations of the robot
        :param gait: a gait matrix
        :return: a list of surface candidates
        """
        surfaces_list = []
        empty_list = False
        for id, config in enumerate(configs):
            foot_surfaces = []
            stance_feet = np.nonzero(gait[id % len(gait)] == 1)[0]
            previous_swing_feet = np.nonzero(gait[(id - 1) % len(gait)] == 0)[0]
            moving_feet = stance_feet[np.in1d(stance_feet, previous_swing_feet, assume_unique=True)]

            foot_surfaces.append(all_surfaces)
            surfaces_list.append(foot_surfaces)

        return surfaces_list, empty_list

    def _compute_com_positions(self, configs, pb):
        """
        Compute the com positions
        :param configs the list of configurations
        """
        com_positions = []
        for phase in pb.phaseData:
            com = configs[phase.id][:3]
            # com[2] += 0.5
            com_positions.append(configs[phase.id][:3])

        return com_positions

    def _compute_effector_positions(self, configs, bvref, pb):
        """
        Compute the desired effector positions
        :param configs the list of configurations
        :param bvref, Array (x3) the desired velocity in base frame
        """
        # TODO: Divide by number of phases in gait
        t_stance = self.T_gait / 2
        effector_positions = np.zeros((4, pb.n_phases, 2))

        for phase in pb.phaseData:
            for foot in phase.moving:
                rpy = pin.rpy.matrixToRpy(pin.Quaternion(np.array(configs[phase.id][3:7])).toRotationMatrix())
                yaw = rpy[2]  # Get yaw for the predicted configuration
                shoulders = np.zeros(2)
                # Compute heuristic position in horizontal frame
                rpy[2] = 0.  # Yaw = 0. in horizontal frame
                Rp = pin.rpy.rpyToMatrix(rpy)[:2, :2]
                heuristic = 0.5 * t_stance * Rp @ bvref[:2] + Rp @ self.offsets_feet[:2, foot]

                # Compute heuristic in world frame, rotation
                shoulders[0] = heuristic[0] * np.cos(yaw) - heuristic[1] * np.sin(yaw)
                shoulders[1] = heuristic[0] * np.sin(yaw) + heuristic[1] * np.cos(yaw)
                effector_positions[foot][phase.id] = np.array(configs[phase.id][:2] + shoulders)

        return effector_positions

    def _retrieve_surfaces(self, surfaces, pb, indices=None):
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
            if foot in pb.phaseData[0].moving:
                vertices.append(surfaces[0][first_phase_i])
                surfaces_inequalities.append(pb.phaseData[0].S[first_phase_i])
                if indices is not None:
                    surface_indices.append(indices[0][first_phase_i])
                first_phase_i += 1
            elif foot in pb.phaseData[1].moving:
                vertices.append(surfaces[1][second_phase_i])
                surfaces_inequalities.append(pb.phaseData[1].S[second_phase_i])
                if indices is not None:
                    surface_indices.append(indices[1][second_phase_i])
                second_phase_i += 1
            else:
                pass
                # print("Error : the foot is not moving in any of the first two phases")

        return vertices, surfaces_inequalities, surface_indices

    def run(self, array_markers, configs, gait, current_contacts, bvref):
        """ Select the nex surfaces to use.

        Args:
            - configs (list): List of configurations (Array x7, [position, orientation]).
            - gait (Array n_gait x 4): Next walking gait.
            - current_contacts (Array 3x4): Current feet position.
            - bvref (Array x3): Reference velocity.

        Return:
            - param1 (list): Surfaces processed. The surfaces are defined using the vertices positions:
                     array([[x0, x1, ... , xn],
                            [y0, y1, ... , yn],
                            [z0, z1, ... , zn]])
            - param2 (list): Surface inequalities.
            - param3 (list): Indice of the surfaces choosen for next contact phases.
        """

        R = [pin.XYZQUATToSE3(np.array(config)).rotation for config in configs]

        initial_contacts = [np.array(current_contacts[:, i].tolist()) for i in range(4)]

        # Reduce and sort incoming data
        surfaces_reduced = reduce_surfaces(array_markers, margin=self.margin, n_points=self.n_points)

        # Apply proccess to filter and decompose the surfaces to avoid overlap
        self.surfaces_processed = remove_overlap_surfaces(surfaces_reduced,
                                                     polySize=self.polySize,
                                                     method=self.method_id,
                                                     min_area=self.min_area,
                                                     initial_floor=self.init_vertices.T)

        surfaces, empty_list = self._get_potential_surfaces(configs, gait, self.surfaces_processed)

        # Without CoM optimization
        # costs = {"effector_positions": [1.0, effector_positions]}
        # pb.generate_problem(R, surfaces, gait, initial_contacts, c0=None,  com=False)

        # With CoM optimization
        self.pb.generate_problem(R, surfaces, gait, initial_contacts, configs[0][:3],  com=True)

        # Generate costs
        com_positions = self._compute_com_positions(configs, self.pb)
        effector_positions = self._compute_effector_positions(configs, bvref,self.pb)
        costs = { "effector_positions": [10.0, effector_positions] ,"coms_3D": [0.1,com_positions]}

        self.pb_data = solve_MIP(self.pb, costs=costs,  com=True)

        # Process result SL1M
        if self.pb_data.success:
            surface_indices = self.pb_data.surface_indices
            vertices, inequalities, indices = self._retrieve_surfaces(surfaces, self.pb, surface_indices)
            return vertices, inequalities, indices

        else:
            print("The MIP problem did NOT converge")
            vertices, inequalities, indices = self._retrieve_surfaces(surfaces, self.pb)
            return vertices, inequalities, indices
