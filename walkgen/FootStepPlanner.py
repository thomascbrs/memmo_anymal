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
import pinocchio as pin
import copy

from walkgen.tools.geometry_utils import Surface
from walkgen.tools.optimisation import quadprog_solve_qp


class FootStepPlanner():
    """ FootStepPlanner initialized by URDF model of the environment.
    Use RBPRM as a guide path since the env is available.
    """

    def __init__(self, model, q, debug = False):
        """ Initialize the FootStepPlanner.

        Args:
            - model (pin.model): Pinocchio robot model.
            - q (array x19): Initial state of the robot.
            - heightmap_path (str): Heightmap binary file path.
            - debug (bool): Store the footstep computed for debug purpose.
        """
        if q.shape[0] != 19:
            raise ArithmeticError(
                "Current state q should be an array of size 19 [pos (x3), quaternion (x4), joint (x12)]")
        # Create the model and data for forward simulation
        self._model = model
        self._data = self._model.createData()

        pin.forwardKinematics(self._model, self._data, q)
        pin.updateFramePlacements(self._model, self._data)

        lf = "LF_FOOT"
        lh = "LH_FOOT"
        rf = "RF_FOOT"
        rh = "RH_FOOT"
        self._contactNames = [lf, lh, rf, rh]
        self._contact_names_SL1M = [lf, rf, lh, rh]
        self._offsets_feet = np.zeros((3, 4))  # Reference shoulder position, base frame.
        self._current_position = np.zeros((3, 4))  # Feet positions, world frame.
        self._current_velocities = np.zeros((3, 4))  # Feet velocities, world frame.
        for i, name in enumerate(self._contactNames):
            Id = self._model.getFrameId(name)
            self._offsets_feet[:, i] = self._data.oMf[Id].translation
            self._current_position[:, i] = self._data.oMf[Id].translation

        self._k_feedback = 0.03
        self._href = 0.48
        self._g = 9.81
        self._L = 0.5

        self.debug = debug
        if debug:
            self.footstep = [] # List of all computed ftesp in the queue of contact for debug purpose

    def compute_footstep(self, queue_cs, q, vq, bvref, timeline, selected_surfaces):
        """ Run the queue in reverse order and update the position for each Contact Schedule (CS)
        following the Raibert heuristic, depending on current linear velocity and both desired angular and
        linear velocty.

        Args :
            - queue_cs (list): List of CS.
            - q (array x19): Current state of the robot.
            - vq (array x18): Linear and angular current velocity.
            - bvref (array x6): Linear and angular desired velocities.
            - timeline (int): timeline in CS.
            - selected_surfaces (dict): Dictionary containing the incoming footstep surfaces.

        Returns:
            - (array 3x4): Target for the incoming footseps.
        """
        if q.shape[0] != 19:
            raise ArithmeticError(
                "Current state q should be an array of size 19 [pos (x3), quaternion (x4), joint (x12)]")
        if vq.shape[0] != 18:
            raise ArithmeticError(
                "Current velocity vq should be an array of size 18 [lin vel (x3), ang vel (x3), joint vel(x12)]")
        if bvref.shape[0] != 6:
            raise ArithmeticError("Reference velocity should be an array of size 6 [lin vel (x3), ang vel (x3)]")

        # Update current feet position
        self._update_current_state(q, vq)

        # Update position for each CS in the queue.
        return self.update_position(queue_cs, q, vq, bvref, timeline, selected_surfaces)

    def update_position(self, queue_cs, q, bv, bvref, timeline_, selected_surfaces):
        """ Update the position for a contact schedule.

        Args:
            - queue_cs (list): List of CS.
            - q (array x19): Current state of the robot.
            - vq (array x18): Linear and angular current velocity.
            - bvref (array x6): Linear and angular desired velocities.
            - timeline (int): timeline in CS.
            - selected_surfaces (dict): Dictionary containing the incoming footstep surfaces.

        Returns:
            - (array 3x4): Target for the incoming footseps.
        """
        if self.debug:
            self.footstep.clear()
            self.footstep = [[],[],[],[]]
            for j in range(4):
                self.footstep[j].append(copy.deepcopy(self._current_position[:,j].tolist()))
        # Get current orientation of the robot
        rpy = pin.rpy.matrixToRpy(pin.Quaternion(q[3:7]).toRotationMatrix())
        Rz = pin.rpy.rpyToMatrix(np.array([0., 0., rpy[2]]))  # Yaw rotation matrix
        Rxy = pin.rpy.rpyToMatrix(np.array([rpy[0], rpy[1], 0.]))  # Roll/Pitch rotation matrix

        q_tmp = q[:3]  # Tmp state, z = 0.
        q_tmp[2] = 0.
        P0 = copy.deepcopy(self._current_position)
        V0 = self._current_velocities.copy()
        timeline = timeline_
        counter = 0

        foot_timeline = [0, 0, 0, 0]
        target_footstep = np.zeros((3,4))
        for cs in reversed(queue_cs):

            for c in range(cs.C):
                name = cs.contactNames[c]
                j = self._contactNames.index(name)
                phases = cs.phases[c]
                N_phase = len(phases)
                if N_phase == 3:
                    T_stance = (phases[0].T + phases[2].T) * cs.dt
                    active_phase = phases[0]
                    inactive_phase = phases[1]

                    # Displacement following the reference velocity compared to current position
                    if active_phase.T + inactive_phase.T - timeline > 0:  # case 1 & 2
                        dt_ = (active_phase.T + inactive_phase.T - timeline) * cs.dt
                        if bvref[5] > 10e-5:
                            dx_ = (bvref[0] * np.sin(bvref[5] * dt_) + bvref[1] *
                                   (np.cos(bvref[5] * dt_) - 1.0)) / bvref[5]
                            dy_ = (bvref[1] * np.sin(bvref[5] * dt_) - bvref[0] *
                                   (np.cos(bvref[5] * dt_) - 1.0)) / bvref[5]
                            yaw = bvref[5] * dt_
                            Rz_tmp = pin.rpy.rpyToMatrix(np.array([0., 0., yaw]))
                        else:
                            dx_ = bvref[0] * dt_
                            dy_ = bvref[1] * dt_
                            Rz_tmp = np.identity(3)

                        q_dxdy = np.array([dx_, dy_, 0.])
                        heuristic = self.compute_heuristic(bv, bvref, Rxy, T_stance, name,
                                                           feedback_term=False)  # without feedback term
                        footstep = Rz @ Rz_tmp @ heuristic + q_tmp + Rz @ q_dxdy

                        P_ = np.identity(3)
                        q_ = np.zeros(3)
                        sf = selected_surfaces.get(name)[foot_timeline[j]]
                        G_ = sf.A
                        h_ = sf.b - sf.A @ footstep
                        delta_x = quadprog_solve_qp(P_, q_, G_, h_)
                        footstep_optim = footstep + delta_x
                        if foot_timeline[j] == 0:
                            target_footstep[:,self._contact_names_SL1M.index(name)] = footstep_optim

                        foot_timeline[j] += 1

                        # else: # if heightmap
                        #     footstep_optim = footstep
                        #     footstep_optim[2] = self.heightmap.get_height(footstep[0], footstep[1])

                        # With feedback term
                        # heuristic_fb = self.compute_heuristic(bv, bvref, Rxy, T_stance, name , feedback_term = True) # with feedback term
                        # footstep_fb = Rz @ Rz_tmp @ heuristic + q_tmp + Rz @ q_dxdy
                        # optimVector.append(OptimData(0,name, selected_surfaces.get(name),heuristic, Rz_tmp ))

                        # Update the trajectory
                        if active_phase.T - timeline >= 0:
                            t0 = 0.
                            V0[:, j] = np.zeros(3)
                        else:
                            t0 = timeline - active_phase.T

                        if t0 <= inactive_phase.T /2:
                            phases[1].trajectory.update(P0[:, j], V0[:,j], footstep_optim, t0 * cs.dt, True)

                        P0[:, j] = footstep_optim
                        V0[:, j] = np.zeros(3)

                        if self.debug:
                            self.footstep[j].append(footstep_optim.tolist())

                    else:  # case 3
                        V0[:, j] = np.zeros(3)
                        if self.debug:
                            self.footstep[j].append(None)

                else:
                    raise ArithmeticError("Problem nombre de phases inside CS.")

            timeline = 0.
            counter += 1

        return target_footstep

    def compute_heuristic(self, bv, bvref, Rxy, T_stance, name, feedback_term=True):
        """ Compute heuristic position in base frame
        """
        footstep = np.zeros(3)

        # Add symmetry term
        footstep += T_stance * 0.5 * bvref[:3]

        # Add feedback term
        footstep += self._k_feedback * bv[:3]
        if feedback_term:
            footstep += -self._k_feedback * bvref[:3]

        #  Add centrifugal term
        cross = np.array([bv[1] * bvref[5] - bv[2] * bvref[4], bv[2] * bvref[3] - bv[0] * bvref[5], 0.0])
        footstep += 0.5 * np.sqrt(self._href / self._g) * cross

        # Legs have a limited length so the deviation has to be limited
        footstep[0] = min(footstep[0], self._L)
        footstep[0] = max(footstep[0], -self._L)
        footstep[1] = min(footstep[1], self._L)
        footstep[1] = max(footstep[1], -self._L)

        # Add shoulders, Yaw axis taken into account later
        j = self._contactNames.index(name)
        footstep += Rxy @ self._offsets_feet[:, j]

        # Remove Z component (working on flat ground)
        footstep[2] = 0.

        return footstep

    def _update_current_state(self, q, vq):
        """ Get current feet positions and velocities by forward kinematics.

        Args:
            - q (array x19): Current state of the robot.
            - bv (array x18): Linear and angular current velocity.
        """
        pin.forwardKinematics(self._model, self._data, q, vq)
        for i, name in enumerate(self._contactNames):
            frame_id = self._model.getFrameId(name)
            oMf = pin.updateFramePlacement(self._model, self._data, frame_id)
            v = pin.getFrameVelocity(self._model, self._data, frame_id)
            self._current_position[:, i] = oMf.translation[:]
            self._current_velocities[:, i] = v.linear[:]

        return 0


if __name__ == "__main__":
    """ Run a simple example of Footstepplanner.
    """
    import os
    from example_robot_data.robots_loader import ANYmalLoader
    from walkgen.GaitManager import GaitManager

    # Reference velocity
    bvref = np.zeros(6)
    bvref[5] = 0.06
    bvref[0] = 0.06

    # Timeline
    timeline = 10

    # Heightmap path
    heightmap_path = os.getcwd() + "/data/lab_scene.dat"

    # Load Anymal model to get the current feet position by forward kinematic.
    ANYmalLoader.free_flyer = True
    anymal = ANYmalLoader().robot

    # FootStepPlanner
    foostep_planner = FootStepPlanner(anymal.model, anymal.q0, heightmap_path)

    cs1, cs2 = dict(), dict()
    cs1["LH_FOOT"] = pin.SE3(np.eye(3), np.array([-0.34, 0.33, 0.0083]))
    cs1["LF_FOOT"] = pin.SE3(np.eye(3), np.array([0.34, 0.23, 0.0083]))
    cs1["RH_FOOT"] = pin.SE3(np.eye(3), np.array([-0.34, -0.23, 0.0083]))
    cs1["RF_FOOT"] = pin.SE3(np.eye(3), np.array([0.34, -0.23, 0.0083]))
    cs2["LH_FOOT"] = pin.SE3(np.eye(3), np.array([0.07, 0.2, 0.0083]))
    cs2["LF_FOOT"] = pin.SE3(np.eye(3), np.array([0.67, 0.2, 0.0083]))
    cs2["RH_FOOT"] = pin.SE3(np.eye(3), np.array([0.07, -0.45, 0.0083]))
    cs2["RF_FOOT"] = pin.SE3(np.eye(3), np.array([0.67, -0.2, 0.0083]))

    gait_manager = GaitManager(anymal.model, anymal.q0)

    # Initial selected surface
    A = [[-1.0000000, 0.0000000, 0.0000000], [0.0000000, -1.0000000, 0.0000000], [0.0000000, 1.0000000, 0.0000000],
         [1.0000000, 0.0000000, 0.0000000], [0.0000000, 0.0000000, 1.0000000], [-0.0000000, -0.0000000, -1.0000000]]

    b = [1.3946447, 0.9646447, 0.9646447, 0.2, 0.1, -0.1]

    vertices = [[-1.3946447276978748, 0.9646446609406726, 0.0], [-1.3946447276978748, -0.9646446609406726, 0.0],
                [0.5346445941834704, -0.9646446609406726, 0.0], [0.5346445941834704, 0.9646446609406726, 0.0]]

    sf = Surface(np.array(A), np.array(b), np.array(vertices).T)

    selected_surfaces = dict()  # Mimic asynchronous behaviour
    for foot in range(4):
        selected_surfaces[foostep_planner._contactNames[foot]] = [sf, sf, sf] # horizon lenght of the Surface is 3.
    foostep_planner.compute_footstep(gait_manager.get_cs(), anymal.q0, anymal.v0, bvref, timeline, selected_surfaces)

    # TODO Add vizualisation.
