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

from argparse import ArgumentError
import numpy as np
import pinocchio as pin
import copy
try:
    from time import perf_counter as clock
except ImportError:
    from time import time as clock

from walkgen_footstep_planner.libwalkgen_footstep_planner_pywrap import Surface as Surface_cpp
from walkgen_footstep_planner.tools.Filter import Filter, FilterMean
from walkgen_footstep_planner.tools.Surface import Surface
from walkgen_footstep_planner.tools.optimisation import quadprog_solve_qp
from walkgen_footstep_planner.tools.Filter import Filter


class FootStepPlanner():
    """ FootStepPlanner initialized by URDF model of the environment.
    Use RBPRM as a guide path since the env is available.
    """
    def __init__(self, model, q, params=None, period=0.5, debug=False, RECORDING=True):
        """ Initialize the FootStepPlanner.

        Args:
            - model (pin.model): Pinocchio robot model.
            - q (array x19): Initial state of the robot.
            - heightmap_path (str): Heightmap binary file path.
            - debug (bool): Store the footstep computed for debug purpose.
            - period (float): Gait period [s]
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
        # Reference shoulder position, base frame.
        self._offsets_feet = np.zeros((3, 4))
        # Feet positions, world frame.
        self._current_position = np.zeros((3, 4))
        # Feet velocities, world frame.
        self._current_velocities = np.zeros((3, 4))
        for i, name in enumerate(self._contactNames):
            Id = self._model.getFrameId(name)
            self._current_position[:, i] = self._data.oMf[Id].translation

        # Obtained with anymal.q0 configuration
        self._offsets_feet = np.array([[0.367, -0.367, 0.367, -0.367], [0.2, 0.2, -0.2, -0.2], [0., 0., 0., 0.]])

        self._k_feedback = 0.03
        self._href = 0.48
        self._g = 9.81
        self._L = 0.5
        self._nsteps = params.nsteps
        # range [0.,1.], % of the curve to fix the position of the targetfoostep --> avoid slipping.
        self._stop_heuristic = 5 / 8
        if params.horizon is None:
            self._horizon = period / params.dt
        else:
            self._horizon = params.horizon

        # print("horizon footstepplanner : ", self._horizon)

        self.debug = debug
        if debug:
            self.footstep = []  # List of all computed ftesp in the queue of contact for debug purpose

        # Low pass filter
        if params.typeGait == "walk":
            cutoff = 6 * [1 / (2 * period)]
            cutoff[1] = 1 / (2 * period)
        elif params.typeGait == "trot":
            cutoff = 6 * [1 / (2 * period)]
        else:
            raise ArgumentError("Wrong type of gait. Try walk or trot")

        print("cut off frequency : ", cutoff)
        # self._q_filter = Filter(cutoff, 1/(params.nsteps * params.dt), 3)
        self._q_filter = FilterMean(period, params.nsteps * params.dt)
        self.q_f = np.zeros(18)

        # self._qv_filter = Filter(cutoff, 1/(params.nsteps * params.dt), 2)
        self._qv_filter = FilterMean(period, params.nsteps * params.dt)
        self.qv_f = np.zeros(6)

        self._previous_surfaces = dict()
        dx, dy = 0.5, 0.5
        height = 0.
        epsilon = 10e-6
        A = [[-1., 0., 0.], [0., -1., 0.], [0., 1., 0.], [1., 0., 0.], [0., 0., 1.], [-0., -0., -1.]]
        b = [dx - q[0], dy - q[1], dy + q[1], dx + q[0], height + epsilon, -height + epsilon]
        vertices = [[q[0] - dx, q[1] + dy, height], [q[0] - dx, q[1] - dy, height], [q[0] + dx, q[1] - dy, height],
                    [q[0] + dx, q[1] + dy, height]]
        for foot in range(4):
            self._previous_surfaces[self._contactNames[foot]] = copy.deepcopy(
                Surface(np.array(A), np.array(b),
                        np.array(vertices).T))

        # Quick debug tools
        self.q_save = []
        self.v_save = []
        self.q_filter_save = []

        self._counter_gait = 0

        # Profiling surface planner
        self._RECORDING = RECORDING
        if self._RECORDING:
            self.profiler = {"update_position": [], "foot_position": [], "foot_trajectory": [], "surface_binding": []}
        else:
            self.profiler = {}

    def reset_profiler(self):
        self.profiler["update_position"] = []
        self.profiler["foot_position"] = []
        self.profiler["foot_trajectory"] = []
        self.profiler["surface_binding"] = []

    def get_profiler(self):
        return self.profiler

    def compute_footstep(self, queue_cs, q, vq, bvref, timeline, selected_surfaces, previous_surfaces):
        """ Run the queue in reverse order and update the position for each Contact Schedule (CS)
        following the Raibert heuristic, depending on current linear velocity and both desired angular and
        linear velocty.

        Args :
            - queue_cs (list): List of CS.
            - q (array x19): Current state of the robot.
            - vq (array x18): Linear and angular current velocity.
            - bvref (array x6): Linear and angular desired velocities in base frame.
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

        # Filter quantities
        rpy = pin.rpy.matrixToRpy(pin.Quaternion(q[3:7]).toRotationMatrix())
        q_ = np.zeros(6)
        q_[:3] = q[:3]
        q_[3:] = rpy
        self.q_f = self._q_filter.filter(q_)

        self.qv_f = self._qv_filter.filter(vq[:6])

        # Quick debug tools
        # q_save = [q[0], q[1], q[3], rpy[0], rpy[1], rpy[2]]
        # self.q_save.append(q_save)
        # self.v_save.append(vq[:6])
        # self.q_filter_save.append(q_filter)
        # np.save("/home/thomas_cbrs/Desktop/edin/tmp/memmo_anymal_test/CoM_analysis/q_9070", np.array(self.q_save))
        # np.save("/home/thomas_cbrs/Desktop/edin/tmp/memmo_anymal_test/CoM_analysis/v_9070", np.array(self.v_save))
        # np.save("/home/thomas_cbrs/Desktop/edin/tmp/memmo_anymal_test/CoM_analysis/q_filter_9070", np.array(self.q_filter_save))

        # Update position for each CS in the queue.
        return self.update_position(queue_cs, self.q_f.copy(), vq[:6].copy(), bvref.copy(), timeline,
                                    selected_surfaces, previous_surfaces)

    def update_position(self, queue_cs, q, bv, bvref, timeline_, selected_surfaces, previous_surfaces):
        """ Update the position for a contact schedule.

        Args:
            - queue_cs (list): List of CS.
            - q (array x6): Current state of the robot (position + orientation(rpy))
            - vq (array x6): Linear and angular current velocity.
            - bvref (array x6): Linear and angular desired velocities.
            - timeline (int): timeline in CS.
            - selected_surfaces (dict): Dictionary containing the incoming footstep surfaces.

        Returns:
            - (array 3x4): Target for the incoming footseps.
        """
        ti = clock()
        if self.debug:
            self.footstep.clear()
            self.footstep = [[], [], [], []]
            for j in range(4):
                self.footstep[j].append(copy.deepcopy(self._current_position[:, j].tolist()))

        if timeline_ == 0:
            self._counter_gait += 1
            print("counter gait : ", self._counter_gait)

        # Get current orientation of the robot
        rpy = q[3:]
        Rz = pin.rpy.rpyToMatrix(np.array([0., 0., rpy[2]]))  # Yaw rotation matrix
        # Roll/Pitch rotation matrix
        Rxy = pin.rpy.rpyToMatrix(np.array([rpy[0], rpy[1], 0.]))

        q_tmp = q[:3]
        q_tmp[2] = 0.

        P0 = copy.deepcopy(self._current_position)
        V0 = self._current_velocities.copy()
        timeline = timeline_
        cs_index = 0
        foot_timeline = [0, 0, 0, 0]
        target_footstep = np.zeros((3, 4))
        for cs in reversed(queue_cs):
            if cs_index <= self._horizon + 2:
                for c in range(cs.C):
                    name = cs.contactNames[c]
                    j = self._contactNames.index(name)
                    phases = cs.phases[c]
                    N_phase = len(phases)
                    if N_phase == 3:
                        T_stance = (phases[0].T + phases[2].T) * cs.dt
                        active_phase = phases[0]
                        inactive_phase = phases[1]

                        if cs_index + active_phase.T - timeline <= self._horizon + 2:
                            # Displacement following the reference velocity compared to current position
                            if active_phase.T + inactive_phase.T - timeline > 0:  # case 1 & 2
                                t1 = clock()
                                if abs(bvref[5]) > 10e-3:
                                    dt_ = (cs_index + active_phase.T + inactive_phase.T - timeline) * cs.dt
                                    dx_ = (bvref[0] * np.sin(bvref[5] * dt_) + bvref[1] *
                                           (np.cos(bvref[5] * dt_) - 1.0)) / bvref[5]
                                    dy_ = (bvref[1] * np.sin(bvref[5] * dt_) - bvref[0] *
                                           (np.cos(bvref[5] * dt_) - 1.0)) / bvref[5]
                                    dt_ = (cs_index + active_phase.T + inactive_phase.T) * cs.dt
                                    yaw = bvref[5] * dt_
                                    Rz_tmp = pin.rpy.rpyToMatrix(np.array([0., 0., yaw]))
                                else:
                                    dt_ = (cs_index + active_phase.T + inactive_phase.T - timeline) * cs.dt
                                    dx_ = bvref[0] * dt_
                                    dy_ = bvref[1] * dt_
                                    Rz_tmp = np.identity(3)

                                q_dxdy = np.array([dx_, dy_, 0.])
                                heuristic = self.compute_heuristic(bv, bvref, Rxy, T_stance, name,
                                                                   feedback_term=False)  # without feedback term
                                footstep = np.dot(Rz, np.dot(Rz_tmp, heuristic)) + q_tmp + np.dot(Rz, q_dxdy)

                                P_ = np.identity(3)
                                q_ = np.zeros(3)
                                sf = selected_surfaces.get(name)[foot_timeline[j]]
                                G_ = sf.A
                                h_ = sf.b - np.dot(sf.A, footstep)
                                delta_x = quadprog_solve_qp(P_, q_, G_, h_)
                                footstep_optim = footstep + delta_x
                                t2 = clock()
                                if self._RECORDING:
                                    self.profiler["foot_position"].append(t2 - t1)
                                if foot_timeline[j] == 0:
                                    target_footstep[:, self._contact_names_SL1M.index(name)] = footstep_optim

                                # else: # if heightmap
                                #     footstep_optim = footstep
                                #     footstep_optim[2] = self.heightmap.get_height(footstep[0], footstep[1])

                                # With feedback term
                                # heuristic_fb = self.compute_heuristic(bv, bvref, Rxy, T_stance, name , feedback_term = True) # with feedback term
                                # footstep_fb = Rz @ Rz_tmp @ heuristic + q_tmp + Rz @ q_dxdy
                                # optimVector.append(OptimData(0,name, selected_surfaces.get(name),heuristic, Rz_tmp ))

                                if foot_timeline[j] == 0:
                                    previous_sf = self._previous_surfaces.get(name)
                                else:
                                    previous_sf = selected_surfaces.get(name)[foot_timeline[j] - 1]

                                # Update the trajectory
                                if active_phase.T - timeline >= 0:
                                    t0 = 0.
                                    V0[:, j] = np.zeros(3)
                                else:
                                    t0 = timeline - active_phase.T

                                if self._counter_gait < 3:
                                    footstep_optim[:2] = P0[:2, j]

                                if t0 <= inactive_phase.T * 0.70:
                                    t1 = clock()
                                    surface_init = Surface_cpp(previous_sf.A, previous_sf.b, previous_sf.vertices.T)
                                    surface_end = Surface_cpp(sf.A, sf.b, sf.vertices.T)
                                    t1f = clock()

                                    phases[1].trajectory.update(P0[:, j], V0[:, j], footstep_optim, t0 * cs.dt,
                                                                surface_init, surface_end)
                                    t2 = clock()
                                    if self._RECORDING:
                                        self.profiler["foot_trajectory"].append(t2 - t1)
                                        self.profiler["surface_binding"].append(t1f - t1)

                                # End of the flying phase, register the surface.
                                if t0 >= inactive_phase.T - self._nsteps:
                                    self._previous_surfaces.pop(name)
                                    self._previous_surfaces[name] = copy.deepcopy(sf)

                                P0[:, j] = footstep_optim
                                V0[:, j] = np.zeros(3)

                                foot_timeline[j] += 1

                                if self.debug:
                                    self.footstep[j].append(footstep_optim.tolist())

                            else:  # case 3
                                V0[:, j] = np.zeros(3)
                                if self.debug:
                                    self.footstep[j].append(None)
            else:
                break
            cs_index += cs.T - timeline
            timeline = 0.

        tf = clock()
        if self._RECORDING:
            self.profiler["update_position"].append(tf - ti)

        return target_footstep

    def compute_heuristic(self, bv, bvref, Rxy, T_stance, name, feedback_term=True):
        """ Compute heuristic position in base frame
        """
        footstep = np.zeros(3)
        # beta = 1.
        beta = 1.35

        # Add symmetry term
        # footstep += T_stance * 0.5 * bvref[:3]
        footstep += beta * T_stance * bvref[:3]

        # Add feedback term
        if feedback_term:
            footstep += self._k_feedback * bv[:3]
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
        footstep += np.dot(Rxy, self._offsets_feet[:, j])

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
