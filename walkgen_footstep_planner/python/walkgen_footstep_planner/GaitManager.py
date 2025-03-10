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
import copy
from walkgen_footstep_planner.FootStepTrajectoryBezier import FootStepTrajectoryBezier
from caracal import ContactPhase, ContactSchedule
from walkgen_footstep_planner.params import FootStepPlannerParams


class GaitManager:
    """ Gait manager. Add and remove contact schedule from the queue
    depending on the current timeline and the horizon length.
    Get informations on the gait to trigger SL1M at the right time.
    """
    def __init__(self, model, q, params=None):
        """Initialize the gait management.

        Args:
            - model (pin.model): Pinocchio robot model.
            - q (Array x19): State of the robot.
            - params (FootStepPlannerParams): parameter class.
        """
        if params != None:
            self._params = copy.deepcopy(params)
        else:
            self._params = FootStepPlannerParams()

        # Create the model and data for forward simulation
        self._model = model
        self._data = self._model.createData()

        # Gait parameters
        self._typeGait = self._params.typeGait
        self._dt = self._params.dt
        self._N_ss = self._params.N_ss
        self._N_ds = self._params.N_ds
        self._N_uds = self._params.N_uds
        self._N_uss = self._params.N_uss
        self._nsteps = self._params.nsteps
        self._stepHeight = self._params.stepHeight

        lf, lh, rf, rh = "LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"
        self._contactNames = [lf, lh, rf, rh]
        # Contact order name for SL1M
        self._contact_names_SL1M = [lf, rf, lh, rh]

        # SurfacePlanner parameters
        self._N_phase_return = self._params.N_phase_return

        pin.forwardKinematics(self._model, self._data, q)
        pin.updateFramePlacements(self._model, self._data)

        self.cs0, self.cs1 = dict(), dict()
        self.cs0["LH_FOOT"] = self._data.oMf[self._model.getFrameId("LH_FOOT")]
        self.cs0["LF_FOOT"] = self._data.oMf[self._model.getFrameId("LF_FOOT")]
        self.cs0["RH_FOOT"] = self._data.oMf[self._model.getFrameId("RH_FOOT")]
        self.cs0["RF_FOOT"] = self._data.oMf[self._model.getFrameId("RF_FOOT")]
        self.cs1["LH_FOOT"] = self._data.oMf[self._model.getFrameId("LH_FOOT")]
        self.cs1["LF_FOOT"] = self._data.oMf[self._model.getFrameId("LF_FOOT")]
        self.cs1["RH_FOOT"] = self._data.oMf[self._model.getFrameId("RH_FOOT")]
        self.cs1["RF_FOOT"] = self._data.oMf[self._model.getFrameId("RF_FOOT")]

        self.gait_generator = QuadrupedalGaitGenerator()
        if self._typeGait == "trot":
            self._initial_cs = copy.deepcopy(
                self.gait_generator.trot(contacts=[self.cs0, self.cs1],
                                         N_ds=50,
                                         N_ss=self._N_ss,
                                         N_uss=self._N_uss,
                                         N_uds=self._N_uds,
                                         stepHeight=self._stepHeight,
                                         startPhase=True,
                                         endPhase=False))
            self._default_cs = copy.deepcopy(
                self.gait_generator.trot(contacts=[self.cs0, self.cs1],
                                         N_ds=self._N_ds,
                                         N_ss=self._N_ss,
                                         N_uss=self._N_uss,
                                         N_uds=self._N_uds,
                                         stepHeight=self._stepHeight,
                                         startPhase=False,
                                         endPhase=False))
        elif self._typeGait == "walk":
            self._initial_cs = copy.deepcopy(
                self.gait_generator.walk(contacts=[self.cs0, self.cs1],
                                         N_ds=100,
                                         N_ss=self._N_ss,
                                         N_uss=self._N_uss,
                                         N_uds=self._N_uds,
                                         stepHeight=self._stepHeight,
                                         startPhase=True,
                                         endPhase=False))
            self._default_cs = copy.deepcopy(
                self.gait_generator.walk(contacts=[self.cs0, self.cs1],
                                         N_ds=self._N_ds,
                                         N_ss=self._N_ss,
                                         N_uss=self._N_uss,
                                         N_uds=self._N_uds,
                                         stepHeight=self._stepHeight,
                                         startPhase=False,
                                         endPhase=False))
        else:
            raise SyntaxError("Unknown gait type in the config file. Try Trot or Walk.")

        if self._params.horizon is None:
            # horizon of one gait.
            self._horizon = copy.deepcopy(self._default_cs.T)
        else:
            self._horizon = self._params.horizon

        self._timeline = 0  # Current timeline
        self._new_step = False
        self._is_first_gait = True
        self._queue_cs = []  # Queue of contact
        self._surface_planner_gait = None

        # Send new step flag only 1 over n_new_steps :
        self._ratio_nsteps = 1
        # self._ratio_nsteps = 2 # trot in simu with a lot of surfaces
        self._new_step_counter = 0

        # Add initial CS (whith longer stand phase to warm-up the MPC)
        self._initial_cs.updateSwitches()
        self._queue_cs.append(self._initial_cs)
        # Add default CS depending on the horizon length.
        print("gait manager horizon : ", self._horizon)
        T_global = self._initial_cs.T
        while T_global < self._horizon:
            cs = copy.deepcopy(self._default_cs)
            cs.updateSwitches()
            self._queue_cs.insert(0, cs)
            T_global += cs.T
        # Checking erros with the horizon length.
        if not self.check_switching_nodes():
            raise ArithmeticError(
                "The horizon length is not compatible with the gait parameters. There cannot be more than 5 switching nodes at the same time in the horizon. "
            )

        if not self.check_returned_phases():
            raise AttributeError(
                "There cannot be more phases of contact in the horizon planned than the number of contact returned by the ConvexPatch Planner."
            )

        # Initialize switches list
        self.initialize_switches(self._default_cs)

    def check_switching_nodes(self):
        """ Run the contact queue of contact and detect if the horizon length
        is not appropriate. There cannot be more than 5 switching nodes in the
        horizon.

        Args:
            - horizon (int): Horizon length.

        Returns:
            - params (bool): Length is compatible or not.
        """
        queue_cs = [self._default_cs, self._default_cs, self._default_cs]
        checked = True
        cs_index = 0
        L = []  # List of switches
        for cs in reversed(queue_cs):
            for key in cs.switches.keys():
                L.append(cs_index + key)
            cs_index += cs.T

        L.sort()
        for i in range(len(L) - 5):
            if L[i + 5] - L[i] < self._horizon:
                checked = False
        return checked

    def check_returned_phases(self):
        """ Check whether the number of phases returned by the ConvexPatch planner is compatible with the
        horizon. There cannot be more phases of contact in the horizon planned than the number of contact
        returned by the ConvexPatch Planner. A convex patch needs to be associated for each foot in each phase.
        """
        # For now, assume each foot move one time by gait.
        # Conservative check, should be more precise.
        checked = True
        remain = int(self._horizon % self._default_cs.T > 0)
        nb_phases = self._horizon // self._default_cs.T + 1
        if remain + nb_phases > self._N_phase_return:
            checked = False
        return checked

    def initialize_switches(self, cs):
        """ Initialize the dictionnary containing the switches that occur in the gait to trigger SL1M at the
        beginning of each new footstep.

        Args:
            - cs (ContactSchedule): ContactSchedule object.
        """
        switches = dict()
        for c in range(cs.C):
            phases = cs.phases[c]
            N_phase = len(phases)
            phase_index = 0.
            for p in range(0, N_phase, 2):
                T_active = phases[p].T
                T_inactive = 0
                if p + 1 < N_phase:
                    T_inactive = phases[p + 1].T
                    switch_inactive = phase_index + T_active - 1
                    switch_active = phase_index + T_active + T_inactive - 1
                    if switch_active > 0.:
                        if not switch_active in switches:
                            if (T_active + T_inactive - 1) == cs.T - 1:
                                switches[switch_active] = self._evaluate_config(cs, 0)
                            else:
                                switches[switch_active] = self._evaluate_config(cs, T_active + T_inactive - 1)
                    if switch_inactive > 0.:
                        if not switch_inactive in switches:
                            switches[switch_inactive] = self._evaluate_config(cs, T_active - 1)
                phase_index += T_active + T_inactive

        s_list = np.sort([t for t in switches])
        for t in s_list:
            # Remove 4 feet on the ground, useless for SL1M.
            if switches[t] == [1., 1., 1., 1.]:
                switches.pop(t)

        # Usefull list to get quickly the current configuration
        list_sorted = np.sort([t for t in switches])
        self._timelines_list = list_sorted.tolist()
        self._timelines_array_double = np.concatenate([list_sorted, list_sorted])
        if self._typeGait == "trot":
            self._n_gait = 2
        elif self._typeGait == "walk":
            self._n_gait = 4
        else:
            raise SyntaxError("Unknown gait type in the config file. Try Trot or Walk.")

        self.switches = switches

    def get_coefficients(self):
        """ Get the coefficients of the Queue of contact.

        Returns:
            - params1 (list): List of list of tuple (matrix, float) with the coefficients for each foot and initial time.
            Example:    Ax = [Ax0, Ax1, Ax2 ... Axn] on x-axis
                        mat_coeff = [Ax.T, Ay.T, Az.T], size degree x 3
                        params1 = [  current _CS,  next_CS,  ...   last_CS_queue ]
                        current_CS = [Foot1, Foot2, Foot3, Foot4]
                        Foot1 = [Matrix_phase_1, Matrix_phase_2 ...]
        """
        coeffs = []
        for cs in reversed(self._queue_cs):
            cs_list = []  # For one CS, list of foot
            for c in range(cs.C):
                foot_list = []  # For one foot, list of matrix containing the coefficients
                phases = cs.phases[c]
                N_phase = len(phases)
                for p in range(0, N_phase, 2):
                    if p + 1 < N_phase:  # otherwise no inactive phase
                        foot_list.append(
                            (phases[p + 1].trajectory.get_t0(), phases[p + 1].trajectory.get_coefficients()))
                cs_list.append(foot_list)
            coeffs.append(cs_list)
        return coeffs

    def update(self):
        """ Update the internal queue of contact schedule.

        Returns:
            - params1 (bool): True if a new gait has been added in the queue.
        """
        addContact = False
        self._new_step = False
        for s in range(self._nsteps):
            self._timeline += 1

            # Reset the timeline when it has been reached the current contact schedule
            if self._timeline == self._queue_cs[-1].T:
                # Remove the executed contact schedule and reset the timeline
                self._timeline = 0
                self._queue_cs.pop()
                self._is_first_gait = False

            # Add a contact schedule to the queue if necessary
            count = 0
            for cs in self._queue_cs:
                count += cs.T
            if count - self._timeline < (self._horizon):
                # gait = self._default_cs
                gait = copy.deepcopy(self._default_cs)
                gait.updateSwitches()
                self._queue_cs.insert(0, gait)
                addContact = True
            if self._is_first_gait:  # Double support phase added for the first gait
                if (self._timeline - self._N_ds) in self.switches:
                    # self._new_step = True
                    self._new_step_counter += 1
                    if self._new_step_counter % self._ratio_nsteps == 0:
                        self._new_step = True
                        self._new_step_counter = 0
                    self._surface_planner_gait = self._retrieve_gait(self._timeline - self._N_ds)
            else:
                if self._timeline in self.switches:
                    # self._new_step = True
                    self._new_step_counter += 1
                    if self._new_step_counter % self._ratio_nsteps == 0:
                        self._new_step = True
                        self._new_step_counter = 0
                    self._surface_planner_gait = self._retrieve_gait(self._timeline)

        return addContact

    def get_cs(self):
        """ Get ContactSchedule list.
        """
        return self._queue_cs

    def _retrieve_gait(self, timeline):
        """ Compute the current gait matrix on the format : [[1,0,0,1],
                                                              0,1,1,0]]
        depending on the timeline.
        """
        gait = []
        id = self._timelines_list.index(timeline)
        for t in self._timelines_array_double[id:id + self._n_gait]:
            gait.append(self.switches[t])
        return np.array(gait)

    def get_current_gait(self):
        return self._surface_planner_gait

    def get_current_gait_backup(self):
        """ Compute the current gait matrix on the format : [[1,0,0,1],
                                                              0,1,1,0]]
        Usefull for the SurfacePlanner.
        """

        # Current gait :
        gait = []
        # gait_tmp = np.zeros(4)
        if self._timeline < self._queue_cs[-1].T - 1:
            timeline = self._timeline
            cs = self._queue_cs[-1]
        else:
            timeline = 0
            cs = self._queue_cs[-2]

        init_gait = self._evaluate_config(cs, timeline)
        if init_gait != [1, 1, 1, 1]:
            gait.append(init_gait)

        switches = dict()
        index_cs = 0
        for cs in reversed(self._queue_cs):
            for c in range(cs.C):
                phases = cs.phases[c]
                N_phase = len(phases)
                phase_index = index_cs * cs.T
                for p in range(0, N_phase, 2):
                    T_active = phases[p].T
                    T_inactive = 0
                    if p + 1 < N_phase:
                        T_inactive = phases[p + 1].T
                        switch_inactive = phase_index + T_active - 1
                        switch_active = phase_index + T_active + T_inactive - 1
                        if switch_active > self._timeline:
                            if not switch_active in switches:
                                if (T_active + T_inactive - 1) == cs.T - 1:
                                    switches[switch_active] = self._evaluate_config(cs, 0)
                                else:
                                    switches[switch_active] = self._evaluate_config(cs, T_active + T_inactive - 1)
                        if switch_inactive > self._timeline:
                            if not switch_inactive in switches:
                                switches[switch_inactive] = self._evaluate_config(cs, T_active - 1)
                    phase_index += T_active + T_inactive
            index_cs += 1

        s_list = np.sort([t for t in switches])

        for k in range(len(s_list)):
            # Remove 4 feet on the ground, useless for SL1M.
            if switches[s_list[k]] != [1., 1., 1., 1.]:
                gait.append(switches[s_list[k]])

        return np.array(gait)

    def _evaluate_config(self, cs, timeline):
        """ Evaluate the config of the foot for a given contact schedule a time
        timeline.

        Args:
            - cs (ContactSchedule): ContactSchedule object.
            - timeline (int): Time to evaluate the CS.

        Returns:
            - param1 (List x4): List of configs (1 --> in contact, 0 --> swing) in order given
                                by contact_name_SL1M
        """
        gait_tmp = np.zeros(4)
        for c in range(cs.C):
            name = cs.contactNames[c]
            j = self._contact_names_SL1M.index(name)
            phases = cs.phases[c]
            N_phase = len(phases)
            active_phase = phases[0]
            inactive_phase = phases[1]
            if N_phase == 3:
                if active_phase.T - timeline - 1 > 0:  # case 1, inside first Active phase
                    gait_tmp[j] = 1
                elif active_phase.T + inactive_phase.T - timeline - 1 > 0:  # case 2, during inactive phase
                    gait_tmp[j] = 0
                else:
                    gait_tmp[j] = 1  # case 3, inside last Active phase
        return gait_tmp.tolist()

    def is_new_step(self):
        """ Return True if a new flying phase is starting. False otherwise.
        Usefull for the SurfacePlanner, to start a new optimisation.
        """
        return self._new_step


class QuadrupedalGaitGenerator:
    """ Create quadrupedal gait with polynomial swing foot trajectory.
    It uses a custom Trajectory object, independant from Caracal.
    """
    def __init__(self, dt=1e-2, S=4, lf="LF_FOOT", lh="LH_FOOT", rf="RF_FOOT", rh="RH_FOOT"):
        self._dt = dt
        self._S = S
        self._contactNames = [lf, lh, rf, rh]

    def walk(self, contacts, N_ds, N_ss, N_uds=0, N_uss=0, stepHeight=0.15, startPhase=True, endPhase=True):
        N_0 = 0
        if startPhase:
            N_0 = N_ds
        if endPhase:
            N = N_0 + 2 * N_ds + 4 * N_ss - 2 * N_uss - N_uds
        else:
            N = N_0 + N_ds + 4 * N_ss - 2 * N_uss - N_uds
        gait = ContactSchedule(self._dt, N, self._S, self._contactNames)
        lf, lh, rf, rh = self._contactNames
        lfSwingTraj = FootStepTrajectoryBezier(self._dt, N_ss, stepHeight, contacts[0][lf], contacts[1][lf])
        lhSwingTraj = FootStepTrajectoryBezier(self._dt, N_ss, stepHeight, contacts[0][lh], contacts[1][lh])
        rfSwingTraj = FootStepTrajectoryBezier(self._dt, N_ss, stepHeight, contacts[0][rf], contacts[1][rf])
        rhSwingTraj = FootStepTrajectoryBezier(self._dt, N_ss, stepHeight, contacts[0][rh], contacts[1][rh])
        gait.addSchedule(
            lh, [ContactPhase(N_0),
                 ContactPhase(N_ss, trajectory=lhSwingTraj),
                 ContactPhase(N - (N_0 + N_ss))])
        gait.addSchedule(lf, [
            ContactPhase(N_0 + N_ss - N_uss),
            ContactPhase(N_ss, trajectory=lfSwingTraj),
            ContactPhase(N - (N_0 + 2 * N_ss - N_uss))
        ])
        gait.addSchedule(rh, [
            ContactPhase(N_0 + N_ds + 2 * N_ss - N_uss - N_uds),
            ContactPhase(N_ss, trajectory=rhSwingTraj),
            ContactPhase(N - (N_0 + N_ds + 3 * N_ss - N_uss - N_uds))
        ])
        gait.addSchedule(rf, [
            ContactPhase(N_0 + N_ds + 3 * N_ss - 2 * N_uss - N_uds),
            ContactPhase(N_ss, trajectory=rfSwingTraj),
            ContactPhase(N - (N_0 + N_ds + 4 * N_ss - 2 * N_uss - N_uds))
        ])
        return gait

    def trot(self, contacts, N_ds, N_ss, N_uss=0, N_uds=0, stepHeight=0.15, startPhase=True, endPhase=True):
        N_0 = 0
        if startPhase:
            N_0 = N_ds
        if endPhase:
            N = N_0 + N_ds + 2 * N_ss - N_uss
        else:
            N = N_0 + 2 * N_ss - N_uss
        gait = ContactSchedule(self._dt, N, self._S, self._contactNames)
        lf, lh, rf, rh = self._contactNames
        lfSwingTraj = FootStepTrajectoryBezier(self._dt, N_ss, stepHeight, contacts[0][lf], contacts[1][lf])
        lhSwingTraj = FootStepTrajectoryBezier(self._dt, N_ss, stepHeight, contacts[0][lh], contacts[1][lh])
        rfSwingTraj = FootStepTrajectoryBezier(self._dt, N_ss, stepHeight, contacts[0][rf], contacts[1][rf])
        rhSwingTraj = FootStepTrajectoryBezier(self._dt, N_ss, stepHeight, contacts[0][rh], contacts[1][rh])
        gait.addSchedule(
            lh, [ContactPhase(N_0),
                 ContactPhase(N_ss, trajectory=lhSwingTraj),
                 ContactPhase(N - (N_0 + N_ss))])
        gait.addSchedule(
            rf, [ContactPhase(N_0),
                 ContactPhase(N_ss, trajectory=rfSwingTraj),
                 ContactPhase(N - (N_0 + N_ss))])
        gait.addSchedule(lf, [
            ContactPhase(N_0 + N_ss - N_uss),
            ContactPhase(N_ss, trajectory=lfSwingTraj),
            ContactPhase(N - (N_0 + 2 * N_ss - N_uss))
        ])
        gait.addSchedule(rh, [
            ContactPhase(N_0 + N_ss - N_uss),
            ContactPhase(N_ss, trajectory=rhSwingTraj),
            ContactPhase(N - (N_0 + 2 * N_ss - N_uss))
        ])

        return gait


if __name__ == "__main__":

    from example_robot_data.robots_loader import ANYmalLoader
    from walkgen_footstep_planner.params import FootStepPlannerParams

    params = FootStepPlannerParams()
    params.typeGait = "walk"
    # Load Anymal model to get the current feet position by forward kinematic.
    params.N_ds = 90
    params.N_ss = 70
    params.N_uds = 0
    params.N_uss = 0
    params.horizon = 130
    ANYmalLoader.free_flyer = True
    anymal = ANYmalLoader().robot
    gait = GaitManager(anymal.model, anymal.q0, params)
