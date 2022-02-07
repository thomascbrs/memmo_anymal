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
from walkgen.contact import ContactSchedule, ContactPhase
from walkgen.FootStepTrajectory import FootStepTrajectory
import ctypes
import yaml

# Mimic the ContactSchedule class to allow ros msg echange between FootStepPlanner and Caracal.

N_DEGREE_X = 5
N_DEGREE_Y = 5
N_DEGREE_Z = 6


class ContactPhaseData(ctypes.Structure):
    """ Ctype structure to exhange contact phase data type.
    """
    C = 4  # number of contacts
    _fields_ = [
        ('T', ctypes.c_int64),  # number of nodes
        ('type', ctypes.c_int64),  # type of contact
        ('nx', ctypes.c_int),
        ('ny', ctypes.c_int),
        ('nz', ctypes.c_int),
        ('coefficients_x', ctypes.c_double * (N_DEGREE_X + 1)),
        ('coefficients_y', ctypes.c_double * (N_DEGREE_Y + 1)),
        ('coefficients_z', ctypes.c_double * (N_DEGREE_Z + 1))
    ]


class ContactScheduleData(ctypes.Structure):
    """ Ctype structure to exhange contact schedule data type.
    """

    C = 4  # number of contacts

    _fields_ = [
        ('dt', ctypes.c_double),  # time step
        ('T', ctypes.c_int64),  # number of nodes
        ('S_total', ctypes.c_int64),  # maximum number of contact phases
        ('C', ctypes.c_int64),  # number of contacts
        ('contactNames', ctypes.c_wchar_p * C),  # contact names
        ('phases', ContactPhaseData * 3 * C)
    ]


class GaitManager:
    """ Gait manager. Add and remove contact schedule from the queue
    depending on the current timeline and the horizon length.
    Works with 2 intern lists.
    - _queue_cs: List of contact schedule.
    - _queue_cs_data: List of contact schedule ctype, allowing data exchanges.
    """

    def __init__(self, model, q, filename=None):
        """Initialize the gait management.

        Args:
            - model (pin.model): Pinocchio robot model.
            - q (Array x19): State of the robot.
            - filename (str): Path to the config file.
        """

        # Create the model and data for forward simulation
        self._model = model
        self._data = self._model.createData()

        if filename is None:
            self._typeGait = "Trot" # By default trotting.
        else:
            self._config = yaml.load(open(filename, 'r'), Loader=yaml.FullLoader)
            self._typeGait = self._config["walkgen_params"]["gait"]["type"]

        pin.forwardKinematics(self._model, self._data, q)
        pin.updateFramePlacements(self._model, self._data)

        cs0, cs1, cs2 = dict(), dict(), dict()
        cs0["LH_FOOT"] = self._data.oMf[self._model.getFrameId("LH_FOOT")]
        cs0["LF_FOOT"] = self._data.oMf[self._model.getFrameId("LF_FOOT")]
        cs0["RH_FOOT"] = self._data.oMf[self._model.getFrameId("RH_FOOT")]
        cs0["RF_FOOT"] = self._data.oMf[self._model.getFrameId("RF_FOOT")]
        cs1["LH_FOOT"] = self._data.oMf[self._model.getFrameId("LH_FOOT")]
        cs1["LF_FOOT"] = self._data.oMf[self._model.getFrameId("LF_FOOT")]
        cs1["RH_FOOT"] = self._data.oMf[self._model.getFrameId("RH_FOOT")]
        cs1["RF_FOOT"] = self._data.oMf[self._model.getFrameId("RF_FOOT")]

        self.gait_generator = QuadrupedalGaitGenerator()
        if self._typeGait == "Trot":
            self._default_cs = copy.deepcopy(
                self.gait_generator.trot(contacts=[cs0, cs1], N_ds=0, N_ss=30, N_uss=0, N_uds=0, endPhase=False))
        elif self._typeGait == "Walk":
            self._default_cs = copy.deepcopy(
                self.gait_generator.walk(contacts=[cs0, cs1], N_ds=5, N_ss=20, N_uss=0, N_uds=0, endPhase=False))
        else:
            raise SyntaxError("Unknown gait type in the config file. Try Trot or gait.")

        # Define the MPC horizon
        self._horizon = copy.deepcopy(self._default_cs.T)
        self._timeline = 0  # Current timeline
        self._queue_cs = [copy.deepcopy(self._default_cs)]
        # Compute switches
        self._queue_cs[-1].updateSwitches()
        # Create data structure
        self._queue_cs_data = [self._create_cs_data(self._queue_cs[-1])]
        # Contact name order for SL1M

        lf = "LF_FOOT"
        lh = "LH_FOOT"
        rf = "RF_FOOT"
        rh = "RH_FOOT"
        self._contact_names_SL1M = [lf, rf, lh, rh]

        # Initialize switches list
        self.initialize_switches(self._default_cs)

    def initialize_switches(self, cs):
        """ Compute the switches that occur in the gait to trigger SL1M at the
        beginning of each new footstep.
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
            if switches[t] == [1.,1.,1.,1.]: # Remove 4 feet on the ground, useless for SL1M.
                switches.pop(t)

        self.switches = switches

    def get_coefficients(self):
        """ Get the coefficients of the Queue of contact.
        Alternative way, to avoid using the complex Data Structure.

        Returns:
            - params1 (list): List of list with the coefficients for each foot.
            Example: [[Ax_0,Ay_0,Az_0,Ax_1 ... , Ax_3, Ay_3,Az_3], ... , [Ax_0,Ay_0,Az_0,Ax_1 ... , Ax_3, Ay_3,Az_3]]

        """
        coeffs = []
        for cs in reversed(self._queue_cs):
            cs_coeff = []
            for c in range(cs.C):
                cs_coeff.append(cs.phases[c][1].trajectory.Ax)
                cs_coeff.append(cs.phases[c][1].trajectory.Ay)
                cs_coeff.append(cs.phases[c][1].trajectory.Az)
            coeffs.append(cs_coeff)
        return coeffs


    def update(self, n_step=1):
        """ Update the queue of contact schedule.

        Returns:
            - params1 (bool): True if a new gait needs to be added in the queue.
        """
        addContact = False
        for s in range(n_step):
            self._timeline += 1

            # Reset the timeline when it has been reached the current contact schedule
            if self._timeline == self._queue_cs[-1].T:
                # Remove the executed contact schedule and reset the timeline
                self._timeline = 0
                self._queue_cs.pop()
                self._queue_cs_data.pop()

            # Add a contact schedule to the queue if necessary
            count = 0
            for cs in self._queue_cs:
                count += cs.T

            if count - self._timeline < self._horizon:
                gait = copy.deepcopy(self._default_cs)
                gait.updateSwitches()
                self._queue_cs.insert(0, gait)
                self._queue_cs_data.insert(0, self._create_cs_data(gait))
                addContact = True
        return addContact


    def get_cs_data(self):
        """ Get ContactSchedule Data type list.
        """
        return self._queue_cs_data

    def get_cs(self):
        """ Get ContactSchedule list.
        """
        return self._queue_cs

    def _create_cs_data(self, cs):
        """ Create Contact Schedule Ctype from Contact schedule object.

        Args:
            - cs (ContactSchedule): ContactSchedule object.

        Returns:
            - (ContactScheduleData): Equivalent Ctype object.
        """
        cs_data = ContactScheduleData()

        # Update the ContactScheduleData
        cs_data.C = cs.C
        for c in range(cs.C):
            cs_data.contactNames[c] = cs.contactNames[c]
        cs_data.dt = cs.dt

        cs_data.T = cs.T
        cs_data.S_total = cs.S_total
        for c in range(cs.C):
            for k in range(3):
                cs_data.phases[c][k].T = cs.phases[c][k].T
                cs_data.phases[c][k].type = cs.phases[c][k].type.value
                if k == 1:
                    cs_data.phases[c][k].is_active = False
                    cs_data.phases[c][k].nx = N_DEGREE_X
                    cs_data.phases[c][k].ny = N_DEGREE_Y
                    cs_data.phases[c][k].nz = N_DEGREE_Z
                    Ax = np.frombuffer(cs_data.phases[c][k].coefficients_x)
                    Ay = np.frombuffer(cs_data.phases[c][k].coefficients_y)
                    Az = np.frombuffer(cs_data.phases[c][k].coefficients_z)
                    Ax[:] = cs.phases[c][k].trajectory.Ax
                    Ay[:] = cs.phases[c][k].trajectory.Ay
                    Az[:] = cs.phases[c][k].trajectory.Az
                else:
                    cs_data.phases[c][k].is_active = True

        return cs_data

    def update_cs_data(self):
        """ Update the Contact Schedule Data coefficients from the Contact Schedule list.
        """
        for id, cs in enumerate(self._queue_cs):
            for c in range(cs.C):
                Ax = np.frombuffer(self._queue_cs_data[id].phases[c][1].coefficients_x)
                Ay = np.frombuffer(self._queue_cs_data[id].phases[c][1].coefficients_y)
                Az = np.frombuffer(self._queue_cs_data[id].phases[c][1].coefficients_z)
                Ax[:] = cs.phases[c][1].trajectory.Ax
                Ay[:] = cs.phases[c][1].trajectory.Ay
                Az[:] = cs.phases[c][1].trajectory.Az

    def get_current_gait(self):
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
        if init_gait != [1,1,1,1]:
            gait.append(init_gait)

        switches = dict()
        index_cs = 0
        for cs in reversed(self._queue_cs):
            for c in range(cs.C):
                phases = cs.phases[c]
                N_phase = len(phases)
                phase_index = index_cs*cs.T
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

        for k in range(len(s_list) ):
            if switches[s_list[k]] != [1.,1.,1.,1.]: # Remove 4 feet on the ground, useless for SL1M.
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
                    gait_tmp[j] = 1 # case 3, inside first Active phase
        return gait_tmp.tolist()

    def is_new_step(self):
        """ Return True if a new flying phase is starting. False otherwise.
        Usefull for the SurfacePlanner, to start a new optimisation.
        """
        if self._timeline in self.switches:
            return True
        else:
            return False


class QuadrupedalGaitGenerator:
    """ Create quadrupedal gait with polynomial swing foot trajectory.
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
        lfSwingTraj = FootStepTrajectory(self._dt, N_ss, stepHeight, contacts[0][lf], contacts[1][lf])
        lhSwingTraj = FootStepTrajectory(self._dt, N_ss, stepHeight, contacts[0][lh], contacts[1][lh])
        rfSwingTraj = FootStepTrajectory(self._dt, N_ss, stepHeight, contacts[0][rf], contacts[1][rf])
        rhSwingTraj = FootStepTrajectory(self._dt, N_ss, stepHeight, contacts[0][rh], contacts[1][rh])
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
        lfSwingTraj = FootStepTrajectory(self._dt, N_ss, stepHeight, contacts[0][lf], contacts[1][lf])
        lhSwingTraj = FootStepTrajectory(self._dt, N_ss, stepHeight, contacts[0][lh], contacts[1][lh])
        rfSwingTraj = FootStepTrajectory(self._dt, N_ss, stepHeight, contacts[0][rf], contacts[1][rf])
        rhSwingTraj = FootStepTrajectory(self._dt, N_ss, stepHeight, contacts[0][rh], contacts[1][rh])
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

    # Load Anymal model to get the current feet position by forward kinematic.
    ANYmalLoader.free_flyer = True
    anymal = ANYmalLoader().robot
    gait = GaitManager(anymal.model, anymal.q0)
