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
from walkgen.FootStepTrajectory import FootStepTrajectory
import yaml
from caracal import ContactPhase, ContactSchedule, SwingFootTrajectoryPolynomial

class GaitManager:
    """ Gait manager. Add and remove contact schedule from the queue
    depending on the current timeline and the horizon length.
    Get informations on the gait to trigger SL1M at the right time.
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
            self._typeGait = "Trot"  # By default trotting.
            self._dt = 0.01
            self._N_ss = 30
            self._N_ds = 0
            self._nx = 5
            self._ny = 5
            self._nz = 6
        else:
            self._config = yaml.load(open(filename, 'r'), Loader=yaml.FullLoader)
            # Gait parameters
            self._typeGait = self._config["walkgen_params"]["gait"]["type"]
            self._dt = self._config["walkgen_params"]["gait"]["dt"]
            self._N_ss = self._config["walkgen_params"]["gait"]["N_ss"]
            self._N_ds = self._config["walkgen_params"]["gait"]["N_ds"]
            # Trajectory parameters
            self._nx = self._config["walkgen_params"]["trajectory"]["nx"]
            self._ny = self._config["walkgen_params"]["trajectory"]["ny"]
            self._nz = self._config["walkgen_params"]["trajectory"]["nz"]

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
                self.gait_generator.trot(contacts=[cs0, cs1],
                                         N_ds=self._N_ds,
                                         N_ss=self._N_ss,
                                         N_uss=0,
                                         N_uds=0,
                                         endPhase=False))
        elif self._typeGait == "Walk":
            self._default_cs = copy.deepcopy(
                self.gait_generator.walk(contacts=[cs0, cs1],
                                         N_ds=self._N_ds,
                                         N_ss=self._N_ss,
                                         N_uss=0,
                                         N_uds=0,
                                         endPhase=False))
        else:
            raise SyntaxError("Unknown gait type in the config file. Try Trot or Walk.")

        self._horizon = copy.deepcopy(self._default_cs.T)  # horizon of the MPC.
        self._timeline = 0  # Current timeline
        self._queue_cs = [copy.deepcopy(self._default_cs)]  # Intern queue of contact
        self._queue_cs[-1].updateSwitches()  # Update the switches

        lf, lh, rf, rh = "LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"
        self._contact_names_SL1M = [lf, rf, lh, rh]  # Contact order name for SL1M

        # Initialize switches list
        self.initialize_switches(self._default_cs)

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
            if switches[t] == [1., 1., 1., 1.]:  # Remove 4 feet on the ground, useless for SL1M.
                switches.pop(t)

        self.switches = switches

    def get_coefficients(self):
        """ Get the coefficients of the Queue of contact.

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

    def get_default_cs(self):
        """ Create a default Contact Schedule (CS) for Caracal based on the internal CS.

        Returns:
            - params1 (CS): The Caracal Contact Schedule.
        """
        contactNames = [name for name in self._default_cs.contactNames]
        gait = ContactSchedule(self._default_cs.dt, self._default_cs.T, self._default_cs.S_total, contactNames)
        for id, phase in enumerate(self._default_cs.phases):
            gait.addSchedule(contactNames[id], [
                ContactPhase(phase[0].T),
                ContactPhase(phase[1].T,
                             trajectory=SwingFootTrajectoryPolynomial(self._default_cs.dt, phase[1].T, self._nx,
                                                                      self._ny, self._nz)),
                ContactPhase(phase[2].T)
            ])
        gait.updateSwitches()
        return gait

    def update(self, n_step=1):
        """ Update the internal queue of contact schedule.

        Returns:
            - params1 (bool): True if a new gait has been added in the queue.
        """
        addContact = False
        for s in range(n_step):
            self._timeline += 1

            # Reset the timeline when it has been reached the current contact schedule
            if self._timeline == self._queue_cs[-1].T:
                # Remove the executed contact schedule and reset the timeline
                self._timeline = 0
                self._queue_cs.pop()

            # Add a contact schedule to the queue if necessary
            count = 0
            for cs in self._queue_cs:
                count += cs.T

            if count - self._timeline < self._horizon:
                gait = copy.deepcopy(self._default_cs)
                gait.updateSwitches()
                self._queue_cs.insert(0, gait)
                addContact = True
        return addContact

    def get_cs(self):
        """ Get ContactSchedule list.
        """
        return self._queue_cs

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
            if switches[s_list[k]] != [1., 1., 1., 1.]:  # Remove 4 feet on the ground, useless for SL1M.
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
                    gait_tmp[j] = 1  # case 3, inside first Active phase
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
