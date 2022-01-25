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
import copy
from walkgen.contact import ContactSchedule, ContactPhase
from walkgen.FootStepTrajectory import FootStepTrajectory
import ctypes

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

    def __init__(self, model, q):
        """Initialize the gait management.

        Args:
            - model (pin.model): Pinocchio robot model.
            - q (Array x19): State of the robot.
        """

        # Create the model and data for forward simulation
        self._model = model
        self._data = self._model.createData()

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
        self._default_cs = copy.deepcopy(
            self.gait_generator.trot(contacts=[cs0, cs1], N_ds=0, N_ss=30, N_uss=0, N_uds=0, endPhase=False))

        # Define the MPC horizon
        self._horizon = copy.deepcopy(self._default_cs.T)
        self._timeline = 0  # Current timeline
        self._queue_cs = [copy.deepcopy(self._default_cs)]
        # Compute switches
        self._queue_cs[-1].updateSwitches()
        # Create data structure
        self._queue_cs_data = [self._create_cs_data(self._queue_cs[-1])]

    def update(self, n_step=1):
        """ Update the queue of contact schedule.
        """
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


class QuadrupedalGaitGenerator:
    """ Create quadrupedal gait with polynomial swing foot trajectory.
    """

    def __init__(self, dt=1e-2, S=4, lf="LF_FOOT", lh="LH_FOOT", rf="RF_FOOT", rh="RH_FOOT"):
        self._dt = dt
        self._S = S
        self._contactNames = [lf, lh, rf, rh]

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