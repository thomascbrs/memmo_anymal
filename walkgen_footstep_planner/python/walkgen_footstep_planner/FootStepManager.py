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

import walkgen_footstep_planner.FootStepPlanner as FootStepPlanner
import walkgen_footstep_planner.GaitManager as GaitManager
from walkgen_footstep_planner.tools.Surface import Surface
import copy
import numpy as np
from walkgen_footstep_planner.params import FootStepPlannerParams
from caracal import QuadrupedalGaitGenerator


class FootStepManager:
    """ Wrapper class to manage the footstep planning of the Walkgen library.
    """

    def __init__(self, model, q, params=None, debug=False):
        """Initialize the management.

        Args:
            - model (pin.model): Pinocchio robot model.
            - q (Array x19): State of the robot in world frame.
            - params (FootStepPlannerParams): parameter class.
        """
        if params != None:
            self._params = copy.deepcopy(params)
        else:
            self._params = FootStepPlannerParams()

        self._gait_manager = GaitManager(model, q, params)  # Gait manager

        self._N_phase_return = self._params.N_phase_return
        self._typeGait = self._params.typeGait
        self._dt = self._params.dt
        self._N_ss = self._params.N_ss
        self._N_ds = self._params.N_ds
        self._N_uss = self._params.N_uss
        self._N_uds = self._params.N_uds
        self._nsteps = self._params.nsteps
        self._stepHeight = self._params.stepHeight

        lf = "LF_FOOT"
        lh = "LH_FOOT"
        rf = "RF_FOOT"
        rh = "RH_FOOT"
        self._contactNames = [lf, lh, rf, rh]

        # Initial selected surface, rectangle of 4dxdy m2 around the initial position.
        dx = 1.5  # Distance on x-axis around the initial position.
        dy = 1.5  # Distance on y-axis around the initial position.
        # Assume 4 feet are on the ground
        lfeet = ["LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"]
        height = np.mean(
            [self._gait_manager.cs0[lfeet[k]].translation[2] for k in range(4)])
        epsilon = 10e-6
        A = [[-1., 0., 0.], [0., -1., 0.], [0., 1., 0.],
             [1., 0., 0.], [0., 0., 1.], [-0., -0., -1.]]
        b = [dx - q[0], dy - q[1], dy + q[1], dx +
             q[0], height + epsilon, -height + epsilon]
        vertices = [[q[0]-dx, q[1]+dy, height], [q[0]-dx, q[1]-dy, height],
                    [q[0]+dx, q[1]-dy, height], [q[0]+dx, q[1]+dy, height]]
        self._init_surface = Surface(
            np.array(A), np.array(b), np.array(vertices).T)

        # Add initial surface to the result structure.
        self._selected_surfaces = dict()
        self._previous_surfaces = dict()
        # Dictionary type :
        # For each foot is associated a list of surfaces, one foot is moving one time for each gait/phase
        for foot in range(4):
            L = []
            for k in range(self._N_phase_return):
                L.append(copy.deepcopy(self._init_surface))
            self._selected_surfaces[self._contactNames[foot]] = L
            self._previous_surfaces[self._contactNames[foot]] = copy.deepcopy(self._init_surface)

        self._new_surfaces = copy.deepcopy(self._selected_surfaces)

        # SL1M quantities needed.
        self._target_foostep = np.zeros((3, 4))
        self._gait_sl1m = np.zeros((4, 4))  # Gait on SL1M format

        # Boolean if a new CS has been added to the gait manager queue.
        self._addContact = False
        self._firstIteration = True
        self._coeffs = []
        self.initialize_default_cs()
        self._foostep_planner = FootStepPlanner(
            model, q, self._params, debug, self._params.dt * self._default_cs.T)  # Foostep planner

    def initialize_default_cs(self):
        """ Create a default contact schedule compatible with Caracal CS.
        contacts, N_ds, N_ss, N_uss=0, N_uds=0, stepHeight=0.15, startPhase=True, endPhase=True
        """
        gait_generator = QuadrupedalGaitGenerator()
        if self._typeGait == "trot":
            self._initial_cs = copy.deepcopy(
                gait_generator.trot(contacts=[self._gait_manager.cs0, self._gait_manager.cs1],
                                    N_ds=self._N_ds,
                                    N_ss=self._N_ss,
                                    N_uss=self._N_uss,
                                    N_uds=self._N_uds,
                                    stepHeight=self._stepHeight,
                                    startPhase=True,
                                    endPhase=False))
            self._default_cs = copy.deepcopy(
                gait_generator.trot(contacts=[self._gait_manager.cs0, self._gait_manager.cs1],
                                    N_ds=self._N_ds,
                                    N_ss=self._N_ss,
                                    N_uss=self._N_uss,
                                    N_uds=self._N_uds,
                                    stepHeight=self._stepHeight,
                                    startPhase=False,
                                    endPhase=False))
        elif self._typeGait == "walk":
            self._initial_cs = copy.deepcopy(
                gait_generator.walk(contacts=[self._gait_manager.cs0, self._gait_manager.cs1],
                                    N_ds=self._N_ds,
                                    N_ss=self._N_ss,
                                    N_uss=self._N_uss,
                                    N_uds=self._N_uds,
                                    stepHeight=self._stepHeight,
                                    startPhase=True,
                                    endPhase=False))
            self._default_cs = copy.deepcopy(
                gait_generator.walk(contacts=[self._gait_manager.cs0, self._gait_manager.cs1],
                                    N_ds=self._N_ds,
                                    N_ss=self._N_ss,
                                    N_uss=self._N_uss,
                                    N_uds=self._N_uds,
                                    stepHeight=self._stepHeight,
                                    startPhase=False,
                                    endPhase=False))
        self._initial_cs.updateSwitches()
        self._default_cs.updateSwitches()

    def get_default_cs(self):
        return self._default_cs

    def get_initial_cs(self):
        return self._initial_cs

    def step(self, q, vq, b_v_ref):
        """ Update the target position and the trajectories.

        Args:
            - q (array x19): Current state of the robot.
            - vq (array x18): Linear and angular current velocity.
            - b_v_ref (array x6): Linear and angular desired velocities.
        """
        self._addContact = False
        # For the first iteration, the gait does not need to be updated.
        if self._firstIteration:
            self._firstIteration = False
        else:
            self._addContact = self._gait_manager.update()

        # Get results from optimisation during the previous phase.
        if self._gait_manager.is_new_step():
            # New step beginning, get the surfaces computed by SL1M during the previous flying phase.
            self.update_previous_surfaces()
            self._selected_surfaces = copy.deepcopy(self._new_surfaces)

        # Run Footstepplanner
        target_foostep = self._foostep_planner.compute_footstep(self._gait_manager.get_cs(), q.copy(), vq.copy(),
                                                                b_v_ref, self._gait_manager._timeline,
                                                                self._selected_surfaces, self._previous_surfaces)

        # Check if a new flying phase is starting to trigger SL1M.
        if self._gait_manager.is_new_step():
            self._gait_sl1m = self._gait_manager.get_current_gait()  # Current walking gait

            # Target foosteps for SL1M.
            self._target_foostep = np.zeros((3, 4))
            for j in range(self._gait_sl1m.shape[1]):  # gait in SL1M order
                name = self._foostep_planner._contact_names_SL1M[j]
                if self._gait_sl1m[
                        0, j] == 0.:  # Flying phase has started, the SL1M problem start for the next one (delay)
                    self._target_foostep[:, j] = target_foostep[:, j]
                else:
                    self._target_foostep[:, j] = self._foostep_planner._current_position[:,
                                                                                         self._foostep_planner.
                                                                                         _contactNames.index(name)]

        self._coeffs = self._gait_manager.get_coefficients()

    def update_previous_surfaces(self):
        """ Keep in memory the starting surface for the current flying phases. Usefull to plan trajectory avoidance with Bezier curves.
        """
        for key in self._previous_surfaces.keys():
            self._previous_surfaces[key] = copy.deepcopy(self._selected_surfaces[key][0])

    def update_surfaces(self, surfaces):
        """ Receive a new set of surfaces. Store it until a new flying phase has started
        so that the surface choosen is not modified during a current flying phase.

        Args:
            - surfaces (dict): The set of new surfaces.
        """
        # TODO: Implement a method to adjust the height of the surface according to the state of the robot.
        self._new_surfaces = copy.deepcopy(surfaces)

    def get_target_footstep(self):
        """ Returns the target foostep for SL1M. (SL1M feet order)
        """
        return self._target_foostep

    def get_coeffs(self):
        """ Returns the list containing the updated polynomial coefficients.
        """
        return self._coeffs

    def get_default_cs(self):
        """ Create a default Contact Schedule (CS) for Caracal based on the internal CS.

        Returns:
            - params1 (CS): The Caracal Contact Schedule.
        """
        return self._default_cs

    def is_new_cs_added(self):
        """ Returns boolean if a new contact schedule has been inserted in the queue.
        """
        return self._addContact

    def is_new_flying_phase(self):
        """ Return True if a new flying phase is starting. False otherwise.
        Usefull for the SurfacePlanner, to start a new optimisation.
        """
        return self._gait_manager.is_new_step()

    def get_sl1m_gait(self):
        """ Get the gait under the format [[0,1,1,0],[1,0,0,1]] necessary for SL1M algorithm.
        """
        return self._gait_sl1m

    def get_q_filter(self):
        """ Returns the state filtered. (x 18)
        """
        return self._foostep_planner.q_f


if __name__ == "__main__":
    """ Run a simple example of the FootStepManager wrapper.
    """
    from example_robot_data.robots_loader import ANYmalLoader
    import os

    # Reference velocity
    bvref = np.zeros(6)
    bvref[5] = 0.06
    bvref[0] = 0.06

    filepath = os.path.dirname(os.path.abspath(
        __file__)) + "/config/params.yaml"

    # Load Anymal model to get the current feet position by forward kinematic.
    ANYmalLoader.free_flyer = True
    anymal = ANYmalLoader().robot
    footstep_manager = FootStepManager(anymal.model, anymal.q0, filepath)

    # Initial selected surface
    A = [[-1.0000000, 0.0000000, 0.0000000], [0.0000000, -1.0000000, 0.0000000], [0.0000000, 1.0000000, 0.0000000],
         [1.0000000, 0.0000000, 0.0000000], [0.0000000, 0.0000000, 1.0000000], [-0.0000000, -0.0000000, -1.0000000]]

    b = [1.3946447, 0.9646447, 0.9646447, 0.2, 0.1, -0.1]

    vertices = [[-1.3946447276978748, 0.9646446609406726, 0.0], [-1.3946447276978748, -0.9646446609406726, 0.0],
                [0.5346445941834704, -0.9646446609406726, 0.0], [0.5346445941834704, 0.9646446609406726, 0.0]]

    sf = Surface(np.array(A), np.array(b), np.array(vertices).T)

    selected_surfaces = footstep_manager._selected_surfaces
    for k in range(5):
        footstep_manager.step(anymal.q0, anymal.v0, bvref, selected_surfaces)
