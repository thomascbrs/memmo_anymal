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

import copy
import pinocchio
import numpy as np

from walkgen_footstep_planner.libwalkgen_footstep_planner_pywrap import FootTrajectoryBezier
from walkgen_footstep_planner.params import FootStepPlannerParams

class FootStepTrajectoryBezier():

    def __init__(self, dt, N, stepHeight, M_current, M_next, params=None):
        if params is not None:
            self._params = copy.deepcopy(params)
        else:
            self._params = FootStepPlannerParams()

        # Stored swing-trajectory properties
        self._dt = copy.deepcopy(dt)
        self._N = copy.deepcopy(N)
        self._stepHeight = copy.deepcopy(stepHeight)
        self._M_current = copy.deepcopy(M_current)
        self._M_next = copy.deepcopy(M_next)

        # Bezier parameters
        margin = self._params.margin  # Margin [m] wrt to the segment crossed in the surface.
        t_margin = self._params.t_margin  # % of the curve constrained around critical point.
        z_margin = self._params.z_margin  # % of the curve after the critical point.
        N_sample = self._params.N_sample  # Number of sample in the least square optimisation for Bezier coeffs
        N_sample_ineq = self._params.N_sample_ineq  # Number of sample while browsing the curve
        degree = self._params.degree  # Degree of the Bezier curve
        t_swing = self._dt * self._N
        maxHeight = stepHeight

        self._curve = FootTrajectoryBezier()
        self._curve.initialize(margin, t_margin, z_margin, N_sample, N_sample_ineq, degree, t_swing, maxHeight)
        self._curve.create_simple_curve(self._M_current.translation, np.zeros(3), self._M_next.translation, 0.)

    def position(self, k):
        if k >= self._N:
            raise ArithmeticError(
                "Invalid argument: k is bigger than the allocated number of nodes (it should be less than or equal to "
                + str(self._N) + ")")
        # TODO(cmastalli): swing interpolation in SE3
        return pinocchio.SE3(np.eye(3), self._curve.evaluateBezier(0, k * self._dt))

    def velocity(self, k):
        if k >= self._N:
            raise ArithmeticError(
                "Invalid argument: k is bigger than the allocated number of nodes (it should be less than or equal to "
                + str(self._N) + ")")
        # TODO(cmastalli): swing interpolation in motion
        return pinocchio.Motion(self._curve.evaluateBezier(1, k * self._dt), np.zeros(3))

    def update(self, x0, v0, xf, t0, init_surface, end_surface):
        self._curve.update(x0,v0,xf,t0, init_surface, end_surface)
        return 0

    def get_coefficients(self):
        return self._curve.getCoefficients()

    def get_t0(self):
        return self._curve.getT0()
