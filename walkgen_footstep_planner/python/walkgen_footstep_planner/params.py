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

import yaml


class FootStepPlannerParams:

    def __init__(self, filename=None):
        """ Parameters for the reactive footstep planning.

        Args:
            - filename (string): path to the config file.
        """

        # Number of phases returned by the SurfacePlanner
        # (N_phase_return surfaces for each foot)
        self.N_phase_return = 3

        # Gait parameters
        self.typeGait = "walk"  # Only "walk" or "trot" working
        self.dt = 0.01
        self.N_ss = 90
        self.N_ds = 70
        self.N_uds = 0
        self.N_uss = 0
        # (int or None), use the lenght of the gait (None) or a defined horizon (int)
        self.horizon = None
        self.nsteps = 1  # Number of iteration.
        self.stepHeight = 0.15  # Step height [m]

        # Bezier parameters
        self.margin_down = 0.10  # Margin [m] wrt to the segment crossed in the surface.
        self.t_margin_down = 0.2  # % of the curve constrained around critical point.
        self.z_margin_down = 0.2  # % of the curve after the critical point.
        self.margin_up = 0.10  # Margin [m] wrt to the segment crossed in the surface.
        self.t_margin_up = 0.045  # % of the curve constrained around critical point.
        self.z_margin_up = 0.045  # % of the curve after the critical point.

        self.N_sample = 10  # Number of sample in the least square optimisation for Bezier coeffs.
        self.N_sample_ineq = 8  # Number of sample while browsing the curve.
        self.degree = 7  # Degree of the Bezier curve.

        if filename is not None:
            self.parseFile(filename)

    def parseFile(self, filename):
        """ Parse yaml config file to setup the parameters.

        Args:
            - filename (string): path to the config file.
        """
        config = yaml.load(open(filename, 'r'), Loader=yaml.FullLoader)
        self.N_phase_return = config["walkgen_params"]["params"]["N_phase_return"]
        self.typeGait = config["walkgen_params"]["gait"]["type"]
        self.dt = config["walkgen_params"]["gait"]["dt"]
        self.N_ds = config["walkgen_params"]["gait"][self.typeGait]["N_ds"]
        self.N_ss = config["walkgen_params"]["gait"][self.typeGait]["N_ss"]
        self.N_uds = config["walkgen_params"]["gait"][self.typeGait]["N_uds"]
        self.N_uss = config["walkgen_params"]["gait"][self.typeGait]["N_uss"]
        self.horizon = config["walkgen_params"]["gait"]["horizon"]
        self.nsteps = config["walkgen_params"]["gait"]["nsteps"]
        self.stepHeight = config["walkgen_params"]["gait"]["stepHeight"]
        self.margin = config["walkgen_params"]["bezier"]["margin"]
        self.t_margin = config["walkgen_params"]["bezier"]["t_margin"]
        self.z_margin = config["walkgen_params"]["bezier"]["z_margin"]
        self.N_sample = config["walkgen_params"]["bezier"]["N_sample"]
        self.N_sample_ineq = config["walkgen_params"]["bezier"]["N_sample_ineq"]
        self.degree = config["walkgen_params"]["bezier"]["degree"]
