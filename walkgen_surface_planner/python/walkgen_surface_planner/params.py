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
import os


class SurfacePlannerParams:

    def __init__(self, filename=None):
        """ Parameters for the reactive footstep planning.

        Args:
            - filename (string): path to the config file.
        """
        # Number of step to return (N_phase_return surfaces for each foot)
        self.N_phase_return = 3
        self.com = False  # Optimisation of the CoM
        self.horizon = 8 # Number of fsteps optimised. 

        # Get slope of the terrain to rotate SL1M inequalities.
        self.fitsize_x = 10
        self.fitsize_y = 5
        self.fitlength = 0.3 # half-square size centered around position of the robot.
        self.recompute_slope = False # Recompute the slope of the terrain for each contact phase configuration.
        
        # order matters! LF LH RF RH
        self.contact_names =  ["FL_foot","RL_foot","FR_foot","RR_foot"]
        self.shoulder_offsets = [[0.367, 0.2],[0.367, -0.2],[-0.367, 0.2],[-0.367, -0.2]] 

        if filename is not None:
            self.parse_file(filename)

    def parse_file(self, filename):
        """ Parse yaml config file to setup the parameters.

        Args:
            - filename (string): path to the config file.
        """
        config = yaml.load(open(filename, 'r'), Loader=yaml.FullLoader)
        self.N_phase_return = config["walkgen_params"]["params"]["N_phase_return"]
        self.contact_names = config["walkgen_params"]["params"]["contact_names"]
        self.shoulder_offsets = config["walkgen_params"]["params"]["shoulder_offsets"]
        self.com = config["walkgen_params"]["params"]["com"]
        self.horizon = config["walkgen_params"]["params"]["horizon"]
        self.fitsize_x = config["walkgen_params"]["heightmap"]["fitsize_x"]
        self.fitsize_y = config["walkgen_params"]["heightmap"]["fitsize_y"]
        self.fitlength = config["walkgen_params"]["heightmap"]["fitlength"]
        self.recompute_slope = config["walkgen_params"]["heightmap"]["recompute_slope"]
