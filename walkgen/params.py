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


class WalkgenParams:

    def __init__(self, filename=None):
        """ Parameters for the reactive footstep planning.

        Args:
            - filename (string): path to the config file.
        """
        self.planeseg = False  # Use URDF of the environment or planeseg visualisation.

        # URDF and heightmap environment without planeseg.
        path = os.path.dirname(os.path.abspath(__file__))
        self.urdf = path + "/data/urdf/lab_scene.urdf"  # Env URDF path
        self.heightmap = path + "/data/lab_scene.dat"  # Heightmap path

        # Planeseg parameters for postprocessing.
        self.n_points = 6  # Maximum number of vertices for the surfaces
        self.method_id = 3  # Method to remove overlapping
        self.poly_size = 10  # Number of maximum point for convex decomposition.
        self.min_area = 0.03  # Minimum area to keep the surfaces

        # SurfacePlanner parameters.
        self.N_phase = 3  # Number of step to proceed (--> N_phase * n_gait step in SL1M)
        self.N_phase_return = 3  # Number of step to return (N_phase_return surfaces for each foot)
        self.com = False  # Optimisation of the CoM

        # Margin in m inside the convex surfaces.
        self.margin = 0.06  # inner surface margin in m

        # Gait parameters
        self.typeGait = "Walk"  # Only "Walk" or "Trot" working
        self.dt = 0.01
        self.N_ss = 20  # 30 for Trot
        self.N_ds = 5  # 0 for Trot
        self.horizon = None # (int or None), use the lenght of the gait (None) or a defined horizon (int)
        self.nsteps = 1 # Number of iteration.

        if filename is not None:
            self.parseFile(filename)

    def parseFile(self, filename):
        """ Parse yaml config file to setup the parameters.

        Args:
            - filename (string): path to the config file.
        """
        config = yaml.load(open(filename, 'r'), Loader=yaml.FullLoader)
        path = os.path.dirname(os.path.abspath(__file__))
        self.planeseg = config["walkgen_params"]["planeseg"]
        self.urdf = path + config["walkgen_params"]["world"]["urdf"]
        self.heightmap = path + config["walkgen_params"]["world"]["heightmap"]
        self.n_points = config["walkgen_params"]["params"]["n_points"]
        self.method_id = config["walkgen_params"]["params"]["method_id"]
        self.poly_size = config["walkgen_params"]["params"]["poly_size"]
        self.min_area = config["walkgen_params"]["params"]["min_area"]
        self.N_phase = config["walkgen_params"]["params"]["N_phase"]
        self.N_phase_return = config["walkgen_params"]["params"]["N_phase_return"]
        self.com = config["walkgen_params"]["params"]["com"]
        self.margin = config["walkgen_params"]["params"]["margin"]
        self.typeGait = config["walkgen_params"]["gait"]["type"]
        self.dt = config["walkgen_params"]["gait"]["dt"]
        self.N_ss = config["walkgen_params"]["gait"]["N_ss"]
        self.N_ds = config["walkgen_params"]["gait"]["N_ds"]
        self.horizon = config["walkgen_params"]["gait"]["horizon"]
        self.nsteps = config["walkgen_params"]["gait"]["nsteps"]