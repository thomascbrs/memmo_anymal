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


class SurfaceProcessingParams:

    def __init__(self, filename=None):
        """ Parameters for the reactive footstep planning.

        Args:
            - filename (string): path to the config file.
        """
        # Use URDF of the environment or planeseg visualisation.
        self.planeseg = False

        # URDF and heightmap environment without planeseg.
        self.path = os.path.dirname(os.path.abspath(__file__))
        self.urdf = "/data/meshes/lab_scene.urdf"  # Env .stl path
        self.stl = "/data/meshes/lab_scene.stl"  # Env .stl path

        # Planeseg parameters for postprocessing.
        self.n_points = 6  # Maximum Number of points for for each convex surface
        self.margin = 0.06  # Margin in [m] inside the convex surfaces
        # Method to remove the overlapping between surfaces :
        #   1. Run the list of surfaces starting with the lowest.
        #   2. Decomposes the lowest surface in set of convex ones by removing the upper surfaces that overlap it. (Tesselation algorithm).
        #   3. Delate some of the remaining surfaces from the list and continue using one of the following method.
        #       - BASIC (0): Intersect the lowest polygon with all the upper polygon that overlay and keep all remaining surfaces.
        #       - AREA (1): After difference and decomposition with the upper polygons, select only the remining polygon
        #               whose area is superior to min_area arg.
        #       - CONVEX (2): Apply the difference with the convex hull of the upper surfaces that intersect, to avoid
        #                 having small surfaces between the upper surfaces.
        #       - AREA_CONVEX (3): Both 1 and 2 methods. -->
        self.method_id = 3
        # Maximum size of the polygon for the convex decomposition.
        self.poly_size = 10
        # Area under which the remaining surfaces is delated.
        self.min_area = 0.03

        if filename is not None:
            self.parseFile(filename)

    def parseFile(self, filename):
        """ Parse yaml config file to setup the parameters.

        Args:
            - filename (string): path to the config file.
        """
        config = yaml.load(open(filename, 'r'), Loader=yaml.FullLoader)
        self.path = config["walkgen_params"]["world"]["path"]
        if self.path == "":
            self.path = os.path.dirname(os.path.abspath(__file__))
        self.planeseg = config["walkgen_params"]["planeseg"]
        self.urdf = config["walkgen_params"]["world"]["urdf"]
        self.n_points = config["walkgen_params"]["params"]["n_points"]
        self.method_id = config["walkgen_params"]["params"]["method_id"]
        self.poly_size = config["walkgen_params"]["params"]["poly_size"]
        self.min_area = config["walkgen_params"]["params"]["min_area"]
        self.margin = config["walkgen_params"]["params"]["margin"]
