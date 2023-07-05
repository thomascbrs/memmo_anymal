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

import numpy as np
from walkgen_surface_processing.tools.geometry_utils import get_normal, order, align_points, getAllSurfacesDict_inner
from .libwalkgen_surface_processing_pywrap import AffordanceLoader


class SurfaceDetector:
    """ Class to extract convex surfaces from an URDF file.
    """

    def __init__(self,
                 filename,
                 orientation_matrix=np.identity(3),
                 translation=np.zeros(3),
                 margin=0.,
                 prefix="environment_",
                 offset_z=0.,
                 margin_aff=0.03,
                 nbTriMargin=0.03,
                 minArea=0.005,
                 affordanceName="Support"):
        """ Initialize the surface detector.

        Args:
            - filename (str): Path of the stl file.
            - orientation (array 3x3): Orientation matrix.
            - translation (array x3): Translation.
            - margin (float): Margin in [m] inside the surfaces.
            - prefix(str): Prefix of obstacle names.
            - offset_z (float): Offset on the z-axis.
            - margin_aff(float) : Margin hpp-affordance params
            - nbTriMargin(float) : nbTriMargin hpp-affordance params
            - minArea(float) : minArea hpp-affordance params
            - affordanceName(String) : affordanceName hpp-affordance params
        """
        self._loader = AffordanceLoader(margin_aff, nbTriMargin, minArea, affordanceName)
        self._loader.load(filename, orientation_matrix, translation)
        names = [prefix + str(k) for k in range(len(self._loader.get_affordances()))]
        affordances = dict(
            zip(names, [(order(align_points(affordance)), get_normal(affordance).tolist())
                        for affordance in self._loader.get_affordances()]))
        self._affordances_reduced = getAllSurfacesDict_inner(affordances, margin, offset_z)

    def extract_surfaces(self):
        """ Extract surfaces from the URDF file.

        Retruns:
            - param1 (dict): Dictionnary type containing all the surfaces ("unique id" : [vertices]).
        """
        return dict(zip(self._affordances_reduced.keys(), [value[0] for value in self._affordances_reduced.values()]))
