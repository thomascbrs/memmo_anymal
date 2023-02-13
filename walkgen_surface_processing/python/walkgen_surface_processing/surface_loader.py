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
import trimesh
from walkgen_surface_processing.tools.geometry_utils import order, apply_margin
from os import listdir
from os.path import isfile, join

class SurfaceLoader:
    """ Class to extract convex surfaces from a folder containing .stl files.
    """

    def __init__(self,
                 folderpath,
                 orientation_matrix=np.identity(3),
                 translation=np.zeros(3),
                 margin=0.,
                 prefix="environment_"):
        """ Load the surfaces.

        Args:
            - folderpath (str): Path of the folder containing the .stl files.
            - orientation (array 3x3): Orientation matrix.
            - translation (array x3): Translation.
            - margin (float): Margin in [m] inside the surfaces.
            - prefix(str): Prefix of obstacle names.
        """
        names = [f for f in listdir(folderpath) if isfile(join(folderpath, f))]
        hmatrix = np.zeros((4,4))
        hmatrix[:3,:3] = orientation_matrix[:,:]
        hmatrix[:3,-1] = translation[:]

        self.all_surfaces = dict()
        self.all_surfaces_reduced = dict()
        for id,file in enumerate(names):
            obj = trimesh.load_mesh(folderpath + file)
            obj.apply_transform(hmatrix)
            vert = order(np.array(obj.vertices))
            filename = prefix + str(id)
            self.all_surfaces[filename] = vert
            self.all_surfaces_reduced[filename] = apply_margin(np.array(vert),margin)

    def extract_surfaces(self):
        """ Extract surfaces from the URDF file.

        Retruns:
            - param1 (dict): Dictionnary type containing all the surfaces ("unique id" : [vertices]).
        """
        return self.all_surfaces_reduced