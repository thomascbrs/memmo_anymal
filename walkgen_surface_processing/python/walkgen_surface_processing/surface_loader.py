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
from walkgen_surface_processing.tools.geometry_utils import order, apply_margin, align_points, process_surfaces
from walkgen_surface_processing.params import SurfaceProcessingParams
from walkgen_surface_processing.tools.SurfaceData import SurfaceData
from os import listdir
from os.path import isfile, join
import copy

class SurfaceLoader:
    """ Class to extract convex surfaces from a folder containing .stl files.
    """

    def __init__(self,
                 folderpath,
                 orientation_matrix=np.identity(3),
                 translation=np.zeros(3),
                 prefix="environment_",
                 params = None,
                 ICRA = False,
                 ):
        """ Load the surfaces.

        Args:
            - folderpath (str): Path of the folder containing the .stl files.
            - orientation (array 3x3): Orientation matrix.
            - translation (array x3): Translation.
            - margin (float): Margin in [m] inside the surfaces.
            - prefix(str): Prefix of obstacle names.
            - params (obj): PArams objects for decompostion.
        """
        if params is not None:
            self._params = copy.deepcopy(params)
        else:
            self._params = SurfaceProcessingParams()

        # Parameters for postprocessing.
        self._n_points = self._params.n_points
        self._method_id = self._params.method_id
        self._poly_size = self._params.poly_size
        self._min_area = self._params.min_area
        self._margin_inner = self._params.margin_inner
        self._margin_outer = self._params.margin_outer
        
        if not ICRA:
            names = [f for f in listdir(folderpath) if isfile(join(folderpath, f))]
            hmatrix = np.zeros((4,4))
            hmatrix[:3,:3] = orientation_matrix[:,:]
            hmatrix[:3,-1] = translation[:]

            self.all_surfaces = dict()
            # self.all_surfaces_reduced = dict()
            for id,file in enumerate(names):
                obj = trimesh.load_mesh(folderpath + file)
                obj.apply_transform(hmatrix)
                vert = order(np.array(obj.vertices))
                filename = prefix + str(id)
                self.all_surfaces[filename] = vert
                # self.all_surfaces_reduced[filename] = apply_margin(np.array(vert),0.)

            # Apply process to filter and decompose the surfaces to avoid overlap and apply a security margin.
            surfaces = [np.array(sf) for sf in self.all_surfaces.values()]
            self.surfaces_processed = process_surfaces(surfaces,
                                                polySize=self._poly_size,
                                                method=self._method_id,
                                                min_area=self._min_area,
                                                margin_inner=self._margin_inner,
                                                margin_outer=self._margin_outer)
        else:
            # /surfaces_g0 --> wooden pallet
            # /surfaces_g1 --> wooden pallets steps
            # /surfaces_g2 --> small blocks
            # /surfaces_g3 --> wood panels 20cm
            # /surfaces_g4 --> crate
            # /surfaces_g5 --> 2 wood panels != height
            # /surfaces_rounded
            # /surfaces_inclined
            self.all_surfaces = dict()
            self.surfaces_processed = []
            # Whole scene
            # folder_names = ["surfaces_g0/","surfaces_g1/","surfaces_g2/","surfaces_g3/","surfaces_g4/","surfaces_g5/","surfaces_rounded/", "surfaces_inclined/"]
            # margins = [[0.01,0.],[0.02,0.05],[0.01,0.03],[0.005,0.03],[0.02,0.05],[0.02,0.05],[0.02,0.],[0.02,0.]] 
            # starting at g2
            # folder_names = ["surfaces_g0/","surfaces_g2/","surfaces_g3/","surfaces_g4/","surfaces_g5/","surfaces_rounded/", "surfaces_inclined/"]
            # margins = [[0.01,0.],[0.01,0.03],[0.005,0.03],[0.02,0.05],[0.02,0.05],[0.03,0.],[0.03,0.]] 
            # # starting at g3
            # folder_names = ["surfaces_g0/","surfaces_g3/","surfaces_g4/","surfaces_g5/","surfaces_rounded/", "surfaces_inclined/"]
            # margins = [[0.01,0.],[0.0,0.06],[0.02,0.05],[0.02,0.05],[0.03,0.],[0.03,0.]] 
            # # starting at g4
            # folder_names = ["surfaces_g0/","surfaces_g4/","surfaces_g5/","surfaces_rounded/", "surfaces_inclined/"]
            # margins = [[0.01,0.],[0.04,0.07],[0.02,0.05],[0.03,0.],[0.03,0.]] 
            # starting at g5
            # folder_names = ["surfaces_g0/","surfaces_g5/","surfaces_rounded/", "surfaces_inclined/"]
            # margins = [[0.01,0.],[0.02,0.05],[0.03,0.],[0.03,0.]]
            # Whole scene rendering
            folder_names = ["surfaces_g0/","surfaces_g1/","surfaces_g2/","surfaces_g3/","surfaces_g4/","surfaces_g5/","surfaces_rounded/", "surfaces_inclined/"]
            margins = [[0.01,0.],[0.02,0.05],[0.01,0.03],[0.00,0.06],[0.04,0.07],[0.02,0.05],[0.03,0.],[0.03,0.]] 
            for i,fname in enumerate(folder_names):
                names = [f for f in listdir(folderpath+fname) if isfile(join(folderpath+fname, f))]
                
                hmatrix = np.zeros((4,4))
                hmatrix[:3,:3] = orientation_matrix[:,:]
                hmatrix[:3,-1] = translation[:]

                all_surfaces_tmp = dict()
                for id,file in enumerate(names):
                    obj = trimesh.load_mesh(folderpath + fname +  file)
                    obj.apply_transform(hmatrix)
                    vert = order(np.array(obj.vertices))
                    filename = prefix + "_gr" + str(i) + "_" + str(id)
                    all_surfaces_tmp[filename] = vert
                    self.all_surfaces[filename] = vert
                    # self.all_surfaces_reduced[filename] = apply_margin(np.array(vert),0.)

                # Apply process to filter and decompose the surfaces to avoid overlap and apply a security margin.
                surfaces = [np.array(sf) for sf in all_surfaces_tmp.values()]
            
                surfaces_processed_tmp = process_surfaces(surfaces,
                                                    polySize=self._poly_size,
                                                    method=self._method_id,
                                                    min_area=self._min_area,
                                                    margin_inner=margins[i][0],
                                                    margin_outer=margins[i][1])
                    
                for sf in surfaces_processed_tmp:
                    if fname == "surfaces_g3/" :
                        # Quick fix
                        S = SurfaceData(sf,0.,0.)
                        if S.h_mean > 0.10:
                            continue
                    self.surfaces_processed.append(sf)            


    def extract_surfaces(self):
        """ Extract surfaces from the URDF file.

        Retruns:
            - param1 (dict): Dictionnary type containing all the surfaces ("unique id" : [vertices]).
        """
        return dict(zip([str(k) for k in range(len(self.surfaces_processed))], [sf.tolist() for sf in self.surfaces_processed]))
