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
import copy

from walkgen_surface_processing.params import SurfaceProcessingParams
from walkgen_surface_processing.tools.geometry_utils import reduce_surfaces, remove_overlap_surfaces, convert_from_marker_array


class SurfaceProcessing:
    """ Process a list of convex surfaces.
    #### Reduce the incoming surfaces.
    The following method is applied to process each surface:
    1. The vertices received are sorted counterclockwise, duplicates removed.
    2. The number of vertices is reduced using Visvalingam-Wyatt algorithm.
    3. An interior surface is calculated, with a margin parallel to each edge.

    #### Decompose the surfaces to avoid overlapping.
    Projection of the surfaces in X,Y plan and reshape them to avoid overlaying
    using Tesselation algorithm following the method:
    1. Run the list of surfaces starting with the lowest.
    2. Decomposes the lowest surface in set of convex ones by removing the upper surfaces that overlap it.
    (Tesselation algorithm).
    Use one of the 4 methods listed in DECOMPO_type to select/intersect the surfaces.
    3. Delate surface from the list and continue.

    --BASIC (0): Intersect the lowest polygon with all the upper polygon that overlay and keep all remaining
                surfaces.
    --AREA (1): After difference and decomposition with the upper polygons, select only the remining polygon
                whose area is superior to min_area arg.
    --CONVEX (2): Apply the difference with the convex hull of the upper surfaces that intersect, to avoid
                having small surfaces between the upper surfaces.
    --AREA_CONVEX (3): Both 1 and 2 methods.
    """

    def __init__(self, initial_height=0., params=None):
        """ Initialize intern parameters for surface processing.

        Args:
            - initial_height (float)
            - parasm (SurfacePlannerParams)
        """
        if params is not None:
            self._params = copy.deepcopy(params)
        else:
            self._params = SurfaceProcessingParams()

        self._initial_height = initial_height
        self._dx = 1.5  # Distance on y-axis around the current position.
        self._dy = 1.5  # Distance on y-axis around the current position.

        # Parameters for postprocessing.
        self._n_points = self._params.n_points
        self._method_id = self._params.method_id
        self._poly_size = self._params.poly_size
        self._min_area = self._params.min_area
        self._margin = self._params.margin

    def run(self, position, markerArray):
        """ Process the surfaces list from markerArray data type.
        Use a moving surface below the robot to ensure a surface is always under the robot. If unnecessary, it will be removed
        with the processing.

        Args:
            - position (list/array x3): Current position of the robot in world frame.
            - markerArray (list): The initial input list, containing marker objects.

        Returns:
            - param1 (Dictionnary): Dictionnary type containing the new surfaces with an unique id.
        """
        if len(markerArray.markers) == 0:
            print("Warning no polygon to process.")
            return dict()

        vertices = [[position[0] - self._dx, position[1] + self._dy, self._initial_height],
                    [position[0] - self._dx, position[1] -
                        self._dy, self._initial_height],
                    [position[0] + self._dx, position[1] -
                        self._dy, self._initial_height],
                    [position[0] + self._dx, position[1] + self._dy, self._initial_height]]

        surface_list = convert_from_marker_array(markerArray)

        # Apply process to filter and decompose the surfaces to avoid overlap
        np_surface_list = [np.array(s).T for s in surface_list]
        surfaces_without_overlap = remove_overlap_surfaces(
            np_surface_list,
            polySize=self._poly_size,
            method=self._method_id,
            min_area=self._min_area,
            initial_floor=np.array(vertices).T)

        # Reduce and sort incoming data
        surfaces_processed = reduce_surfaces(
            surfaces_without_overlap, margin=self._margin, n_points=self._n_points)

        return dict(zip([str(k) for k in range(len(surfaces_processed))], [sf.T.tolist() for sf in surfaces_processed]))
