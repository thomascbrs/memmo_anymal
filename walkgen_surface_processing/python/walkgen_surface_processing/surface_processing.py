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
from walkgen_surface_processing.tools.geometry_utils import process_surfaces, convert_from_marker_array, order, reduce_surfaces


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
        self._dx = 2.8  # Distance on y-axis around the current position.
        self._dy = 2.8  # Distance on y-axis around the current position.

        # Parameters for postprocessing.
        self._n_points = self._params.n_points
        self._method_id = self._params.method_id
        self._poly_size = self._params.poly_size
        self._min_area = self._params.min_area
        self._margin_inner = self._params.margin_inner
        self._margin_outer = self._params.margin_outer
        self._offsets = self._params.offset_z

        self._offsets_clearmap = 0.02  # (initial_height + offsets_clearmap) height under which the surfaces are removed.
        self._clearmap = False  # Boolean to remove the some of the ground surfaces.

    def min_height(self, surfaces):
        try:
            return min(point[2] for surf in surfaces for point in surf)
        except Exception as e:
            print(e)
            print(surfaces)
        z_values = arr[..., 2]     
        return np.min(z_values)

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

        # Convert incoming data.
        surface_list = convert_from_marker_array(markerArray, self._offsets)

        # Sort, remove duplicates and reduce the number of points.
        surface_reduced = reduce_surfaces(surface_list, self._n_points)


        # floor height is reduced if other obstacles are lower
        # ~ min_h = self.min_height(surface_reduced)
        # ~ if (min_h < self._initial_height):
            # ~ print("WARNING: floor is too high, change _initial_height parameter (min height of obstacle, initial_height).", min_h, self._initial_height)
        # ~ min_h = min(min_h, self._initial_height)
        # Add floor around robot position.
        # ~ vertices = np.array([[position[0] - self._dx, position[1] + self._dy, min_h],
                             # ~ [position[0] - self._dx, position[1] - self._dy, min_h],
                             # ~ [position[0] + self._dx, position[1] - self._dy, min_h],
                             # ~ [position[0] + self._dx, position[1] + self._dy, min_h]])
        # ~ surface_reduced.append(vertices)

        # Apply process to filter and decompose the surfaces to avoid overlap and apply a security margin.
        surfaces_processed = process_surfaces(surface_reduced,
                                              polySize=self._poly_size,
                                              method=self._method_id,
                                              min_area=self._min_area,
                                              margin_inner=self._margin_inner,
                                              margin_outer=self._margin_outer,
                                              clearmap=self._clearmap,
                                              clearmap_limit=(self._initial_height + self._offsets_clearmap))

        return dict(zip([str(k) for k in range(len(surfaces_processed))], [sf.tolist() for sf in surfaces_processed]))

    def set_offset_clearmap(self, offset):
        """ Offset from which all surfaces under the initial height + offset will be removed if the boolean clearmap is active.
        """
        self._offsets_clearmap = offset

    def set_clearmap(self, clearmap):
        """ Set the clearmap boolean to remove the surfaces on the ground.
        """
        self._clearmap = clearmap
