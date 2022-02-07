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
from shapely.geometry import Polygon

"""
Usefull structure to work on surface decomposition.
"""
class SurfaceData():
    """Store the data related to a 3D surface.

    Attributes:
    - vertices (array 3xn): The surface is defined using the vertices positions:
                                    array([[x0, x1, ... , xn],
                                           [y0, y1, ... , yn],
                                           [z0, z1, ... , zn]]).
    - h_mean (float): Height mean of the surface.
    - normal (arrayx3): Normal vector.
    - equation (arrayx4): [a,b,c,d] parameters. Surface equation such as ax + by + cz + d = 0.
    - contour (list): The list containing the vertices of the contour line such as
                            [x0,y0,x1,y1, ... , xn,yn] in X,Y axis.
    - Polygon: Polygon on X,Y axis. Polygon class from shapely.geometry.
    - vertices_reshaped2D (list): List containing the new surfaces in 2D, after some process.
    """

    def __init__(self, vertices, isFloor=False):
        """Initialize the surface data with the vertices positions in 3D.

        Args:
            - vertices (array 3xn): The surface is defined using the vertices positions:
                                    array([[x0, x1, ... , xn],
                                           [y0, y1, ... , yn],
                                           [z0, z1, ... , zn]]).
            - isFloor (bool): Flag for surface that has been added manually added.
                              (usefull for filtering process)

        Raises:
            ValueError: If vertice.shape[0] != 3.

        """
        if vertices.shape[0] != 3:
            raise ValueError('Array should be of size 3xn')

        self.vertices = vertices  # Initial 3D surface
        self.h_mean = self._compute_height(vertices)
        self.normal = self._compute_normal(vertices)
        self.equation = self._compute_equation(vertices, self.normal)
        self.contour = self._get_contour(vertices[:2, :])
        self.Polygon = self._get_Polygon(self.contour)
        self.vertices_reshaped2D = None
        self.isFloor = isFloor

    def _compute_equation(self, vertices, normal):
        """ Get surface equation such as ax + by + cz + d = 0.

        Returns:
            - array 4x: [a,b,c,d] parameters.
        """
        d = -vertices[:, 0] @ normal
        return np.concatenate((normal, d), axis=None)

    def _compute_height(self, vertices):
        """ Compute mean the mean height of all vertices.
        Returns:
            - float: Mean height.
        """
        return np.mean(vertices[2, :])

    def _compute_normal(self, vertices):
        """ Compute normal of a surface.

        Returns:
            - array x3: The normal of the surface.
        """
        # Computes normal surface
        S_normal = np.cross(vertices[:, 0] - vertices[:, 1], vertices[:, 0] - vertices[:, 2])
        if S_normal @ np.array([0., 0., 1.]) < 0.:  # Check orientation of the normal
            S_normal = -S_normal

        norm = np.linalg.norm(S_normal)
        if norm > 10e-5:
            return S_normal / np.linalg.norm(S_normal)
        else:
            return np.array([1., 0., 0.])

    def _get_contour(self, vertices):
        """ Compute the contour of a given surface, projected in X,Y plan.

        Returns :
            - list: The list containing the vertices of the contour line such as
                            [x0,y0,x1,y1, ... , xn,yn].
        """
        contour = [] # Contour representation [x0,y0,x1,y1, ... , xn,yn]
        for k in range(vertices.shape[1]):
            contour.append(vertices[0,k])
            contour.append(vertices[1,k])

        return contour

    def _get_Polygon(self,contour):
        ''' Get Polygon object from a contour.

        Returns:
            - Polygon : Polygon class from shapely.geometry.
        '''
        poly = []

        for k in range(0,len(contour),2):
            poly.append((contour[k],contour[k+1]))

        return Polygon(poly)
