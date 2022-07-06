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
from walkgen_surface_processing.tools.transforms import apply_margin, cross

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

    def __init__(self, vertices, margin_inner, margin_outer):
        """Initialize the surface data with the vertices positions in 3D.

        Args:
            - vertices (list): List of 3D vertices array x3.

        """
        # Surface main informations
        self.vertices = vertices  # Initial 3D vertices
        self.h_mean = self._compute_height(vertices)  # mean height of surface
        self.normal = self._compute_normal(vertices)  # Nomral of the surface
        self.equation = self._compute_equation(vertices, self.normal)  # ax +by + cz + d = 0

        # MArgin information
        self._margin_inner = margin_inner
        self._margin_outer = margin_outer

        # Inner contour with margin
        self.vertices_inner = None
        self.contour_inner = None
        self.Polygon_inner = None

        # Outer contour with margin
        self.vertices_outer = None
        self.contour_outer = None
        self.Polygon_outer = None

        self._initialize_inner()
        self._initialize_outer()

        # Results of decomposition algorithm if necessary
        self.vertices_reshaped2D = None

    def _initialize_inner(self):
        self.vertices_inner = np.array(apply_margin(self.vertices, self._margin_inner))
        self.contour_inner = self._get_contour(self.vertices_inner)  # [x0,y0,x1,y1, ... , xn,zn]
        self.Polygon_inner = self._get_Polygon(self.contour_inner)

    def _initialize_outer(self):
        self.vertices_outer = np.array(apply_margin(self.vertices, -self._margin_outer))
        self.contour_outer = self._get_contour(self.vertices_outer)  # [x0,y0,x1,y1, ... , xn,zn]
        self.Polygon_outer = self._get_Polygon(self.contour_outer)

    def _compute_equation(self, vertices, normal):
        """ Get surface equation such as ax + by + cz + d = 0.

        Returns:
            - array 4x: [a,b,c,d] parameters.
        """
        d = -np.dot(vertices[0], normal)
        return np.concatenate((normal, d), axis=None)

    def _compute_height(self, vertices):
        """ Compute mean the mean height of all vertices.
        Returns:
            - float: Mean height.
        """
        return np.mean([vt[2] for vt in vertices])

    def _compute_normal(self, vertices):
        """ Compute normal of a surface.

        Returns:
            - array x3: The normal of the surface.
        """
        # Computes normal surface
        S_normal = cross(vertices[0] - vertices[1], vertices[0] - vertices[2])
        # Check orientation of the normal
        if np.dot(S_normal, np.array([0., 0., 1.])) < 0.:
            S_normal = -S_normal

        norm = np.linalg.norm(S_normal)
        if norm > 10e-5:
            return S_normal / np.linalg.norm(S_normal)
        else:
            return np.array([0., 0., 1.])

    def _get_contour(self, vertices):
        """ Compute the contour of a given surface, projected in X,Y plan.

        Returns :
            - list: The list containing the vertices of the contour line such as
                            [x0,y0,x1,y1, ... , xn,yn].
        """
        contour = []  # Contour representation [x0,y0,x1,y1, ... , xn,yn]
        for k in range(len(vertices)):
            contour.append(vertices[k][0])
            contour.append(vertices[k][1])

        return contour

    def _get_Polygon(self, contour):
        ''' Get Polygon object from a contour.

        Returns:
            - Polygon : Polygon class from shapely.geometry.
        '''
        poly = []

        for k in range(0, len(contour), 2):
            poly.append((contour[k], contour[k + 1]))

        return Polygon(poly)

    def get_contour_inner(self):
        if self.contour_inner is None:
            self._initialize_inner()
        return self.contour_inner

    def get_contour_outer(self):
        if self.contour_outer is None:
            self._initialize_outer()
        return self.contour_outer

    def get_vertices_inner(self):
        if self.vertices_inner is None:
            self._initialize_inner()
        return self.vertices_inner

    def get_vertices_outer(self):
        if self.vertices_outer is None:
            self._initialize_outer()
        return self.vertices_outer
