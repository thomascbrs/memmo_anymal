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
import hppfcl
import pickle
import ctypes

from walkgen_surface_planner.tools.optimisation import quadprog_solve_qp


class MapHeader(ctypes.Structure):
    _fields_ = [
        ("size_x", ctypes.c_int),
        ("size_y", ctypes.c_int),
        ("x_init", ctypes.c_double),
        ("x_end", ctypes.c_double),
        ("y_init", ctypes.c_double),
        ("y_end", ctypes.c_double),
    ]


class Heightmap:

    def __init__(self, n_x, n_y, x_lim, y_lim, offset=0.):
        """
        :param n_x number of samples in x
        :param n_y number of samples in y
        :param x_lim bounds in x
        :param y_lim bounds in y
        """
        self.n_x = n_x
        self.n_y = n_y

        self.x = np.linspace(x_lim[0], x_lim[1], n_x)
        self.y = np.linspace(y_lim[0], y_lim[1], n_y)

        self.z = np.zeros((n_x, n_y))

        # Usefull quantities
        self.x_init = x_lim[0]
        self.x_end = x_lim[1]
        self.y_init = y_lim[0]
        self.y_end = y_lim[1]

        self.dx = (self.x_end - self.x_init)/(self.n_x - 1)
        self.dy = (self.y_end - self.y_init)/(self.n_y - 1)

        self.heightmap_fit_length = 0.3  # Length of half the heightmap fit in a direction
        self.heightmap_fit_size = 10  # Number of points on each axis in the heightmap fit

        self.A = np.ones((self.heightmap_fit_size ** 2, 3))
        self.b = np.zeros((self.heightmap_fit_size ** 2, 1))

        self.offset = offset

    def xIndex(self, x):
        """
        Get the closest i indice of a given x-axis position in the heightmap

        Params:
            - x (float): x-axis position

        Return :
            - int: Index of the closest point in the heightmap, or -1 if outside the boundaries.
        """
        if x < self.x_init or x > self.x_end:
            return -1
        else:
            return int((x - self.x_init)/self.dx)

    def yIndex(self, y):
        """
        Get the closest i indice of a given y-axis position in the heightmap

        Params:
            - y (float): y-axis position

        Return :
            - int: Index of the closest point in the heightmap, or -1 if outside the boundaries.
        """
        if y < self.y_init or y > self.y_end:
            return -1
        else:
            return int((y - self.y_init)/self.dy)

    def get_height(self, x, y):
        """ Get height given a 2D position

        Parameters:
            x (float): x-axis position
            y (float): y-axis position
        """
        iX = self.xIndex(x)
        iY = self.yIndex(y)
        if iX == -1 or iY == -1:
            return self.offset
        else:
            return self.z[iX, iY] + self.offset

    def fit_surface(self, x, y):
        """
        Update the surface equation to fit the heightmap, [a,b,c] such as ax + by -z +c = 0 for a given 2D position

        Parameters:
            x (float) x-axis position
            y (float) y-axis position
        """
        xVector = np.linspace(x - self.heightmap_fit_length,
                              x + self.heightmap_fit_length, self.heightmap_fit_size)
        yVector = np.linspace(y - self.heightmap_fit_length,
                              y + self.heightmap_fit_length, self.heightmap_fit_size)
        index = 0
        for i in range(0, self.heightmap_fit_size):
            for j in range(0, self.heightmap_fit_size):
                self.A[index, 0] = xVector[i]
                self.A[index, 1] = yVector[j]
                self.b[index] = self.get_height(xVector[i], yVector[j])
                index += 1

        fit = quadprog_solve_qp(np.dot(self.A.T, self.A),
                                (-np.dot(self.A.T, self.b)).reshape((3)))

        return fit

    def save_pickle(self, filename):
        with open(filename, 'wb') as pickle_out:
            pickle.dump(self, pickle_out)

    def save_binary(self, filename):
        """
        Save heightmap matrix as binary file.
        Args :
        - filename (str) : name of the file saved.
        """

        arr_bytes = self.z.astype(ctypes.c_double).tobytes()
        h = MapHeader(self.n_x, self.n_y,
                      self.x[0], self.x[-1], self.y[0], self.y[-1])
        h_bytes = bytearray(h)

        with open(filename, "ab") as f:
            f.truncate(0)
            f.write(h_bytes)
            f.write(arr_bytes)

    def build(self, affordances):
        """
        Build the heightmap and return it
        For each slot in the grid create a vertical segment and check its collisions with the
        affordances until one is found
        :param affordances list of affordances
        """
        last_z = 0.
        for i in range(self.n_x):
            for j in range(self.n_y):
                p1 = np.array([self.x[i], self.y[j], -1.])
                p2 = np.array([self.x[i], self.y[j], 10.])
                segment = np.array([p1, p2])
                fcl_segment = convex(segment, [0, 1, 0])

                intersections = []
                for affordance in affordances:
                    fcl_affordance = affordance_to_convex(affordance)
                    if distance(fcl_affordance, fcl_segment) < 0:
                        for triangle_list in affordance:
                            triangle = [np.array(p) for p in triangle_list]
                            if intersect_line_triangle(segment, triangle):
                                intersections.append(
                                    get_point_intersect_line_triangle(segment, triangle)[2])

                if len(intersections) != 0:
                    self.z[i, j] = np.max(np.array(intersections))
                    last_z = self.z[i, j]
                else:
                    # self.z[i, j] = last_z # increase same height on y axis
                    self.z[i, j] = 0.


def affordance_to_convex(affordance):
    """
    Creates a hpp-FCL convex object with an affordance
    """
    vertices = hppfcl.StdVec_Vec3f()
    faces = hppfcl.StdVec_Triangle()
    for triangle_list in affordance:
        [vertices.append(np.array(p)) for p in triangle_list]
        faces.append(hppfcl.Triangle(0, 1, 2))
    return hppfcl.Convex(vertices, faces)


def convex(points, indices):
    """
    Creates a hpp-FCL convex object with a list of points and three indices of the vertices of the
    triangle (or segment)
    """
    vertices = hppfcl.StdVec_Vec3f()
    faces = hppfcl.StdVec_Triangle()
    vertices.extend(points)
    faces.append(hppfcl.Triangle(indices[0], indices[1], indices[2]))
    return hppfcl.Convex(vertices, faces)


def distance(object1, object2):
    """
    Returns the distance between object1 and object2
    """
    guess = np.array([1., 0., 0.])
    support_hint = np.array([0, 0], dtype=np.int32)

    shape = hppfcl.MinkowskiDiff()
    shape.set(object1, object2, hppfcl.Transform3f(), hppfcl.Transform3f())
    gjk = hppfcl.GJK(150, 1e-8)
    gjk.evaluate(shape, guess, support_hint)
    return gjk.distance


# Method to intersect triangle and segment
def signed_tetra_volume(a, b, c, d):
    return np.sign(np.dot(np.cross(b - a, c - a), d - a) / 6.0)


def intersect_line_triangle(segment, triangle):
    s1 = signed_tetra_volume(segment[0], triangle[0], triangle[1], triangle[2])
    s2 = signed_tetra_volume(segment[1], triangle[0], triangle[1], triangle[2])

    if s1 != s2:
        s3 = signed_tetra_volume(
            segment[0], segment[1], triangle[0], triangle[1])
        s4 = signed_tetra_volume(
            segment[0], segment[1], triangle[1], triangle[2])
        s5 = signed_tetra_volume(
            segment[0], segment[1], triangle[2], triangle[0])

        if s3 == s4 and s4 == s5:
            return True
        else:
            return False
    else:
        return False


def get_point_intersect_line_triangle(segment, triangle):
    s1 = signed_tetra_volume(segment[0], triangle[0], triangle[1], triangle[2])
    s2 = signed_tetra_volume(segment[1], triangle[0], triangle[1], triangle[2])

    if s1 != s2:
        s3 = signed_tetra_volume(
            segment[0], segment[1], triangle[0], triangle[1])
        s4 = signed_tetra_volume(
            segment[0], segment[1], triangle[1], triangle[2])
        s5 = signed_tetra_volume(
            segment[0], segment[1], triangle[2], triangle[0])

        if s3 == s4 and s4 == s5:
            n = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
            t = np.dot(triangle[0] - segment[0], n) / \
                np.dot(segment[1] - segment[0], n)
            return segment[0] + t * (segment[1] - segment[0])
        else:
            return np.zeros(3)
    else:
        return np.zeros(3)


def load_heightmap(heightmap_path, offset=0., fit_length=0.3, fit_size=10):
    """ Returns Heightmap class from path.
    """
    with open(heightmap_path, 'rb') as fs:
        loaded_model = pickle.load(fs)
    loaded_model.offset = offset
    # Length of half the heightmap fit in a direction
    loaded_model.heightmap_fit_length = fit_length
    # Number of points on each axis in the heightmap fit
    loaded_model.heightmap_fit_size = fit_size
    return loaded_model
