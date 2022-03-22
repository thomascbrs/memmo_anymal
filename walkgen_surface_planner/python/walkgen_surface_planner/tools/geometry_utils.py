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
from scipy.spatial import ConvexHull


def norm(sq):
    """
    Computes b=norm
    """
    cr = np.cross(sq[2] - sq[0], sq[1] - sq[0])
    return np.abs(cr / np.linalg.norm(cr))


def compute_inner_inequalities(vertices, margin):
    """
    Compute surface inequalities from the vertices list with a margin, update self.ineq_inner,
    self.ineq_vect_inner
    ineq_iner X <= ineq_vect_inner
    the last row contains the equality vector
    Keyword arguments:
    Vertice of the surface  = [[x1 ,y1 ,z1 ]
                            [x2 ,y2 ,z2 ]
                                ...      ]]
    """
    nb_vert = vertices.shape[0]

    # Computes normal surface
    S_normal = np.cross(vertices[0, :] - vertices[1, :],
                        vertices[0, :] - vertices[2, :])
    # Check orientation of the normal
    if np.dot(S_normal, np.array([0., 0., 1.])) < 0.:
        S_normal = -S_normal

    normal = S_normal / np.linalg.norm(S_normal)

    ineq_inner = np.zeros((nb_vert + 1, 3))
    ineq_vect_inner = np.zeros((nb_vert + 1))

    ineq_inner[-1, :] = normal
    ineq_vect_inner[-1] = -(-normal[0] * vertices[0, 0] -
                            normal[1] * vertices[0, 1] - normal[2] * vertices[0, 2])

    for i in range(nb_vert):

        if i < nb_vert - 1:
            AB = vertices[i, :] - vertices[i + 1, :]
        else:
            # last point of the list with first
            AB = vertices[i, :] - vertices[0, :]

        n_plan = np.cross(AB, normal)
        n_plan = n_plan / np.linalg.norm(n_plan)

        # normal = [a,b,c].T
        # To keep the half space in the direction of the normal :
        # ax + by + cz + d >= 0
        # - [a,b,c] * X <= d

        # Take a point M along the normal of the plan, from a distance margin
        # OM = OA + AM = OA + margin*n_plan

        M = vertices[i, :] + margin * n_plan

        # Create the parallel plan that pass trhough M
        ineq_inner[i, :] = -np.array([n_plan[0], n_plan[1], n_plan[2]])
        ineq_vect_inner[i] = -n_plan[0] * M[0] - \
            n_plan[1] * M[1] - n_plan[2] * M[2]

    return ineq_inner, ineq_vect_inner, normal


def compute_inner_vertices(vertices, ineq_inner, ineq_vect_inner):
    """"
    Compute the list of vertice defining the inner surface :
    update self.vertices_inner = = [[x1 ,y1 ,z1 ]    shape((nb vertice , 3))
                                    [x2 ,y2 ,z2 ]
                                        ...      ]]
    """
    S_inner = []
    nb_vert = vertices.shape[0]

    # P = np.array([a,b,c,d]) , (Plan) ax + by + cz + d = 0
    P_normal = np.zeros(4)
    P_normal[:3] = ineq_inner[-1, :]
    P_normal[-1] = -ineq_vect_inner[-1]

    P1, P2 = np.zeros(4), np.zeros(4)

    for i in range(nb_vert):
        if i < nb_vert - 1:
            P1[:3], P2[:3] = ineq_inner[i, :], ineq_inner[i + 1, :]
            P1[-1], P2[-1] = -ineq_vect_inner[i], -ineq_vect_inner[i + 1]

            A, B = plane_intersect(P1, P2)
            S_inner.append(LinePlaneCollision(P_normal, A, B))
        else:
            P1[:3], P2[:3] = ineq_inner[i, :], ineq_inner[0, :]
            P1[-1], P2[-1] = -ineq_vect_inner[i], -ineq_vect_inner[0]

            A, B = plane_intersect(P1, P2)
            S_inner.append(LinePlaneCollision(P_normal, A, B))

    vertices_inner = np.array(S_inner)
    return vertices_inner


def getAllSurfacesDict_inner(all_surfaces, margin):
    '''
    Computes the inner vertices of the given convex surface, with a margin.
    Args :
    - all_surfaces : Dictionary containing the surface vertices, normal and name.
    - margin : (float) margin in m
    Returns :
    - New dictionnary with inner vertices
    '''

    all_names = []
    surfaces = []
    for name_surface in all_surfaces:
        vertices = order(np.array(all_surfaces.get(name_surface)[0]))
        ineq_inner, ineq_inner_vect, normal = compute_inner_inequalities(
            vertices, margin)
        vertices_inner = compute_inner_vertices(
            vertices, ineq_inner, ineq_inner_vect)

        # Save inner vertices
        all_names.append(name_surface)
        surfaces.append((vertices_inner.tolist(), normal.tolist()))

    surfaces_dict = dict(zip(all_names, surfaces))
    return surfaces_dict

# TODO, from stackoverflow, find reference


def order(vertices, method="convexHull"):
    """
    Order the array of vertice in counterclock wise using convex Hull method
    """
    if len(vertices) <= 3:
        return 0
    v = np.unique(vertices, axis=0)
    n = norm(v[:3])
    y = np.cross(n, v[1] - v[0])
    y = y / np.linalg.norm(y)
    c = np.dot(v, np.c_[v[1] - v[0], y])
    if method == "convexHull":
        h = ConvexHull(c)
        vert = v[h.vertices]
    else:
        mean = np.mean(c, axis=0)
        d = c - mean
        s = np.arctan2(d[:, 0], d[:, 1])
        vert = v[np.argsort(s)]

    return vert


# TODO, from stackoverflow, find reference
def plane_intersect(P1, P2):
    """
    Reference:
    Get the intersection between 2 plan, return Point and direction
    :param P1,P2: Plan equalities
              np.array([a,b,c,d])
              ax + by + cz + d = 0
    Returns : 1 point and 1 direction vect of the line of intersection, np.arrays, shape (3,)
    """

    P1_normal, P2_normal = P1[:3], P2[:3]

    aXb_vec = np.cross(P1_normal, P2_normal)

    A = np.array([P1_normal, P2_normal, aXb_vec])
    d = np.array([-P1[3], -P2[3], 0.]).reshape(3, 1)

    # could add np.linalg.det(A) == 0 test to prevent linalg.solve throwing error

    p_inter = np.linalg.solve(A, d).T

    return p_inter[0], (p_inter + aXb_vec)[0]


# TODO, from stackoverflow, find reference
def LinePlaneCollision(P, A, B, epsilon=1e-6):
    """
    Reference:
    Get the intersection point between 1 plane and 1 line
    :param P: Plane equality
                np.array([a,b,c,d])
                ax + by + cz + d = 0
    :param A,B : 2 points defining the line np.arrays, shape(3,)
    Returns : 1 point,  np.array, shape (3,)
    """
    plane_normal = P[:3]
    if P[0] == 0:
        if P[1] == 0:
            # a,b = 0 --> z = -d/c
            planePoint = np.array([0, 0, -P[-1] / P[2]])
        else:
            # a,c = 0 --> y = -d/b
            planePoint = np.array([0, -P[-1] / P[1], 0])
    else:
        planePoint = np.array([-P[-1] / P[0], 0., 0])  # b,c = 0 --> x = -d/a

    rayDirection = A - B
    ndotu = plane_normal.dot(rayDirection)
    if abs(ndotu) < epsilon:
        raise RuntimeError("no intersection or line is within plane")

    w = A - planePoint
    si = -plane_normal.dot(w) / ndotu
    Psi = w + si * rayDirection + planePoint
    return Psi
