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
import quadprog
from scipy.spatial import HalfspaceIntersection
from pyhull import qconvex


def compute_projection2D(pos, oTl=np.zeros(3), oRl=np.zeros((3, 3))):
    return np.dot(oRl.T , (pos - oTl))[:2]


def compute_worldFrame(pos, oTl=np.zeros(3), oRl=np.zeros((3, 3))):
    return oTl + np.dot(oRl , pos)


def get_normal(vertices):
    """ Compute normal of a surface.

    Args:
        - vertices (array or list nx3):  Surface is defined using the vertices positions:
                                        array([[x0, y0, z0],
                                                  ...
                                                [xn, yn, zn]]).

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
        return np.array([1., 0., 0.])


def get_surface_frame(vertices):
    """ Projection of the vertices in local frame of the surface (z' = 0 for all vertices).
    Base of the surface : x', y', z', Origin : P0
    Origin : P0
    z' : nomal vector
    x' : P1-P0 / norm(P1-P0)
    y' : z' ^ x'

    Args:
        - vertices (list) : List of 3D vertices.

    Returns :
        - (list) : List of 2D vectors.
    """
    oRl = np.zeros((3, 3))
    oRl[:, 0] = (vertices[1] - vertices[0]) / np.linalg.norm(vertices[1] - vertices[0])
    oRl[:, 2] = get_normal(vertices)
    y = cross(oRl[:, 2], oRl[:, 0])
    oRl[:, 1] = y / np.linalg.norm(y)
    oTl = vertices[0]
    return oTl, oRl


def apply_margin(vertices, margin=0.):
    """ Apply a margin in [m] : - margin > 0 --> inside the convex surface
                                - margin < 0 --> outside the convex surface

    Args :
        - vertices (list) : List of 3D vertices.
        - margin (float) : margin in [m]

    Returns :
        - (list) : surface reshaped.
    """
    # Projection of the 3D vertices in the 2D surface.
    oTl, oRl = get_surface_frame(vertices)
    vertices_2D = np.array([compute_projection2D(vt, oTl, oRl).tolist() for vt in vertices])
    equations = get_equations(vertices_2D)
    inner_eq = []
    for a, b, c in equations:
        if b != 0.:
            inner_eq.append([
                np.sign(b) * a / b,
                np.sign(b) * 1.0,
                np.sign(b) * (c / b + np.sign(b) * margin * np.sqrt(1 + (a / b)**2))
            ])
        else:
            inner_eq.append([a, 0., c + np.sign(a) * a * margin])
    # halfspace equation for inner surface
    halfspaces = np.array(inner_eq)
    norm_vector = np.reshape(np.linalg.norm(halfspaces[:, :-1], axis=1), (halfspaces.shape[0], 1))
    A = np.hstack((halfspaces[:, :-1], norm_vector))
    b = -halfspaces[:, -1:].reshape(halfspaces.shape[0])
    q = np.zeros(3)
    q[2] = -1
    P = 10e-6 * np.identity(3)
    res = quadprog_solve_qp(P, q, G=A, h=b)

    # Halfspace intersection points 2D with margin in local surface frame
    feasible_point = res[:2]
    hs = HalfspaceIntersection(halfspaces, feasible_point)

    # Halfspace intersection points 3D with margin in world frame
    vert_inner_l = order(np.hstack([hs.intersections, np.zeros((hs.intersections.shape[0], 1))]))
    return [compute_worldFrame(pos, oTl, oRl).tolist() for pos in vert_inner_l]


def quadprog_solve_qp(P, q, G=None, h=None, C=None, d=None, verbose=False):
    """
    min (1/2)x' P x + q' x
    subject to  G x <= h
    subject to  C x  = d
    """
    # qp_G = .5 * (P + P.T)   # make sure P is symmetric
    qp_G = .5 * (P + P.T)  # make sure P is symmetric
    qp_a = -q
    qp_C = None
    qp_b = None
    meq = 0
    if C is not None:
        if G is not None:
            qp_C = -np.vstack([C, G]).T
            qp_b = -np.hstack([d, h])
        else:
            qp_C = -C.transpose()
            qp_b = -d
        meq = C.shape[0]
    elif G is not None:  # no equality constraint
        qp_C = -G.T
        qp_b = -h
    # t_init = clock()
    res = quadprog.solve_qp(qp_G, qp_a, qp_C, qp_b, meq)
    # t_end = clock()  - t_init
    if verbose:
        return res

    return res[0]


def order(points):
    """
    Order and remove repeated points in counterclockwise using pyhull library (only for 2D vectors)
    Assumptions : Project directly in X,Y plane and not into the surface local frame to avoid computational
    burden.

    Args:
        - points (list): List of 2D or 3D points.
    """
    if len(points) < 3:
        return 0
    output = qconvex("Fx", points[:, :2])
    output.pop(0)
    return [points[int(elt)].tolist() for elt in output]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def get_equations(points):
    """ Hyperplan normal with offsets : Ax <= -b
    """
    output = qconvex("n", points[:, :])
    return [[float(item) for item in elt.split()] for elt in output[2:]]
