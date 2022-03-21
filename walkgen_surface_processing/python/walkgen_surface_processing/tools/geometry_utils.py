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
from enum import Enum

import visvalingamwyatt as vw
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon

from walkgen_surface_processing.tools.SurfaceData import SurfaceData
import walkgen_surface_processing.tools.Tess2 as Tess2


class DECOMPO_type(Enum):
    BASIC = 0
    AREA = 1
    CONVEX = 2
    AREA_CONVEX = 3


tess = Tess2.hxGeomAlgo_Tess2()


def reduce_surfaces(markerArray, margin=0., n_points=None):
    ''' Process the surfaces list from markerArray data type.

    The following method is applied to process each surface:
    1. The vertices received are sorted counterclockwise, duplicates removed.
    2. The number of vertices is reduced using Visvalingam-Wyatt algorithm.
    3. An interior surface is calculated, with a margin parallel to each edge.

    Args:
        - markerArray (list): The initial input list, containing marker objects.
        - margin (float): The margin to apply for each edge of the surface.
        - n_points (int or None): The maximal number of vertices for each surface.
                                  None --> No reduction.

    Returns:
        - list : The list the surfaces processed. The surfaces are defined using the vertices positions:
                 array([[x0, x1, ... , xn],
                        [y0, y1, ... , yn],
                        [z0, z1, ... , zn]])
    '''
    surface_list = []
    # surface_list.append(np.array(init_vertices).T)

    for id, marker in enumerate(markerArray.markers):
        # Marker structure :
        # [Pt1,Pt2,Pt2,Pt3,Pt3,Pt4, ... , Ptn-1, Ptn, Pt1, Ptn] !Warning order at the end
        # if id != 6 :
        pts = [[pt.x, pt.y, pt.z] for pt in marker.points]  # List not sorted, with duplicates
        vertices = order(np.array(pts))  # Sorted, no duplicates

        if n_points is None:
            vertices_vw = vertices
        else:
            simplifier = vw.Simplifier(vertices)
            vertices_vw = simplifier.simplify(number=n_points)

        if margin == 0.:
            vertices_vw = order(vertices_vw)
            surface_list.append(vertices_vw.T)

        else:
            ineq_inner, ineq_inner_vect, normal = compute_inner_inequalities(vertices_vw, margin)
            vertices_inner = compute_inner_vertices(vertices_vw, ineq_inner, ineq_inner_vect)
            vertices_inner = order(vertices_inner)  # If margin create intersection, need to be sorted

            surface_list.append(vertices_inner.T)

    return surface_list


def remove_overlap_surfaces(surfacesIn, polySize=10, method=0, min_area=0., initial_floor=None):
    """Filter the surfaces. Projection of the surfaces in X,Y plan and reshape them to avoid overlaying
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

    Args:
        - surfaces (list): List of the surface, defined using the vertices positions:
                                array([[x0, x1, ... , xn],
                                       [y0, y1, ... , yn],
                                       [z0, z1, ... , zn]]).
        - polySize (int): Number of maximum point for convex decomposition.
        - method (int): Strategy to apply for the decomposition.
        - min_area (flat or None): Keep the area

    Returns:
        - (list): List of the surface reshaped, defined using the vertices positions:
                                array([[x0, x1, ... , xn],
                                       [y0, y1, ... , yn],
                                       [z0, z1, ... , zn]]).
    """
    method_type = DECOMPO_type(method)

    surfaces_init = []
    surfaces = []
    contours_intersect = []
    surfaces_intersect = []
    # Format the incoming data with SurfaceData structure.
    # Project on X,Y axis the surface.
    for sf in surfacesIn:
        s = SurfaceData(sf)
        surfaces_init.append(s)
        surfaces.append(s)

    initial_floor_data = None
    if initial_floor is not None:
        initial_floor_data = SurfaceData(initial_floor, isFloor=True)
        surfaces_init.append(initial_floor_data)
        surfaces.append(initial_floor_data)

    # Run the list of surfaces starting with the lowest.
    while len(surfaces) > 1:
        h_mean = [sf.h_mean for sf in surfaces]
        id_ = np.argmin(h_mean)  # Get id of the lowest surface remaining and process it.

        # Get the list of contour that intersect the surface.
        contours_intersect.clear()
        surfaces_intersect.clear()

        initial_floor_intersection = False  # ONly for convex process
        for i, s_ in enumerate(surfaces):
            if i != id_:
                if surfaces[id_].Polygon.intersects(s_.Polygon):
                    if method_type == DECOMPO_type.CONVEX or method_type == DECOMPO_type.AREA_CONVEX:
                        if s_.isFloor:
                            initial_floor_intersection = True
                        else:
                            surfaces_intersect.append(s_)

                    contours_intersect.append(s_.contour)

        if method_type == DECOMPO_type.CONVEX or method_type == DECOMPO_type.AREA_CONVEX:
            if len(surfaces_intersect) == 1:  # No union needed
                contours_intersect.append(surfaces_intersect[0].contour)
            if len(surfaces_intersect) > 1:
                contours_intersect.clear()  # Redefine the contour for the difference
                vertices_union = np.zeros((1, 2))
                for sf in surfaces_intersect:
                    vertices_union = np.vstack([vertices_union, sf.vertices[:2, :].T])
                convexHUll = ConvexHull(vertices_union[1:, :])
                vert_2D = np.zeros((2, 1))
                for j in convexHUll.vertices:
                    vert_2D = np.hstack([vert_2D, convexHUll.points[j, :].reshape((2, 1), order="F")])

                contours_intersect.clear()
                contours_intersect = [get_contour(vert_2D[:, 1:])]
                if initial_floor_intersection:
                    contours_intersect.append(initial_floor_data.contour)

        if len(contours_intersect) == 0:  # No surface overllaping, keep initial surface.
            surfaces[id_].vertices_reshaped2D = [surfaces[id_].vertices[:2, :]]

        if len(contours_intersect) != 0:  # Surface overllaping, decompose the surface.

            res = tess.difference([surfaces[id_].contour], contours_intersect, polySize=polySize)
            surface_processed = process_tess_results(res, polySize)

            if surface_processed is None:  # no surface left after intersection.
                surfaces[id_].vertices_reshaped2D = None
            else:
                surfaces[id_].vertices_reshaped2D = surface_processed.copy()

        # Remove the surface processed from the lists.
        surfaces.pop(id_)

    # Reshape remaining surface
    surfaces[0].vertices_reshaped2D = [surfaces[0].vertices[:2, :]]

    new_surfaces = []
    for sf in surfaces_init:
        if sf.vertices_reshaped2D is not None:
            for vert in sf.vertices_reshaped2D:
                if method_type == DECOMPO_type.AREA or method_type == DECOMPO_type.AREA_CONVEX:
                    # Keep the surface if the area > min_area
                    # Or if the surface has not been decomposed
                    if get_Polygon(get_contour(vert)).area > min_area or len(sf.vertices_reshaped2D) < 1:
                        vert_3D = projection_surface(vert, sf.equation)
                        new_surfaces.append(vert_3D)
                else:
                    vert_3D = projection_surface(vert, sf.equation)
                    new_surfaces.append(vert_3D)

    return new_surfaces


def process_tess_results(res, poly_numbers):
    """ Get the surfaces in 2D from tess polygon decomposition.

    Args:
        - res (Tess object): Result object from Tess library.
        - poly_numbers (int): Maximum number of vertices for the polygon decomposition.

    Returns:
        - List or None:  List of (array 2xn) surfaces defined using the vertices positions:
                      array([[x0, x1, ... , xn],
                             [y0, y1, ... , yn]]).
    """
    if res.elementCount == 0:
        return None

    vertices_Ids = []
    surfaces = []
    for i in range(res.elementCount):
        list_Id = [id for id in res.elements[poly_numbers * i:poly_numbers * (i + 1)] if id != -1]
        vertices_Ids.append(list_Id)

    # Get 2D surfaces
    for vertices_Id in vertices_Ids:
        surface = np.zeros((2, len(vertices_Id)))
        for k, Id in enumerate(vertices_Id):
            surface[0, k] = res.vertices[2 * Id]
            surface[1, k] = res.vertices[2 * Id + 1]
        surfaces.append(surface)

    return surfaces


def get_contour(surface):
    """ Compute the contour of a given surface, projected in X,Y plan.

    Args :
        - surface (array 3xn):  Surface is defined using the vertices positions:
                                array([[x0, x1, ... , xn],
                                       [y0, y1, ... , yn],
                                       [z0, z1, ... , zn]]).

    Returns :
        - list: The list containing the vertices of the contour line such as
                          [x0,y0,x1,y1, ... , xn,yn].
    """
    contour = []  # Contour representation [x0,y0,x1,y1, ... , xn,yn]
    for k in range(surface.shape[1]):
        contour.append(surface[0, k])
        contour.append(surface[1, k])

    return contour


def get_Polygon(contour):
    ''' Get Polygon object from a contour.

    Args:
        - contour (list): The list containing the vertices of the contour line such as
                          [x0,y0,x1,y1, ... , xn,yn].

    Returns:
        - Polygon : Polygon class from shapely.geometry.
    '''
    poly = []

    for k in range(0, len(contour), 2):
        poly.append((contour[k], contour[k + 1]))

    return Polygon(poly)


def get_normal(vertices):
    """ Compute normal of a surface.

    Args:
        - vertices (array 3xn):  Surface is defined using the vertices positions:
                                array([[x0, x1, ... , xn],
                                       [y0, y1, ... , yn],
                                       [z0, z1, ... , zn]]).

    Returns:
        - array x3: The normal of the surface.
    """
    # Computes normal surface
    S_normal = np.cross(vertices[:, 0] - vertices[:, 1], vertices[:, 0] - vertices[:, 2])
    if np.dot(S_normal, np.array([0., 0., 1.])) < 0.:  # Check orientation of the normal
        S_normal = -S_normal

    norm = np.linalg.norm(S_normal)
    if norm > 10e-5:
        return S_normal / np.linalg.norm(S_normal)
    else:
        return np.array([1., 0., 0.])


def projection_surface(vertices2D, equation):
    """ Project an array of 2D vertices (z = 0) inside the surface.

    Returns:
        - array 3xn: The surface is defined using the vertices positions:
                        array([[x0, x1, ... , xn],
                               [y0, y1, ... , yn],
                               [z0, z1, ... , zn]]).
    """
    if equation[2] < 10e-5:
        raise ValueError('The surface is vertical, cannot project 2D vectors inside.')
    z = (1 / equation[2]) * (-equation[3] - np.dot(equation[:2], vertices2D[:2, :]))
    return np.vstack([vertices2D[:2, :], z])


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
    S_normal = np.cross(vertices[0, :] - vertices[1, :], vertices[0, :] - vertices[2, :])
    if np.dot(S_normal, np.array([0., 0., 1.])) < 0.:  # Check orientation of the normal
        S_normal = -S_normal

    normal = S_normal / np.linalg.norm(S_normal)

    ineq_inner = np.zeros((nb_vert + 1, 3))
    ineq_vect_inner = np.zeros((nb_vert + 1))

    ineq_inner[-1, :] = normal
    ineq_vect_inner[-1] = -(-normal[0] * vertices[0, 0] - normal[1] * vertices[0, 1] - normal[2] * vertices[0, 2])

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
        ineq_inner, ineq_inner_vect, normal = compute_inner_inequalities(vertices, margin)
        vertices_inner = compute_inner_vertices(vertices, ineq_inner, ineq_inner_vect)

        # Save inner vertices
        all_names.append(name_surface)
        surfaces.append((vertices_inner.tolist(), normal.tolist()))

    surfaces_dict = dict(zip(all_names, surfaces))
    return surfaces_dict

## TODO, from stackoverflow, find reference
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


## TODO, from stackoverflow, find reference
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


## TODO, from stackoverflow, find reference
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
