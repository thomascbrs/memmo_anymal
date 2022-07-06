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

from walkgen_surface_processing.tools.transforms import apply_margin
from walkgen_surface_processing.tools.SurfaceData import SurfaceData
import walkgen_surface_processing.tools.Tess2 as Tess2
from pyhull import qconvex


class DECOMPO_type(Enum):
    BASIC = 0
    AREA = 1
    CONVEX = 2
    AREA_CONVEX = 3


tess = Tess2.hxGeomAlgo_Tess2()


def convert_from_marker_array(marker_array ,offset = 0.0):
    '''Convert to a list of list from ros msg.'''
    surface_list = []
    for marker in marker_array.markers:
        # Marker structure :
        # [Pt1,Pt2,Pt2,Pt3,Pt3,Pt4, ... , Ptn-1, Ptn, Pt1, Ptn] !Warning order at the end
        pts = [[pt.x, pt.y, pt.z + offset] for pt in marker.points]  # List not sorted, with duplicates
        surface_list.append(remove_duplicates(pts).tolist())
    return surface_list


def reduce_surfaces(surface_list, n_points=None):
    ''' Remove duplicates, sort in counter-clock wise and reduce the numbre of points.

    The following method is applied to process each surface:
    1. The vertices received are sorted counterclockwise, duplicates removed.
    2. The number of vertices is reduced using Visvalingam-Wyatt algorithm.

    Args:
        - surface_list (list): List of List containing 3D points.
        - n_points (int or None): The maximal number of vertices for each surface.
                                  None --> No reduction.

    Returns:
        - (list) : The list the surfaces reduced. List of list containing 3D points.
    '''
    if 'MarkerArray' in str(type(surface_list)):
        surface_list = convert_from_marker_array(surface_list)
    else:
        surface_list_tmp = []
        for vs in surface_list:
            ordered_s = order(remove_duplicates(vs))
            if ordered_s != 0:
                surface_list_tmp.append(ordered_s)

        surface_list = surface_list_tmp

    out_surface_list = []
    if n_points is None:
        out_surface_list = surface_list
    else:
        for vertices in surface_list:
            simplifier = vw.Simplifier(vertices)

            out_surface_list.append(simplifier.simplify(number=n_points))

    return out_surface_list


def process_surfaces(surfacesIn, polySize=10, method=0, min_area=0., margin_inner=0., margin_outer=0., clearmap=False,clearmap_limit = 0. ):
    """Filter the surfaces. Projection of the surfaces in X,Y plan and reshape them to avoid overlaying
    using Tesselation algorithm following the method:
    1. Run the list of surfaces starting with the lowest.
    2. Apply an inner security margin on the contour of the surface.
    2. Decomposes the lowest surface in set of convex ones by removing the upper surfaces that overlap it
    with an outer margin.
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

    surfaces = []
    new_surfaces = []

    for sf in surfacesIn:
        try:
            s = SurfaceData(sf, margin_inner=margin_inner, margin_outer=margin_outer)
            if clearmap and (s.h_mean < clearmap_limit):
                pass
            else:
                surfaces.append(s)
        except:
            print("Applying inner margin not feasible. Surface removed.")

    # Temporary list for decomposition.
    contours_intersect = []
    surfaces_intersect = []

    # Run the list of surfaces starting with the lowest.
    while len(surfaces) > 1:
        try :
            contours_intersect.clear()
            surfaces_intersect.clear()
        except AttributeError:
            del contours_intersect[:]
            del surfaces_intersect[:]


        h_mean = [sf.h_mean for sf in surfaces]
        id_ = np.argmin(h_mean)

        for i, s_ in enumerate(surfaces):
            if i != id_:
                if surfaces[id_].Polygon_inner.intersects(s_.Polygon_outer):
                    surfaces_intersect.append(s_)
                    contours_intersect.append(s_.contour_outer)

        if method_type == DECOMPO_type.CONVEX or method_type == DECOMPO_type.AREA_CONVEX:
            # If only one surface, no need for the convex union
            if len(surfaces_intersect) > 1:
                # Redefine the contour for the difference
                try :
                    contours_intersect.clear()
                except AttributeError:
                    del contours_intersect[:]
                vertices_union = np.zeros((1, 2))
                for sf in surfaces_intersect:
                    vertices_union = np.vstack([vertices_union, sf.vertices_outer[:, :2]])
                convexHUll = ConvexHull(vertices_union[1:, :])
                vert_2D = np.zeros((2, 1))
                for j in convexHUll.vertices:
                    vert_2D = np.hstack([vert_2D, convexHUll.points[j, :].reshape((2, 1), order="F")])

                contours_intersect = [get_contour(vert_2D[:, 1:])]

        # No surface overllaping, keep initial surface.
        if len(contours_intersect) == 0:
            new_surfaces.append(surfaces[id_].vertices_inner)

        # Surface overllaping, decompose the surface.
        if len(contours_intersect) != 0:

            res = tess.difference([surfaces[id_].contour_inner], contours_intersect, polySize=polySize)
            surface_processed = process_tess_results(res, polySize)

            if surface_processed is None:  # no surface left after intersection.
                surfaces[id_].vertices_reshaped = None
            else:
                for vt in surface_processed:

                    if method_type == DECOMPO_type.AREA or method_type == DECOMPO_type.AREA_CONVEX:
                        # Keep the surface if the area > min_area
                        # Or if the surface has not been decomposed
                        if get_Polygon(get_contour(vt)).area > min_area:
                            new_surfaces.append(projection_surface(vt, surfaces[id_].equation))
                    else:
                        new_surfaces.append(projection_surface(vt, surfaces[id_].equation))

        # Remove the surface processed from the lists.
        surfaces.pop(id_)

    # Add last surface remaining
    if len(new_surfaces) > 0:
        new_surfaces.append(surfaces[0].get_vertices_inner())
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
        - vertices (array or list nx3):  Surface is defined using the vertices positions:
                                        array([[x0, y0, z0],
                                                  ...
                                                [xn, yn, zn]]).

    Returns:
        - array x3: The normal of the surface.
    """
    # Computes normal surface
    S_normal = np.cross(vertices[0] - vertices[1], vertices[0] - vertices[2])
    # Check orientation of the normal
    if np.dot(S_normal, np.array([0., 0., 1.])) < 0.:
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
    return np.vstack([vertices2D[:2, :], z]).T


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
        normal = get_normal(np.array(vertices))
        vertices_inner = apply_margin(np.array(vertices), margin)

        # Save inner vertices
        all_names.append(name_surface)
        surfaces.append((vertices_inner, normal.tolist()))

    surfaces_dict = dict(zip(all_names, surfaces))
    return surfaces_dict


def align_points(vertices):
    """ Align points to ensure convexity of the surface. 3 first point kept and the others projected into the 2D surface.

    Args:
        - vertices (array or list n x 3): vertices in 3D.

    Returns:
        - param1 (array n x 3): vertices in 3D aliggned.
    """
    vertices_projected = []
    vertices_projected.append(vertices[0])
    vertices_projected.append(vertices[1])
    vertices_projected.append(vertices[2])
    if len(vertices) == 3:
        return np.array(vertices_projected)
    else:
        normal = get_normal(vertices)
        # Projection of the other vertices into the surface obtained by the 3 first vertices
        # to ensure convexity of the surfaces.
        for k in range(3, len(vertices)):
            vertices_projected.append(vertices[k] - np.dot(np.dot((vertices[k] - vertices[0]), normal), normal))
        return np.array(vertices_projected)


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


def remove_duplicates(points):
    return np.unique(points, axis=0)
