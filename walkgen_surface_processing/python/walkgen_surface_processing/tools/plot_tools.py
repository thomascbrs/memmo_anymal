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

import matplotlib.pyplot as plt
import numpy as np


def plot_marker_surface(array_markers, ax=None):
    """ PLot the edges and vertices from arrayMarker data type.

    Args:
        - markerArray (list): The initial input list, containing marker objects.
    """
    if ax is None:
        plt.figure()
        ax = plt.axes(projection="3d")

    for Id, marker in enumerate(array_markers.markers):
        points_ = marker.points
        X = [pt.x for pt in points_]
        Y = [pt.y for pt in points_]
        Z = [pt.z for pt in points_]
        ax.scatter(X, Y, Z, label="Id : " + str(Id))  # Plot 3D points

        X, Y, Z = [], [], []
        X = [pt.x for pt in points_]
        Y = [pt.y for pt in points_]
        Z = [pt.z for pt in points_]
        ax.plot3D(X, Y, Z)  # Plot edges

    ax.legend()
    return ax


def plot_contour(contour, ax=None, color="b"):
    """ Plot a contour surface in 2D.

    Args :
        - contour (list): List containing of the vertices of the contour line such as
                          [x0,y0,x1,y1, ... , xn,yn]
        - ax (ax): Pyplot axe object
        - color (str): Color of the edges.

    Returns :
        - ax: Pyplot object
    """
    if ax is None:
        fig = plt.figure()
        ax = plt.axes()

    cont_arr = np.array(contour).reshape(2, int(len(contour) / 2), order="F")

    xs = np.append(cont_arr[0, :], cont_arr[0, 0]).tolist()
    ys = np.append(cont_arr[1, :], cont_arr[1, 0]).tolist()
    ax.plot(xs, ys, color=color)

    return ax


def plot_surface2D(surface, ax=None, color="b"):
    """ Plot 2D surface.

    Args:
        - surface (array 2xn): 2D surface defined using the vertices positions:
                               array([[x0, x1, ... , xn],
                                      [y0, y1, ... , yn]]).
        - ax (ax): Pyplot axe object
        - color (str): Color of the edges.

    Returns :
        - ax: Pyplot object
    """
    if ax is None:
        fig = plt.figure()
        ax = plt.axes()

    xs = np.append(surface[0, :], surface[0, 0]).tolist()
    ys = np.append(surface[1, :], surface[1, 0]).tolist()
    ax.plot(xs, ys, color=color)

    return ax

def plot_surface(points, ax, color="b"):
    """
    Plot a 3D surface.
    """
    xs = np.append(points[:, 0], points[0, 0]).tolist()
    ys = np.append(points[:, 1], points[0, 1]).tolist()
    zs = np.append(points[:, 2], points[0, 2]).tolist()
    ax.plot(xs, ys, zs, color=color)
