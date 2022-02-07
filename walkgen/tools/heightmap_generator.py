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
import os
import numpy as np
import pickle

from walkgen.tools.heightmap import Heightmap
from walkgen.tools.geometry_utils import getAllSurfacesDict_inner

from solo_rbprm.solo_abstract import Robot

from hpp.corbaserver.affordance.affordance import AffordanceTool
from hpp.corbaserver.rbprm.tools.surfaces_from_path import getAllSurfacesDict
from hpp.corbaserver.problem_solver import ProblemSolver
from hpp.gepetto import ViewerFactory

# --------------------------------- PROBLEM DEFINITION -----------------------------------------------------

environment_urdf = os.getcwd() + "/data/urdf/lab_scene.urdf"

N_X = 100
N_Y = 100
X_BOUNDS = [-1.5, 4.0]
Y_BOUNDS = [-1.5, 1.0]

# Initialize HPP with solo_robot, a robot is needed to initialize the collision tool.
rom_names = ['solo_LFleg_rom', 'solo_RFleg_rom', 'solo_LHleg_rom', 'solo_RHleg_rom']
others = ['FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT']
LIMBS = ['solo_RHleg_rom', 'solo_LHleg_rom', 'solo_LFleg_rom', 'solo_RFleg_rom']
paths = [
    os.environ["INSTALL_HPP_DIR"] + "/solo-rbprm/com_inequalities/feet_quasi_flat/",
    os.environ["INSTALL_HPP_DIR"] + "/solo-rbprm/relative_effector_positions/"
]

COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
# --------------------------------- METHODS ---------------------------------------------------------------


def init_afftool():
    """
    Initialize the affordance tool and return the solo abstract rbprm builder, the surface
    dictionary and all the affordance points
    """
    robot = Robot()
    robot.setJointBounds("root_joint", [-5., 5., -5., 5., 0.241, 1.5])
    robot.boundSO3([-3.14, 3.14, -0.01, 0.01, -0.01, 0.01])
    robot.setFilter(LIMBS)
    for limb in LIMBS:
        robot.setAffordanceFilter(limb, ['Support'])
    ps = ProblemSolver(robot)
    vf = ViewerFactory(ps)
    afftool = AffordanceTool()
    afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])
    afftool.loadObstacleModel(environment_urdf, "environment", vf)
    ps.selectPathValidation("RbprmPathValidation", 0.05)

    return afftool


def plot_surface(points, ax, color_id=0, alpha=1.):
    """
    Plot a surface
    """
    xs = np.append(points[0, :], points[0, 0]).tolist()
    ys = np.append(points[1, :], points[1, 0]).tolist()
    zs = np.append(points[2, :], points[2, 0]).tolist()
    if color_id == -1:
        ax.plot(xs, ys, zs)
    else:
        ax.plot(xs, ys, zs, color=COLORS[color_id % len(COLORS)], alpha=alpha)


def draw_whole_scene(surface_dict, ax=None, title=None):
    """
    Plot all the potential surfaces
    """
    if ax is None:
        fig = plt.figure()
        if title is not None:
            fig.suptitle(title, fontsize=16)
        ax = fig.add_subplot(111, projection="3d")
    for key in surface_dict.keys():
        plot_surface(np.array(surface_dict[key][0]).T, ax, 5)
    return ax


def plot_heightmap(heightmap, alpha=1., ax=None):
    """
    Plot the heightmap
    """
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
    i = 0
    if alpha != 1.:
        i = 1

    xv, yv = np.meshgrid(heightmap.x, heightmap.y, sparse=False, indexing='ij')
    ax.plot_surface(xv, yv, heightmap.z, color=COLORS[i], alpha=alpha)

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_zlim([np.min(heightmap.z), np.max(heightmap.z) + 1.])

    return ax

def save_pickle(obj, filename):
    with open(filename, 'wb') as pickle_out:
        pickle.dump(obj, pickle_out)


# --------------------------------- MAIN ---------------------------------------------------------------
if __name__ == "__main__":
    afftool = init_afftool()
    affordances = afftool.getAffordancePoints('Support')
    all_surfaces = getAllSurfacesDict(afftool)
    new_surfaces = getAllSurfacesDict_inner(all_surfaces, 0.1)

    heightmap = Heightmap(N_X, N_Y, X_BOUNDS, Y_BOUNDS)
    heightmap.build(affordances)
    # heightmap.save_binary(os.environ["SOLO3D_ENV_DIR"] + params.environment_heightmap)

    heightmap_path = os.getcwd() + "/data/lab_scene.dat"
    heightmap.save_pickle(heightmap_path)
    # save_pickle(heightmap,heightmap_path)

    ax_heightmap = plot_heightmap(heightmap)
    ax = draw_whole_scene(all_surfaces)
    draw_whole_scene(new_surfaces, ax)
    plt.show(block=True)
