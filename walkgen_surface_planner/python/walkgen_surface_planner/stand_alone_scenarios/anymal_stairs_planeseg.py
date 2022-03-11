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

import os
import pickle
import numpy as np
from time import perf_counter as clock
import matplotlib.pyplot as plt

import sl1m.tools.plot_tools as plot

from walkgen_surface_planner.SurfacePlanner import SurfacePlanner
from walkgen_surface_planner.params import SurfacePlannerParams
from walkgen_surface_planner.tools.geometry_utils import reduce_surfaces, remove_overlap_surfaces
from walkgen_surface_planner.tools.plot_tools import plot_marker_surface
from walkgen_surface_planner.tools.geometry_utils import reduce_surfaces, remove_overlap_surfaces

# Load Planeseg data, MarkerArray class example extracted from rosbag.
path = os.path.dirname(os.path.abspath(__file__)) + "/../data/"
fileObject = path + "example_marker_array.pickle"
with open(fileObject, 'rb') as file2:
    array_markers = pickle.load(file2)

# Walkgen parameters.
params = SurfacePlannerParams()

params.planeseg = True
params.N_phase = 4
params.typeGait = "walk"
params.com = False
params.margin = 0.
params.n_points = 6
params.method_id = 3
params.poly_size = 10
params.min_area = 0.03

# Surface Planer initialization with params.
surface_planner = SurfacePlanner(params = params)

# Initial config
initial_config = np.array([0.2, 0.5, 0., 0., 0., 0., 1.])
q = np.array([0., 0., 0., 0., 0., 0., 1., -0.1, 0.7, -1., -0.1, -0.7, 1., 0.1, 0.7, -1., 0.1, -.7, 1.])
q[:3] = initial_config[:3]
h_init = -0.06
init_vertices = np.array([[0.65, -1, h_init], [0.65, 1., h_init], [-0.65, 1., h_init], [-0.65, -1, h_init]]).T
surface_planner._init_surface.vertices = init_vertices  # No need to update A, b, recomputed by SL1M

# Reference velocity
bvref = np.zeros(6)
bvref[0] = 0.08
bvref[5] = 0.

# Order : [LF, RF, LH, RH]
GAITS = {}
GAITS["walk"] = np.array([[0., 1., 1., 1.], [1., 0., 1., 1.], [1., 1., 0., 1.], [1., 1., 1., 0.]])
GAITS["trot"] = np.array([[1., 0., 1., 0.], [0., 1., 0., 1.]])
gait_pattern = GAITS[params.typeGait]

# Reduce and sort incoming data
surfaces_reduced = reduce_surfaces(array_markers, margin=params.margin, n_points=params.n_points)

# Apply proccess to filter and decompose the surfaces to avoid overlap
set_surfaces = remove_overlap_surfaces(surfaces_reduced,
                                       polySize=params.poly_size,
                                       method=params.method_id,
                                       min_area=params.min_area,
                                       initial_floor=surface_planner._init_surface.vertices)

# order ['LF_FOOT', 'RF_FOOT', 'LH_FOOT', 'RH_FOOT']
current_contacts = np.array([[0.37, 0.37, -0.37, -0.37], [0.2, -0.2, 0.2, -0.2], [0., 0., 0., 0.]])
for k in range(4):
    current_contacts[:, k] += q[:3]
current_contacts[2, :] = h_init

# Run MIP problem.
t0 = clock()
selected_surfaces = surface_planner.run(q, gait_pattern, bvref, current_contacts, set_surfaces)
t1 = clock()
print("Run MIP [ms]", 1000. * (t1 - t0))

# Plot initial surfaces from markerArray
fig = plt.figure(figsize=(10, 6))
ax = plt.axes(projection='3d')
plt.title("Initial surfaces")
plot_marker_surface(array_markers, ax)

# Plot SL1M results
fig = plt.figure(figsize=(10, 6))
ax = plt.axes(projection='3d')
plt.title("SL1M result")
for sf in surface_planner.surfaces_processed:
    plot.plot_surface(sf, ax=ax)
plot.plot_planner_result(surface_planner.pb_data.all_feet_pos, coms=surface_planner.pb_data.coms, ax=ax, show=True)
