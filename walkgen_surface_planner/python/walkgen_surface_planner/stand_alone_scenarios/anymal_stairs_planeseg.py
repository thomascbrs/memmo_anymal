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
import matplotlib.pyplot as plt
try:
    from time import perf_counter as clock
except ImportError:
    from time import time as clock 

import sl1m.tools.plot_tools as plot

from walkgen_surface_planner.SurfacePlanner import SurfacePlanner
from walkgen_surface_planner.params import SurfacePlannerParams

# Initial config
initial_config = np.array([0.2, 0.5, 0., 0., 0., 0., 1.])
h_init = -0.06

# Load an exemple of surfaces processed using walkgen_surface_processing.
path = os.path.dirname(os.path.abspath(__file__)) + "/../data/"
try:
    filename = path + "example_marker_array_processed.pickle"
    with open(filename, 'rb') as file2:
        all_surfaces = pickle.load(file2)
except ValueError:
    # Python2.7
    filename = path + "example_marker_array_processed_prot2.pickle"
    with open(filename, 'rb') as file2:
        all_surfaces = pickle.load(file2)

# # Process the planeseg data using walkgen_surface_processing
# from walkgen_surface_processing.surface_processing import SurfaceProcessing
# from walkgen_surface_processing.params import SurfaceProcessingParams
# from walkgen_surface_processing.tools.plot_tools import plot_marker_surface

# # Load Planeseg data, MarkerArray class example extracted from rosbag.
# path = os.path.dirname(os.path.abspath(__file__)) + "/../data/"
# fileObject = path + "example_marker_array.pickle"
# with open(fileObject, 'rb') as file2:
#     array_markers = pickle.load(file2)
# # Plot initial surfaces from markerArray
# fig = plt.figure(figsize=(10, 6))
# ax = plt.axes(projection='3d')
# plt.title("Initial surfaces")
# plot_marker_surface(array_markers, ax)

# params_processing = SurfaceProcessingParams()
# params_processing.margin = 0.
# params_processing.n_points = 6
# params_processing.method_id = 3
# params_processing.poly_size = 10
# params_processing.min_area = 0.03
# surface_processing = SurfaceProcessing(initial_height=h_init, params=params_processing)
# all_surfaces = surface_processing.run(initial_config[:3], array_markers)

# Surface Planer initialization with params.
params = SurfacePlannerParams()
params.N_phase = 4
params.typeGait = "walk"
params.com = True
surface_planner = SurfacePlanner(params=params)

# Reference velocity
bvref = np.zeros(6)
bvref[0] = 0.08
bvref[5] = 0.

q = np.array([0., 0., 0., 0., 0., 0., 1., -0.1, 0.7, -1., -0.1, -0.7, 1., 0.1, 0.7, -1., 0.1, -.7, 1.])
q[:3] = initial_config[:3]

# Order : [LF, RF, LH, RH]
GAITS = {}
GAITS["walk"] = np.array([[0., 1., 1., 1.], [1., 0., 1., 1.], [1., 1., 0., 1.], [1., 1., 1., 0.]])
GAITS["trot"] = np.array([[1., 0., 1., 0.], [0., 1., 0., 1.]])
gait_pattern = GAITS[params.typeGait]

surface_planner.set_surfaces(all_surfaces)

# order ['LF_FOOT', 'RF_FOOT', 'LH_FOOT', 'RH_FOOT']
current_contacts = np.array([[0.37, 0.37, -0.37, -0.37], [0.2, -0.2, 0.2, -0.2], [0., 0., 0., 0.]])
for k in range(4):
    current_contacts[:, k] += q[:3]
current_contacts[2, :] = h_init

# Run MIP problem.
t0 = clock()
selected_surfaces = surface_planner.run(q[:7], gait_pattern, bvref, current_contacts)
t1 = clock()
print("Run MIP [ms]", 1000. * (t1 - t0))



# Plot SL1M results
fig = plt.figure(figsize=(10, 6))
ax = plt.axes(projection='3d')
plt.title("SL1M result")
for value in surface_planner.all_surfaces.values():
    plot.plot_surface(np.array(value).T, ax)
plot.plot_planner_result(surface_planner.pb_data.all_feet_pos, coms=surface_planner.pb_data.coms, ax=ax, show=True)
