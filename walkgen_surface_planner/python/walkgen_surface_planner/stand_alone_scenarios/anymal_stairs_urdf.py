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
from time import perf_counter as clock

import sl1m.tools.plot_tools as plot
import matplotlib.pyplot as plt

from walkgen_surface_planner.SurfacePlanner import SurfacePlanner
from walkgen_surface_planner.params import SurfacePlannerParams
from walkgen_surface_processing.SurfaceDetector import SurfaceDetector


# Walkgen parameters.
params = SurfacePlannerParams()

params.planeseg = True
params.N_phase = 10
params.typeGait = "trot"
params.com = True
params.margin = 0.01

# Extract surfaces from URDF file.
surface_detector = SurfaceDetector(
    params.path + params.urdf, params.margin, q0=None, initial_height=0.)
all_surfaces = surface_detector.extract_surfaces()

# Surface Planer initialization with params.
surface_planner = SurfacePlanner(params=params)
surface_planner.set_surfaces(all_surfaces)

# Initial config
initial_config = np.array([0.25, 0., 0., 0., 0., 0., 1.])
q = np.array([0., 0., 0., 0., 0., 0., 1., -0.1, 0.7, -1., -
             0.1, -0.7, 1., 0.1, 0.7, -1., 0.1, -.7, 1.])
q[:3] = initial_config[:3]

# Reference velocity
bvref = np.zeros(6)
bvref[0] = 0.05
bvref[5] = 0.

# Order : [LF, RF, LH, RH]
GAITS = {}
GAITS["walk"] = np.array([[0., 1., 1., 1.], [1., 0., 1., 1.], [
                         1., 1., 0., 1.], [1., 1., 1., 0.]])
GAITS["trot"] = np.array([[1., 0., 1., 0.], [0., 1., 0., 1.]])
gait_pattern = GAITS[params.typeGait]


# order ['LF_FOOT', 'RF_FOOT', 'LH_FOOT', 'RH_FOOT']
current_contacts = np.array(
    [[0.37, 0.37, -0.37, -0.37], [0.2, -0.2, 0.2, -0.2], [0., 0., 0., 0.]])
for k in range(4):
    current_contacts[:, k] += q[:3]

print(current_contacts)

# Run MIP problem.
t0 = clock()
selected_surfaces = surface_planner.run(
    q, gait_pattern, bvref, current_contacts)
t1 = clock()
print("Run MIP [ms]", 1000. * (t1 - t0))

# Plot SL1M results
fig = plt.figure(figsize=(10, 6))
ax = plt.axes(projection='3d')
for value in surface_planner.all_surfaces.values():
    plot.plot_surface(np.array(value).T, ax)
plot.plot_planner_result(surface_planner.pb_data.all_feet_pos,
                         coms=surface_planner.pb_data.coms, ax=ax, show=True)
