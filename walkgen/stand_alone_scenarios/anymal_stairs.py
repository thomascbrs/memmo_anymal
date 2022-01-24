#!/usr/bin/env python3
#
# Copyright 2021 University of Edinburgh
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
import pinocchio as pin
import pickle
import os
import walkgen.SurfacePlanner as SurfacePlanner
import sl1m.tools.plot_tools as plot
from time import perf_counter as clock
import matplotlib.pyplot as plt
from walkgen.tools.plot_tools import plot_marker_surface

# Load Marker array class example from ROS simulation
fileObject = os.getcwd() + "/data/example_marker_array.pickle"

with open(fileObject, 'rb') as file2:
    array_markers = pickle.load(file2)

# PARAMETERS FOR SURFACES PROCESSING
margin = 0.
n_points = 6
method_id = 3
polySize = 10
min_area = 0.03

# SL1M PARAMETERS
N_phase = 15

# WALKING PARAMETERS
T_gait = 2.
n_gait = 4  # Number of different phases in the gait

gait = np.array([[0.,1.,1.,1.],
                 [1.,0.,1.,1.],
                 [1.,1.,0.,1.],
                 [1.,1.,1.,0.]])
# gait = np.array([[0.,1.,0.,1.],
#                  [1.,0.,1.,0.]])
bvref = np.array([0.15,0.,0.]) # Reference velocity
initial_config = np.array([0.,0.3,0.,0.,0.,0.,1.]) # Initial config
h_init = -0.06
init_vertices = np.array([[0.65, -1, h_init], [0.65, 1., h_init], [-0.65, 1., h_init], [-0.65, -1, h_init]])

surface_planner = SurfacePlanner(N_phase,
                                 n_gait,
                                 T_gait,
                                 margin=margin,
                                 n_points=n_points,
                                 method_id=method_id,
                                 polySize=polySize,
                                 min_area=min_area,
                                 init_vertices=init_vertices )

# Initialisation of model quantities, get feet current position
anymal = surface_planner.anymal
q = anymal.q0.copy()
q[:3] = initial_config[:3]
pin.centerOfMass(anymal.model, anymal.data, q)
pin.updateFramePlacements(anymal.model, anymal.data)
pin.crba(anymal.model, anymal.data, q)

indexes = ['LF_FOOT', 'RF_FOOT', 'LH_FOOT', 'RH_FOOT']
current_contacts = np.zeros((3,4))
for i,idx in enumerate(indexes) :
    Id = anymal.model.getFrameId(idx)
    current_contacts[:,i] = anymal.data.oMf[Id].translation
current_contacts[2,:] = h_init

# List of configurations in planned horizon, using the reference velocity.
configs = []
configs.append(initial_config.tolist())
for i in range(1,N_phase):
    config = np.zeros(7)
    config[:3] = bvref*(T_gait/n_gait)*i + initial_config[:3]
    rpy = np.array([0.,0.,0.])
    config[3:] = pin.Quaternion(pin.rpy.rpyToMatrix(rpy)).coeffs()
    configs.append(config.tolist())

t0 = clock()
vertices, inequalities, indices = surface_planner.run(array_markers, configs, gait, current_contacts, bvref)
t1 = clock()

print("Run MIP [ms]", 1000. * (t1 - t0))

# Plot initial surfaces from markerArray
fig = plt.figure(figsize=(10, 6))
ax = plt.axes(projection='3d')
plt.title("Initial surfaces")
plot_marker_surface(array_markers,ax)

# Plot SL1M results
fig = plt.figure(figsize=(10, 6))
ax = plt.axes(projection='3d')
plt.title("SL1M result")
for sf in surface_planner.surfaces_processed :
    plot.plot_surface(sf,ax=ax)
plot.plot_planner_result(surface_planner.pb_data.all_feet_pos, coms=surface_planner.pb_data.coms, ax=ax, show=True)