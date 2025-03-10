import numpy as np
try:
    from time import perf_counter as clock
except ImportError:
    from time import time as clock
import os
import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from walkgen_surface_processing.surface_processing import SurfaceProcessing
from walkgen_surface_processing.params import SurfaceProcessingParams
from walkgen_surface_processing.tools.plot_tools import plot_surface, plot_marker_surface

# Load Planeseg data, MarkerArray class example extracted from rosbag.
path = os.path.dirname(os.path.abspath(__file__)) + "/../data/"
try:
    filename = path + "example_marker_array.pickle"
    with open(filename, 'rb') as file2:
        array_markers = pickle.load(file2)
except ValueError:
    # Python2.7
    filename = path + "example_marker_array_prot2.pickle"
    with open(filename, 'rb') as file2:
        array_markers = pickle.load(file2)

# Parameters of the environment
params = SurfaceProcessingParams()
params.n_points = 10  # Maximum Number of points for for each convex surface.
params.margin_inner = 0.02  # Margin in [m] inside the convex surfaces (safe footstep planning)
params.margin_outer = 0.04  # Margin in [m] outside the convex surfaces (collision avoidance)
params.method_id = 1  # Method for convex decomposition.
params.poly_size = 10  # Maximum size of the polygon for the convex decomposition.
params.min_area = 0.01  # Area under which the remaining surfaces is delated.

# Initial height
initial_height = -0.2
position = np.array([0.2, 0.5, 0.])

# Extract surfaces from .stl file.
surface_processing = SurfaceProcessing(initial_height=initial_height, params=params)
surface_processing.set_clearmap(False)
# surface_processing.set_offset_clearmap(0.02)
t0 = clock()
all_surfaces = surface_processing.run(position, array_markers)
t1 = clock()
print("Time took to process the surfaces [ms] : ", 1000 * (t1 - t0))

# Plot surfaces.
fig = plt.figure(figsize=(10, 6))
ax = plt.axes(projection='3d')
plot_marker_surface(array_markers, ax)
plt.title("Initial surfaces from planeseg rosbag.")

fig = plt.figure(figsize=(10, 6))
ax = plt.axes(projection='3d')
plot_marker_surface(array_markers, ax)
for sf in all_surfaces.values():
    random_color = list(np.random.choice(range(255), size=3) / 255)
    plot_surface(np.array(sf), ax, color=random_color)
plt.title("Surfaces processed")
plt.show()
