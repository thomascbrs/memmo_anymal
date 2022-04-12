import numpy as np
from time import perf_counter as clock
import os
import pickle
import matplotlib.pyplot as plt

from walkgen_surface_processing.surface_processing import SurfaceProcessing
from walkgen_surface_processing.params import SurfaceProcessingParams
from walkgen_surface_processing.tools.plot_tools import plot_surface, plot_marker_surface

# Load Planeseg data, MarkerArray class example extracted from rosbag.
path = os.path.dirname(os.path.abspath(__file__)) + "/../data/"
fileObject = path + "example_marker_array.pickle"
with open(fileObject, 'rb') as file2:
    array_markers = pickle.load(file2)

# Parameters of the environment
params = SurfaceProcessingParams()
params.n_points = 10  # Maximum Number of points for for each convex surface.
params.margin = 0.01  # Margin in [m] inside the convex surfaces.
params.method_id = 3  # Method for convex decomposition.
params.poly_size = 10  # Maximum size of the polygon for the convex decomposition.
params.min_area = 0.003   # Area under which the remaining surfaces is delated.

# Initial height
initial_height = -0.06
position = np.array([0.2,0.5,0.])

# Extract surfaces from .stl file.
surface_processing = SurfaceProcessing(initial_height=initial_height, params=params)
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
for sf in all_surfaces.values():
    random_color=list(np.random.choice(range(255),size=3) / 255)
    plot_surface(np.array(sf).T, ax, color=random_color)
plt.title("Surfaces processed")
plt.show()
