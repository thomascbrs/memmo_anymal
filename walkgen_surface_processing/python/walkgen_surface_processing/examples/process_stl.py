import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt

from walkgen_surface_processing.surface_detector import SurfaceDetector
from walkgen_surface_processing.params import SurfaceProcessingParams
from walkgen_surface_processing.tools.plot_tools import plot_surface

# Parameters of the environment
params = SurfaceProcessingParams()
params.margin = 0.08  # Inner margins [m].

# Rotation and translation of the environment
translation = np.array([0., 0., 0.])  # translation
rpy = np.array([0., 0., 0.])  # Roll,pitch & yaw.
R = pin.rpy.rpyToMatrix(rpy)  # Rotation matrix

# Extract surfaces from .stl file.
surface_detector = SurfaceDetector(params.path + params.stl, R, translation, 0. , "environment_")
all_surfaces = surface_detector.extract_surfaces()

surface_detector = SurfaceDetector(params.path + params.stl, R, translation, params.margin , "environment_")
all_surfaces_reduced = surface_detector.extract_surfaces()

# Plot surfaces.
fig = plt.figure(figsize=(10, 6))
ax = plt.axes(projection='3d')
for sf in all_surfaces.values():
    plot_surface( np.array(sf).T, ax,color = "k")
for sf in all_surfaces_reduced.values():
    plot_surface( np.array(sf).T, ax,color = "r")
plt.title("Surfaces extracted from .stl file")
plt.show()
