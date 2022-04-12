## Surface Processing

Tools to extract convex patch from .stl file or post-process a set of convex surfaces.

---
### Convex surfaces from .stl file :

Extraction of support surfaces based on hpp-affordance library. A margin inside the surfaces can be added. To run an example :
```
python3 -m walkgen_surface_processing.examples.process_stl
```


Mesh file | Surfaces extracted
--- | ---
![](./doc/mesh_stl.png) | ![](./doc/affordances_example.png)

---
### Post-process of planeseg data:

"Plane Seg is a library for fitting planes to LIDAR, depth camera data or elevation maps. It uses robust estimation to fit planes by clustering planar points with similar normals" : https://github.com/ori-drs/plane_seg.

Some tools to post-process the data given by this library can be found here :
- Add an inner margin to the convex surfaces.
- Add a surface of support under the robot.
- Reduce the number of points of the surfaces.
- Remove overlapping between surfaces by using a convex decomposition (using Tess algorithm, see https://github.com/azrafe7/hxGeomAlgo)
- Remove surfaces with a small area.

For more information, see the notebook post-processing-tools.ipynb. To run an example :

```
python3 -m walkgen_surface_processing.examples.process_planeseg
```

Planeseg surfaces | Post-Processing
--- | ---
![](./doc/planeseg_rosbag.png) | ![](./doc/planeseg_postprocess.png)

---
### Dependencies:

- numpy, pickle, matplotlib, scipy
- visvalingamwyatt algorithm : ```pip3 install visvalingamwyatt```
- rdp algorithm (only for comparison with visvalingamwyatt) : ```pip3 install rdp```
- shapely : ```pip3 install shapely```
- pinocchio (python): ```sudo apt install -qqy robotpkg-py38-pinocchio```
- eiquadprog (cpp): ```sudo apt install robotpkg-eiquadprogs```
- hpp-fcl (cpp): ```sudo apt install robotpkg-hpp-fcl```
- hpp-affordance (cpp): ```sudo apt install robotpkg-hpp-affordance```
