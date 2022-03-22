# memmo_anymal
Workspace containing the works related to memmo project and the anymal demo.

--> PlaneSeg and SL1M integration.

--> Caracal and SL1M integration.

The workspace is composed of 3 python modules:
- walkgen_foostep_planner
- walkgen_surface_planner
- walkgen_ros
---
## Installation
Install cmake submodule:
```
git submodule init
git submodule update
```

Build the walkgen library:
```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=~/install -DFOOTSTEP_PLANNER=ON -DSURFACE_PLANNER=ON -DROS_INTERFACE=ON -DPYTHON_EXECUTABLE=$(which python3.8) -DPYTHON_STANDARD_LAYOUT=ON
```


The compilation option are the following:
- DCMAKE_BUILD_TYPE : build in release mode, usefull for cpp bindings (not used yet)
- DCMAKE_INSTALL_PREFIX : The path to install the modules.
- DFOOTSTEP_PLANNER : Install walkgen_foostep_planner.
- DSURFACE_PLANNER : Install walkgen_surface_planner.
- DROS_INTERFACE : Install walkgen_ros.
- DPYTHON_EXECUTABLE : Which python to use.
- DPYTHON_STANDARD_LAYOUT : Build the python modules inside python3.8 folder.
---

## Surface planner processing
Tools to post-process the data from planeseg or extract

---

## Footstep planner
This module is used to interface with the caracal library. It allows to update the trajectory of the feet at each time step by sending to the Caracal MPC the new polynomial coefficients of the trajectory. It contains :
- FootstepManager : Wrapper class to interface with Caracal.
- GaitManager : Allow creation and delation of Contact Schedule. Get the informations of the current gait to send to the surface planner.
- FootstepPlanner : Given a queue of contact, update the swing trajectory using Raibert's heuristic.
- params : A parameters class configurable with a yaml file.

#### Dependencies
It should works with python2. Check at the end of the README.me to install the dependencies.
- numpy
- quadprog
- caracal
- pinocchio
- yaml
---

## Surface planner
This module run SL1M algorithm to select the surfaces. There are 2 version of the code, by turning on/off the boolean planeseg in the config file :

- Planseg : Use perception to detect the surfaces. The convex surfaces are received from planseg as a ros markerArray message. There are post-processed (see notebook for explanation) and then used in sl1m algorithm.
- URDF : The environment comes from an URDF file. The surfaces are extracted using RBPRM. Prior to use this, a heightmap needs to be created externally (to rotate the inequalities, better SL1M behaviour).

#### Dependencies
Only tested with python3. Check at the end of the README.me to install the dependencies.
- sl1m
- visvalingamwyatt
- scipy
- shapely
- matplotlib
- pickle
- ctypes
- solo_rbprm (TODO, change with anymal)
- hpp.corbaserver
- hpp.gepetto
- anymal_rbprm
- quadprog
- hppfcl

#### Heightmap generation
TODO, clean the heightmap generation, make it easier to use.
Create an heightmap from the URDF environment. Usefull to use the SurfacePlanner with the guide path RBPRM, for the simulations. An example as been put inside the data folder, and will be exported to the install path seletec to build the modules (DCMAKE_INSTALL_PREFIX option). Setup the path inside the config file.

- Setup the ROS_PACKAGE_PATH variable environment in your /.bashrc file, for hpp. This allow to define in the urdf the paths as "package://meshes/lab_scene.stl". It can be defined in the install folder or the repository folder, since the data folder (containing urdf, meshes, heightmap) will be installed as well in the install folder (after make install is runned).
```export ROS_PACKAGE_PATH=your_path/memmo_anymal/walkgen_surface_planner/python/walkgen_surface_planner/data:$ROS_PACKAGE_PATH
```
An example of environment (URDF and STL files) has been put in the data folder.
To create a new heightmap, open the file directly and tune the path (heightmap and urdf environment) with your env or path. The binary file will be created in the data folder with the new name. Open a terminal and run :
```
$ hpp-rbprm-server
```

In another terminal, from the walkgen_surface_planner folder, run :
```
$ python3 tools/heightmap_generator.py
```

#### Example
Example of the SurfacePlanner with the convex patches returned by planeseg. Run :
```
python3 -m walkgen_surface_planner.stand_alone_scenarios.anymal_stairs_planeseg
```

Example of the SurfacePlanner with the URDF and heightmap of the environment. Run :
```
hpp-rbprm-server
```
In another terminal, run :
```
python3 -m walkgen_surface_planner.stand_alone_scenarios.anymal_stairs_urdf
```

#### Unitest
To test with planeseg data, run:
```
python3 -m walkgen_surface_planner.tests.surface_planner_planeseg
```



To run the tests for the SurfacePlanner with the URDF env, run:
```
hpp-rbprm-server
```
```
python3 -m walkgen_surface_planner.tests.surface_planner_urdf
```


---

## Ros interface
The module walkgen_ros allows to interface with ros message. It contains publisher interfaces, reading and transform ros messages for both modules Surface and Footstep planner. Memmo_planner_ros should be installed to run ros experiments, on feature-walkgen branch : https://github.com/thomascbrs/memmo_planner_ros/tree/feature-walkgen

#### Dependencies
numpy
rospy
pinocchio
visualization_msgs
geometry_msgs
footstep_msgs (Memmo_planner_ros)
std_msgs

---
## Installation dependencies
- Caracal: Access needed, on branch feature-walkgen.
- visualization_msgs: ```sudo apt-get install ros-noetic-visualization-msgs```
- geometry_msgs: ```sudo apt-get install ros-noetic-geometry-msgs```
- footstep_msgs: ```sudo apt-get install ros-noetic-footstep-msgs```
- rospy : ```pip3 install rospy```
- visvalingamwyatt algorithm : ```pip3 install visvalingamwyatt```
- shapely : ```pip3 install shapely```
- sl1m : https://github.com/loco-3d/sl1m/tree/devel
- Gurobi : https://www.gurobi.com/documentation/9.5/quickstart_windows/software_installation_guid.html
- pinocchio, example-robot-data, etc..:
```
sudo apt install -qqy robotpkg-py38-pinocchio
sudo apt install robotpkg-py38-qt5-gepetto-viewer-corba # Install gepetto-viewer
sudo apt install robotpkg-example-robot-data # Robot data
sudo apt install robotpkg-eigen-quadprog
sudo apt install robotpkg-py38-ndcurves
sudo apt install robotpkg-eiquadprogs
sudo apt install robotpkg-py38-example-robot-data
sudo apt install robotpkg-py38-quadprog
sudo apt install robotpkg-py38-solo-rbprm
sudo apt install robotpkg-hpp
sudo apt install robotpkg-py38-hpp-rbprm
sudo apt install robotpkg-py38-hpp-rbprm-corba
```

## TODO
- Generate a heightmap from the ros msg MarkerArray for SL1M in order to rotate the inequalities.
-  Bezier curves.
