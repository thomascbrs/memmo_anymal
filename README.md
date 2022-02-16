# memmo_anymal

Workspace containing the works related to memmo project and the anymal demo.

--> PlaneSeg and SL1M integration.

--> Caracal and SL1M integration.

## Dependencies
- Caracal: Access needed, on branch feature-walkgen.
- visualization_msgs: ```sudo apt-get install ros-noetic-visualization-msgs```
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
## Walkgen
Install cmake submodule:
```
git submodule init
git submodule update
```

Build the walkgen library:
```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=~/install -DPYTHON_EXECUTABLE=$(which python3.8) -DPYTHON_STANDARD_LAYOUT=ON
```

## Examples

Example of the SurfacePlanner with the convex patches returned by planeseg. Run :
```
python3 -m walkgen.stand_alone_scenarios.anymal_stairs_planeseg
```


Example of the SurfacePlanner with the URDF and heightmap of the environment. Run :
```
hpp-rbprm-server
```
In another terminal, run :
```
python3 -m walkgen.stand_alone_scenarios.anymal_stairs_urdf
```

## Test

To test with planeseg data, run:
```
python3 -m walkgen.tests.surface_planner_urdf
```



To run the tests for the SurfacePlanner with the URDF env, run:
```
hpp-rbprm-server
```
```
python3 -m walkgen.tests.surface_planner_urdf
```

## Heightmap
Create an heightmap from the URDF environment. Usefull to use the SurfacePlanner with the guide path RBPRM, for the simulations.
- Setup the variable DEVEL_DIR, with the path of this repo (temporary): This should be removed, to be tested.

```export DEVEL_DIR=your_path```
- Setup the ROS_PACKAGE_PATH variable environment in your /.bashrc file, for hpp. This allow to define in the urdf the paths as "package://meshes/lab_scene.stl"  :

```export ROS_PACKAGE_PATH=your_path/memmo_anymal/walkgen/data:$ROS_PACKAGE_PATH ```
An example of environment (URDF and STL files) has been put in the data folder.
To create a new heightmap, open the config file and tune the path (heightmap and urdf environment). The binary file will be created in the data folder with the new name. Open a terminal and run :
```
$ hpp-rbprm-server
```

In another terminal, from the main folder, run :
```
$ python3 walkgen/tools/heightmap_generator.py
```

Temporary setup for path to devel directory
```
$ sh -c "echo \"export DEVEL_DIR=~/devel\" >> ~/.bashrc"
```

## TODO

- Generate a heightmap from the ros msg MarkerArray for SL1M in order to rotate the inequalities.
-  Bezier curves.
- Check the QP for Footstepplanner.
- more tests
