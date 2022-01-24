# memmo_anymal

Workspace containing the works related to memmo project and the anymal demo.

--> PlaneSeg and SL1M integration.

--> Caracal and SL1M integration.

Dependencies:
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

Launch an exemple of PlaneSeg-->SL1M, from memmo_anymal folder:
```
python3 walkgen/stand_alone_scenarios/anymal_stairs.py
```