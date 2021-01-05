# NH-ORCA-python
This is a pybind11 version of [rvo2](http://gamma.cs.unc.edu/RVO2/).

Based on the python wrapper, a **NH-ORCA** algorithm is implemented for <u>two-wheeled robots</u> (such as turtlebot) .

Here is the [paper](https://ieeexplore.ieee.org/document/5652073). 

The wrapper code can be checked as `py_wrapper.cpp`.

NH Turtlebot control codes can be check under the `python` folder.

It requires Eigen, openmp, numpy and pygame(for visualization).



Build 
---------------------
Building requires [CMake](http://cmake.org/) to be installed.
python need to be installed with numpy libarary
```
sudo apt-get install libeigen3-dev 
pip install numpy 
pip install pygame
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
sudo make all
cp pyrvo2*.so ../python
```
now you have the library in the python folder

## Run Simulation

```
cd python
jupyter
# Then run the rvo2_circle.ipynb
```

## Run On Real TurtleBots

For real use cases, you have to read the topics that in the `turtlebot.py`.

Each robot of index X receive odom and pose from `/turtleX/odom` and `/vrpn_client_node/turtleX/pose`, pushlish control messages (Twist ) to the topic `/turtleX/cmd_vel_mux/input/teleop`.

```
cd python
python turtleCtrl.py
```

## Data Saving 

First record all data into a ros bag. Then  make use of the script `bag_to_csv.py` for data extraction.

Optimal Reciprocal Collision Avoidance
======================================

<http://gamma.cs.unc.edu/RVO2/>

[![Build Status](https://travis-ci.org/snape/RVO2.svg?branch=master)](https://travis-ci.org/snape/RVO2)
[![Build status](https://ci.appveyor.com/api/projects/status/0nyp7y4di8x1gh9o/branch/master?svg=true)](https://ci.appveyor.com/project/snape/rvo2)

Copyright 2008 University of North Carolina at Chapel Hill

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

<http://www.apache.org/licenses/LICENSE-2.0>

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Please send all bug reports for the Python wrapper to
[RVO2-pybind11](https://github.com/dongfangliu/RVO2-pybind11), and bug
report for the RVO2 library itself to [geom@cs.unc.edu](mailto:geom@cs.unc.edu).

The RVO2 authors may be contacted via:

Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, and Dinesh Manocha  
Dept. of Computer Science  
201 S. Columbia St.  
Frederick P. Brooks, Jr. Computer Science Bldg.  
Chapel Hill, N.C. 27599-3175  
United States of America
