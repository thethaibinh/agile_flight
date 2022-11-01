# DESS - DEpth-based Sampling and Steering Constraints for Memoryless Local Planners

## Demo
![Fixed yawing](https://github.com/thethaibinh/agile_flight/blob/master/evaluation_results/fixed_yawing.gif?raw=true)
<br> Local planners using the fixed yawing method get stuck more often when facing a large obstacle.

![DESS](https://github.com/thethaibinh/agile_flight/blob/master/evaluation_results/depth-based_steering.gif?raw=true)
<br> DESS navigate through all large convex obstacles.

## Acknowledgements
This evaluation code is based on the flight testing API from [ICRA 2022 DodgeDrone Challenge: Vision-based Agile Drone Flight](https://github.com/uzh-rpg/agile_flight). We implement and evaluate navigation policies on top of the platform.

### Prerequisite

1 . Before continuing, make sure to have g++ and gcc to version 9.3.0. You can check this by typing in a terminal `gcc --version` and `g++ --version`. Follow [this guide](https://linuxize.com/post/how-to-install-gcc-compiler-on-ubuntu-18-04/) if your compiler is not compatible.

2. In addition, make sure to have ROS installed. Follow [this guide](http://wiki.ros.org/noetic/Installation/Ubuntu) and install ROS Noetic if you don't already have it.

3. Install catkin tools.
```
sudo apt install python3-catkin-tools 
```
4. Install [anaconda](https://www.anaconda.com/).

### Installation

We only support Ubuntu 20.04 with ROS noetic. Other setups are likely to work as well but not actively supported.

Start by creating a new catkin workspace.

```
cd     # or wherever you'd like to install this code
export ROS_VERSION=noetic
export CATKIN_WS=./dess
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin init
catkin config --extend /opt/ros/$ROS_VERSION
catkin config --merge-devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-fdiagnostics-color

cd src
git clone https://github.com/thethaibinh/agile_flight
cd agile_flight
```

Run the `setup_ros.bash` and `./setup_py.bash` sequentially in the main folder of this repository, it will ask for sudo permissions. Then build the packages.

```bash
./setup_ros.bash
./setup_py.bash
catkin build
```

**Testing approaches in the simulator:**
Navigate to the workspace directory and run
```
source devel/setup.bash
cd src/agile_flight/
```
To run the the evaluation automatically, you can use the `./run.bash N` script provided in this folder. It will automatically perform `N` rollouts and then create an `evaluation.yaml` file which summarizes the rollout statistics.

**Visualize results**
Change the name for each result file as "result_easy", "result_medium", "result_hard" corresponding to the environment.
Then run plot script in /evaluation_results.