# STAMINER - Online State-to-State Time-Optimal Trajectory Planning for Quadrotors in Unknown Cluttered Environments

This repository contains the implementation code for the algorithm described in the paper "[Online State-to-State Time-Optimal Trajectory Planning for Quadrotors in Unknown Cluttered Environments](https://ieeexplore.ieee.org/document/10556839)", [ICUAS ’24](https://uasconferences.com/2024_icuas/). Please don't hesitate to contact the corresponding author [Thai Binh Nguyen](mailto:thethaibinh@gmail.com) if you have any requests.

## Demonstration video
[![STAMINER](https://img.youtube.com/vi/85Y4ae2CjIo/0.jpg)](https://www.youtube.com/watch?v=85Y4ae2CjIo)

## Update

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
export CATKIN_WS=./STAMINER
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
echo "source $PWD/devel/setup.bash" >> ~/.bashrc
catkin init
catkin config --extend /opt/ros/$ROS_VERSION
catkin config --merge-devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-fdiagnostics-color

cd src
git clone https://github.com/thethaibinh/STAMINER
cd STAMINER
```

Run the `setup_ros.bash` and `./setup_py.bash` sequentially in the main folder of this repository, it will ask for sudo permissions. Then build the packages.

```bash
./setup.bash
conda activate agileflight
source ~/.bashrc
catkin build
```
You may need to run `catkin build` two time for all packages recognize each others.

**Testing approaches in the simulator:**
Navigate to the workspace directory and run
```
source devel/setup.bash
cd src/STAMINER/
```
To run the the evaluation automatically, you can use the `./run.bash N` script provided in this folder. It will automatically perform `N` rollouts and then create an `evaluation.yaml` file which summarizes the rollout statistics.

**Visualize results**
