# A Depth-based Hybrid Approach for Safe Flight Corridor Generation in Memoryless Planning

This repository contains the flight simulation code for the algorithm described in the paper [A Depth-Based Hybrid Approach for Safe Flight Corridor Generation in Memoryless Planning](https://www.mdpi.com/1424-8220/23/16/7206), Thai Binh Nguyen, Manzur Murshed, Tanveer Choudhury, Kathleen Keogh, Gayan Kahandawa Appuhamillage, and Linh Nguyen, <em>[Sensors](https://doi.org/10.3390/s23167206)</em>. 

## About

If our repo helps your academic projects, please cite our paper. Thank you!

```
@article{Binh2023fsd,
    author={Thai Binh Nguyen, Manzur Murshed, Tanveer Choudhury, Kathleen Keogh, Gayan Kahandawa Appuhamillage, and Linh Nguyen},
    title={A Depth-Based Hybrid Approach for Safe Flight Corridor Generation in Memoryless Planning},
    journal={Sensors},
    volume={23},
    year={2023},
    number={16},
    article-number={7206},
    url={https://www.mdpi.com/1424-8220/23/16/7206},
    issn={1424-8220},
    doi={10.3390/s23167206}
}
```

Please don't hesitate to contact the corresponding author [Thai Binh Nguyen](mailto:thethaibinh@gmail.com) if you have any requests.

## Demo
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/0awjK_5kGfw/0.jpg)](https://www.youtube.com/watch?v=0awjK_5kGfw)

## Acknowledgements
This evaluation code is based on the flight testing API from [ICRA 2022 DodgeDrone Challenge: Vision-based Agile Drone Flight](https://github.com/uzh-rpg/agile_flight). We implement and evaluate navigation policies on top of the platform.

## Prerequisite

1. Before continuing, make sure to have g++ and gcc to version 9.3.0. You can check this by typing in a terminal `gcc --version` and `g++ --version`. Follow [this guide](https://linuxize.com/post/how-to-install-gcc-compiler-on-ubuntu-18-04/) if your compiler is not compatible.

2. In addition, make sure to have ROS installed. Follow [this guide](http://wiki.ros.org/noetic/Installation/Ubuntu) and install ROS Noetic if you don't already have it.

3. Install catkin tools.
```
sudo apt install python3-catkin-tools
```
4. Install [anaconda](https://www.anaconda.com/).

## Installation

We only support Ubuntu 20.04 with ROS noetic. Other setups are likely to work as well but not actively supported.

Start by creating a new catkin workspace.

```
cd     # or wherever you'd like to install this code
export ROS_VERSION=noetic
export CATKIN_WS=./fsd
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin init
catkin config --extend /opt/ros/$ROS_VERSION
catkin config --merge-devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-fdiagnostics-color

cd src
git clone https://github.com/thethaibinh/agile_flight
cd agile_flight
git checkout fsd
git submodule update --init --recursive
```

Run the `setup.bash`, it will ask for sudo permissions. Then build the packages.

```bash
./setup.bash
conda activate agileflight
catkin build
```
The internet interruption while preparing the environment might cause issues in the catkin build. To fix that, please remove the previous conda environment and run the script again.
```
conda activate base
conda env remove -n agileflight
./setup.bash
conda activate agileflight
```
## Testing approaches in the simulator:
Navigate to the workspace directory and run
```
cd ..
cd ..
source devel/setup.bash
source ~/.bashrc
conda activate agileflight
cd src/agile_flight/
```
<br>To run the the evaluation automatically, you can use the `./run.bash N` script provided in this folder. It will automatically perform `N` rollouts and then override the `evaluation.yaml` file which summarizes the rollout statistics.
