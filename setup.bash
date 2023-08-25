#!/bin/bash

if [[ ! -f "$(pwd)/setup.bash" ]]
then
  echo "please launch from the racing folder!"
  exit
fi

project_path=$PWD
echo $project_path

echo "Making sure submodules are initialized and up-to-date"
git submodule update --init --recursive

echo "Using apt to install dependencies..."
echo "Will ask for sudo permissions:"
sudo apt update
sudo apt install -y --no-install-recommends build-essential cmake libzmqpp-dev libopencv-dev unzip python3-catkin-tools
sudo pip install uniplot
sudo apt-get install ros-noetic-rqt ros-noetic-rqt-common-plugins ros-noetic-rqt-robot-plugins ros-noetic-pcl-ros

echo "Ignoring unused Flightmare folders!"
touch flightmare/flightros/CATKIN_IGNORE

echo "Downloading Flightmare Unity standalone..."
wget "https://github.com/thethaibinh/flightmare_unity/releases/download/DESS/RPG_Flightmare_Data.zip" --directory-prefix=$project_path/flightmare/flightrender

echo "Unziping Flightmare Unity Standalone... (this might take a while)"
unzip -o $project_path/flightmare/flightrender/RPG_Flightmare_Data.zip -d $project_path/flightmare/flightrender | awk 'BEGIN {ORS=" "} {if(NR%10==0)print "."}'

echo "Removing Flightmare Unity Standalone zip file"
rm $project_path/flightmare/flightrender/RPG_Flightmare_Data.zip

chmod +x $project_path/flightmare/flightrender/RPG_Flightmare.x86_64

echo "Setting the flightmare environment variable. Please add 'export FLIGHTMARE_PATH=$PWD/flightmare' to your .bashrc!"
export FLIGHTMARE_PATH=$project_path/flightmare

#
echo "export FLIGHTMARE_PATH=$project_path/flightmare" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-pip
#
echo "Createing an conda environment from the environment.yaml file. Make sure you have anaconda installed"
conda env create -f environment.yaml

#
echo "Source the anaconda environment. If errors, change to the right anaconda path."
source ~/anaconda3/etc/profile.d/conda.sh

#
echo "Actiavte the environment"
conda activate agileflight

echo "Compiling the agile flight environment and install the environment as python package"
cd $project_path/flightmare/flightlib/build
cmake ..
make -j10
pip install .


echo "Install RPG baseline"
cd $project_path/flightmare/flightpy/flightrl
pip install .

pip install opencv-contrib-python flightgym rpg_baselines
# apt install required packages
sudo apt update
sudo apt install -y zip htop screen libgl1-mesa-glx

# pip install required packages
pip install seaborn thop torch==1.13.0 torchvision==0.14.0

echo "Done!"
echo "Have a save flight!"
