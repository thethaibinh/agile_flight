#!/bin/bash

if [[ ! -f "$(pwd)/setup_py.bash" ]]
then
  echo "please launch from the agile_flight folder!"
  exit
fi

project_path=$PWD
echo $project_path

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
# echo "Run the first vision demo."
# cd $project_path/envtest 
# python3 -m python.run_vision_demo --render 1

echo "Done!"
echo "Have a save flight!"
