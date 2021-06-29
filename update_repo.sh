#!/bin/sh
# assume conda is installed and you are in the working environment

# set the python interpreter path and python lib path here


python_int_path="${CONDA_PREFIX}/bin/python" # can also be done with "which"
python_lib_path="${CONDA_PREFIX}/lib/libpython3.8.so"

echo $python_lib_path


#git pull

cd build

cmake -DPYTHON_EXECUTABLE=$python_int_path -DPYTHON_LIBRARY=$python_lib_path -Dpybind11_DIR=`python3 -m pybind11 --cmakedir` ..

make -j4

conda develop "$(pwd)/"

# rename conda.pth to libry.pth
site_packages="${CONDA_PREFIX}/lib/python3.8/site-packages/"
#echo $site_packages
mv "${site_packages}conda.pth" "${site_packages}libry.pth"

cd ..
