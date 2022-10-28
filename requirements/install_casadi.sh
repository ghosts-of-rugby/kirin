#! /bin/bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
cd $SCRIPT_DIR

sudo apt install gcc g++ gfortran git cmake liblapack-dev pkg-config
sudo apt install coinor-libipopt-dev

git clone https://github.com/casadi/casadi.git casadi

cd casadi; mkdir build; cd build
cmake -DWITH_IPOPT=true .. && make && sudo make install  # install at /usr/local/lib
# make python

cd $SCRIPT_DIR
rm -rf casadi
