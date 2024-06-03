#!/usr/bin/env bash

# Fail on first error.
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo -e "\033[40;32m${DIR} \033[0m"

# download
wget https://github.com/casadi/casadi/releases/download/3.5.5/casadi-3.5.5-1.tar.gz
tar -zxvf casadi-3.5.5-1.tar.gz
echo -e "\033[40;32mdownload finish \033[0m"

cd casadi-3.5.5.1
mkdir build && cd build
cmake .. -DWITH_IPOPT=ON -DWITH_EXAMPLES=OFF
make -j4
sudo make install
sudo ldconfig

# Clean up.
sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
sudo rm -fr casadi-3.5.5-1.tar.gz casadi-3.5.5.1
