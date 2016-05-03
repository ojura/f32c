#!/bin/bash

# This script sets up a vanilla Ubuntu 16.04 system with everything needed to run the particle filter
# this includes: cloning the git repo, installing ROS Kinetic, building gcc cross-compiler toolchain with patches,
# building & installing ujprog

sudo -s -- <<"EOF"

echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
apt update
apt dist-upgrade --yes
apt install git libftdi-dev libopencv-dev python-pyqt5 python-rosdep rosbash ros-core ros-kinetic-dynamic-reconfigure ros-kinetic-geometry-msgs ros-kinetic-nav-msgs ros-kinetic-roscpp ros-kinetic-rospy ros-kinetic-rqt-gui-py ros-kinetic-sensor-msgs ros-kinetic-stage ros-kinetic-std-srvs ros-kinetic-tf ros-kinetic-rqt-reconfigure ros-kinetic-stage-ros --yes
apt-get clean


cd ~/Desktop
sudo -u $SUDO_USER mkdir -p fpgaPF
cd fpgaPF
sudo -u $SUDO_USER git clone https://github.com/ojura/f32c.git
sudo -u $SUDO_USER git clone https://github.com/f32c/tools.git
cd tools/ujprog
sudo -u $SUDO_USER make -f Makefile.linux -j8
mv ujprog /usr/bin/
chown root:root /usr/bin/ujprog
chmod u+sx,o+x /usr/bin/ujprog
adduser $SUDO_USER dialout

cd ../../f32c/src/compiler
sudo -u $SUDO_USER ./download.sh
sudo -u $SUDO_USER ./build.sh
rm -rf binutils* gcc* mpc* mpfr* gmp*

cd ../examples/pf_serial
sudo -u $SUDO_USER ln -s "$(realpath scripts/mips.sh)" ~/mips.sh
EOF
