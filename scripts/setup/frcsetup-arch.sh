#!/bin/bash

# Vars
BEGINDIR=$(pwd)
SETUPDIR=~/frcsetup/
PYV=$(python -c "import sys;t='{v[0]}.{v[1]}'.format(v=list(sys.version_info[:2]));sys.stdout.write(t)")

# Create temp dir for install

mkdir $SETUPDIR
cd $SETUPDIR

# Setuptools
# Cower
sudo pacman -S git
sudo pacman -S base-devel
sudo pacman -S openssl
git clone https://aur.archlinux.org/cower.git
cd cower
makepkg -s
sudo pacman -U *.pkg.tar.xz
#PacAUR
git clone https://aur.archlinux.org/pacaur.git
cd pacaur
makepkg -s
sudo pacman -U *.pkg.tar.xz

# Python
pacaur -S python2
pacaur -S python2-pip
pip install numpy scipy matplotlib tk gflags glog

# OpenCV2
pacaur -S opencv2-git

# Toolchain
pacaur -S frc-2017

# Bazel
pacaur -S bazel

# Fix Python (if /usr/bin/python isn't 2.7)
if [ $PYV -eq "2.7" ]
then
  echo "Python 2.7 already configured as default!"
  .
else
  sudo rm /usr/bin/python # Avoids using -f flag
  sudo ln -s /usr/bin/python2 /usr/bin/python
fi

# Back to where the script was executed
cd $STR

# Cleanup
rm -rf $SETUPDIR
