#!/bin/bash

# Vars
BEGINDIR=$(pwd)
SETUPDIR=~/frcsetup/
PYV=$(python -c "import sys;t='{v[0]}.{v[1]}'.format(v=list(sys.version_info[:2]));sys.stdout.write(t)")

[ "$(whoami)" != "root" ] && exec sudo -- "$0" "$@"

# Create temp dir for install

mkdir $SETUPDIR
cd $SETUPDIR

# Git
sudo apt-get -yq install git

# Misc.
sudo apt-get -yq install curl
sudo apt-get -yq install htop
sudo apt-get -yq install ssh-askpass

# Clang
sudo apt-get -yq install clang clang-format

# Fix Python (if /usr/bin/python isn't 2.7)
if [ $PYV -eq "2.7" ]
then
  echo "Python 2.7 already configured as default!"
  .
else
  sudo rm /usr/bin/python # Avoids using -f flag
  sudo ln -s /usr/bin/python2 /usr/bin/python
fi

# Python deps
sudo apt-get -yq install python-numpy python-scipy python-matplotlib python-tk python-gflags
curl -O https://pypi.python.org/packages/d7/7f/082e2a23f8dff00bd98a7ff7db1b27a3cc66012f2db952a5fc00d8f66b13/glog-0.3.1.tar.gz
cd $(tar -axvf glog-0.3.1.tar.gz)
sudo python setup.py build
sudo python setup.py install

# FRC Toolchain (as of 2017)
sudo apt-add-repository -y ppa:wpilib/toolchain
sudo apt update
sudo apt install -yq frc-toolchain

# Bazel
sudo apt-get install openjdk-8-jdk
echo "deb [arch=amd64] http://storage.googleapis.com/bazel-apt stable jdk1.8" | sudo tee /etc/apt/sources.list.d/bazel.list
curl https://bazel.build/bazel-release.pub.gpg | sudo apt-key add -
sudo apt-get update
sudo apt-get install -yq bazel

# OpenCV
sudo apt-get install -yq libopencv-dev

# Back to where the script was executed
cd $STR

# Cleanup
rm -rf $SETUPDIR

# Exit if they don't want ZSH and NVIM (and some extras)
((!$#)) && echo No arguments supplied! && exit 1

# Otherwise
echo Installing customization options

# ZSH
sudo apt-get install zsh

# Oh my ZSH!
sh -c "$(curl -fsSL https://raw.githubusercontent.com/robbyrussell/oh-my-zsh/master/tools/install.sh)"

# Powerlevel9k
git clone https://github.com/bhilburn/powerlevel9k.git ~/.oh-my-zsh/custom/themes/powerlevel9k

# Neovim
sudo add-apt-repository ppa:neovim-ppa/stable
sudo apt-get update
sudo apt-get install neovim

# Sketchy python fix cause for my dotfiles cause why not
sudo ln -sf $(which python2.7) /usr/local/bin/python2
sudo ln -sf $(which python3) /usr/local/bin/python3

# Neovim python
sudo python2 -m pip install neovim
sudo python3 -m pip install neovim

# Why not
echo Some nice dotfiles are at:
echo https://github.com/KyleStach1678/dotfiles.git
echo https://github.com/jishnusen/dotfiles.git
