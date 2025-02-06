#!/bin/bash

# Check and configure UTF-8 locale
locale  # Check current locale settings

sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # Verify locale settings

# Install dependencies for adding repositories
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Update package list and install curl
sudo apt update && sudo apt install -y curl

# Add ROS 2 repository key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and upgrade system
sudo apt update
sudo apt upgrade -y

# Install ROS 2 Humble
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-ros-base
sudo apt install -y ros-dev-tools

# Source ROS 2 setup script automatically in bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "ROS 2 Humble installation completed. Open a new terminal or run 'source ~/.bashrc' to use ROS 2."
