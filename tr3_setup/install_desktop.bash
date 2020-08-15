#!/usr/bin/env bash

while true; do
	read -p "You must enable main, universe, restricted, and multiverse repositories in the Software & Updates program to continue. If you try to run this without doing so, you will have a bad time. Have you completed this? [y/n] `echo $'\n> '`" yn
	case $yn in
		[Yy]* ) break;;
		[Nn]* ) exit;;
		* ) echo "Please answer yes or no.";;
	esac
done

sudo apt-get update

echo "Installing ROS Melodic"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update
sudo apt-get install ros-melodic-desktop-full -y

sudo apt-get install python-pip -y
sudo pip install -U rosdep
sudo rosdep init
rosdep update

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential -y
sudo apt-get install ros-melodic-moveit ros-melodic-joy ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-gazebo-ros-control ros-melodic-navigation ros-melodic-web-video-server ros-melodic-ros-numpy ros-melodic-tf2-sensor-msgs ros-melodic-slam-gmapping ros-melodic-navigation -y

echo "Installing TR3 Packages"
mkdir ~/ros_ws
mkdir ~/ros_ws/src
cd ~/ros_ws/src
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
git clone https://github.com/slaterobotics/tr3_essentials
cd ~/ros_ws
catkin_make
source ~/.bashrc

echo "Installing Node.js"
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.3/install.sh | bash
export NVM_DIR="$([ -z "${XDG_CONFIG_HOME-}" ] && printf %s "${HOME}/.nvm" || printf %s "${XDG_CONFIG_HOME}/nvm")"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh" # This loads nvm
nvm install 12.18.3
cd ~/ros_ws/src/tr3_essentials/tr3_os/src/
npm i
cd ~/ros_ws

echo "Install complete."
