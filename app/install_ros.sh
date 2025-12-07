sudo apt update
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
sudo apt install python3-rosdep python3-rosinstall-generator python3-vcstool build-essential
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ekagaku_gritcat/grit_ws/install/setup.bash" >> ~/.bashrc