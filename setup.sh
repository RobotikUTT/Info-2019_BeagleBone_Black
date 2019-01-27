echo "Setting up ros sources..."

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt update

echo "Installing ros..."
sudo apt install ros-melodic-ros-base

echo "rosdep initialization..."
sudo rosdep init
rosdep update

echo "building robot..."
catkin_make