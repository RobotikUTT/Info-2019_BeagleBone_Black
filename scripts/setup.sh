#
#   Script for installing ros and building code
#   [sh version]
cd ..

echo "Setting up ros sources..."

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt update

echo "Installing ros and catkin-tools..."
sudo apt install -y ros-melodic-ros-base python-catkin-tools

echo "rosdep initialization..."
sudo rosdep init
rosdep update

echo "sourcing ros..."
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "building robot..."
catkin build
