#
#   Script for installing ros and building code
#   [sh version]
#cd ..
echo "Setting up ros sources..."

# En cas de problème : https://www.ros.org/install/
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt update

echo "Installing ros and catkin-tools..."
sudo apt install -y ros-melodic-ros-base python-catkin-tools ros-melodic-roslint

echo "Installing python3-pip and python3-rospkg"
sudo apt-get install -y python3-pip
pip3 install rospkg python-can Adafruit_BBIO pyserial

echo "rosdep initialization..."
sudo rosdep init
rosdep update

# A adapter en cas d'utilisation de zsh (remplacer bash par zsh)
echo "sourcing ros..."
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "building robot..."
catkin build
