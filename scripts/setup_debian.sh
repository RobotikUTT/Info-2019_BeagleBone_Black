# Install dev dependencies
sudo apt install -y ros-base-dev python-pip python3-pip

# Updating deps
sudo rosdep init
rosdep update

# Install python dependencies 
sudo pip3 install rospkg python-can
sudo pip install Adafruit_BBIO