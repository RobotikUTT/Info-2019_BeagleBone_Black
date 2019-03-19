### Installation for production purpose ###

#auto can0
#iface can0 inet manual
#    pre-up /sbin/ip link set $IFACE type can bitrate 125000 listen-only off
#    up /sbin/ifconfig $IFACE up
#    down /sbin/ifconfig $IFACE down

# Regular installation
source ./setup.sh

# Install tree overlay
echo "Install beagle bone overlays to provide tools for pins"
sudo apt install bb-cape-overlays

# Setup can devices
sudo modprobe can
sudo modprobe can-raw
sudo modprobe can-dev
sudo ip link set can0 up type can bitrate 500000

# GPIO library
sudo apt install -y build-essential python-dev python-pip
pip install Adafruit_BBIO