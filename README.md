**ICAR NG** | Autonomous Car Prototype

This repository is used to store thesis related files and code. I hope this will be useful for future students who will work on this project. As of today, this project is being reworked from scratch. Me myself, not a big fan of writing documentation, but I will try to keep it up to date.

## Dependencies

### realsense2_camera

`realsense2_camera` is a ROS package for Intel RealSense cameras. This package can be installed using commands below:

```bash
# Register the server's public key:
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

# Make sure apt HTTPS support is installed:
sudo apt-get install apt-transport-https

# Add the server to the list of repositories:
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update

# Install the libraries:
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils

# Optionally install the developer and debug packages:
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg

# Install ROS wrapper:
sudo apt install ros-humble-realsense2-*
```

### libserialport

`libserialport` is a minimal, cross-platform shared library written in C that is intended to take care of the OS-specific details when writing software that uses serial ports. This package can be installed using commands below:

```bash
# Clone the repository:
git clone git://sigrok.org/libserialport && cd libserialport

# Build and install:
./autogen.sh
./configure
make
sudo make install

# Update shared library cache:
sudo ldconfig

# Clean up:
cd .. && rm -rf libserialport
```

### GeographicLib

`GeographicLib` is a small set of C++ classes for performing conversions between geographic, UTM, UPS, MGRS, geocentric, and local cartesian coordinates, for gravity (e.g., EGM2008), geoid height, and geomagnetic field (e.g., WMM2015) calculations, and for solving geodesic problems. This package can be installed using commands below:

```bash
# Download the source code:
wget https://sourceforge.net/projects/geographiclib/files/distrib-C++/GeographicLib-2.3.tar.gz

# Extract the archive:
tar xfpz GeographicLib-2.3.tar.gz && cd GeographicLib-2.3

# Build and install:
mkdir BUILD
cd BUILD
cmake ..
make
sudo make install

# Update shared library cache:
sudo ldconfig

# Clean up:
cd ../.. && rm -rf GeographicLib-2.3*
```

### Other dependencies

Several other dependencies are required to build this project. These dependencies can be installed using commands below:

```bash
sudo apt install ros-humble-imu-tools
sudo apt install libyaml-cpp-dev
sudo apt install libpcap-dev
```
