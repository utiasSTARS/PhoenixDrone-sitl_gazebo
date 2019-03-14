# Gazebo SITL plugins for PhoenixDrone

This repository is built upon RotorS from ETHZ-ASL: https://github.com/ethz-asl/rotors_simulator

Our main contribution is a custom model of the Phoenix Drone under `models/phoenixdrone/phoenixdrone.sdf` and its aerodynamic plugin with calibrated aerodyanmic coefficients under `src/gazebo_ts_aerodynamic_plugin.cpp`.

You do not need to explicitly build this library, this repo is referenced as a submodule under the main PX4 repository and will be compiled automatically when running SITL.

**If you use this simulator in academic work, please cite RotorS and our paper as per the README in the top level repository.**

## Install Gazebo Simulator

Follow instructions on the [official site](http://gazebosim.org/tutorials?cat=install) to install Gazebo. Mac OS and Linux users should install Gazebo 7.


## Protobuf

Install the protobuf library, which is used as an interface to Gazebo.

### Ubuntu Linux

```bash
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev gazebo7 libgazebo7-dev
```

### Mac OS

```bash
brew install graphviz sdformat3 eigen opencv
brew install gazebo7
```

An older version of protobuf (`< 3.0.0`) is required on Mac OS:

```bash
brew tap homebrew/versions
brew install homebrew/versions/protobuf260
```

## Build Gazebo Plugins (all operating systems)

Clone the gazebo plugins repository to your computer. IMPORTANT: If you do not clone to ~/src/sitl_gazebo, all remaining paths in these instructions will need to be adjusted.

```bash
mkdir -p ~/src
cd src
git clone https://github.com/Dronecode/sitl_gazebo.git
```

Create a build folder in the top level of your repository:

```bash
mkdir Build
```

Next add the location of this build directory to your gazebo plugin path, e.g. add the following line to your .bashrc (Linux) or .bash_profile (Mac) file:


```bash
# Set the plugin path so Gazebo finds our model and sim
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$HOME/src/sitl_gazebo/Build
# Set the model path so Gazebo finds the airframes
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/src/sitl_gazebo/models
# Disable online model lookup since this is quite experimental and unstable
export GAZEBO_MODEL_DATABASE_URI=""
```

You also need to add the the root location of this repository, e.g. add the following line to your .bashrc (Linux) or .bash_profile (Mac) file:
```bash
# Set path to sitl_gazebo repository
export SITL_GAZEBO_PATH=$HOME/src/sitl_gazebo
```

Navigate into the build directory and invoke CMake:

```bash
cd ~/src/sitl_gazebo
cd Build
cmake ..
```

Now build the gazebo plugins by typing:

```bash
make
```

### GStreamer Support
If you want support for the GStreamer camera plugin, make sure to install
GStreamer before running `cmake`: e.g. on Ubuntu with:
```
sudo apt-get install gstreamer1.0-* libgstreamer1.0-*
```

### Geotagging Plugin
If you want to use the geotagging plugin, make sure you have `exiftool`
installed on your system. On Ubuntu it can be installed with:
```
sudo apt-get install libimage-exiftool-perl
```

## Install

If you want the libraries and models to be usable anywhere on your system without
specifying their paths, install as shown below.

**Note: If you are using ubuntu, it is best to see the packaging section.**

```bash
sudo make install
```

## Testing

Gazebo will now launch when you type 'gazebo' in the shell:

```bash
. /usr/share/gazebo/setup.sh
. /usr/share/mavlink_sitl_gazebo/setup.sh
gazebo worlds/iris.world
```

Please refer to the documentation of the particular flight stack to learn how to run it against this framework, e.g. [PX4](http://dev.px4.io/simulation-gazebo.html)

## Packaging

### Deb

To create a debian package for Ubuntu and install it to your system:

```bash
cd Build
cmake ..
make
rm *.deb
cpack -G DEB
sudo dpkg -i *.deb
```
