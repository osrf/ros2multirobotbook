## Installation of the RMF Essentials

The current version of RMF is 2021.10, and it targets ROS Foxy, and Debian binary packages are released for Ubuntu Focal Fossa 20.04 LTS.

RMF leverages the ROS and Gazebo-Ignition ecosystem. We will have to setup their repositories as the first steps of the RMF installation process. Since the current binary release targets Ubuntu Focal please ensure you match the system and version before proceeding with the installation. If you have ROS 2 and Gazebo installed you can directly skip to the [Setup Sources and Installation of RMF](#setup-sources-and-installation-of-rmf) section.

> For the most updated installation instruction, please refer to [here](https://github.com/open-rmf/rmf).

### Setup Locale

Make sure you have a locale which supports UTF-8. If you are in a minimal environment, such as a docker container, the locale may be something minimal like POSIX. You can check your locale by running `locale` directly in your terminal.

We test with the following settings. It should be fine if you’re using a different UTF-8 supported locale. In case you need it, here is an example on how to switch to the US English UTF-8 locale:

```
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Setup and Install ROS 2

You will need to add the ROS 2 apt repositories to your system. To do so, first authorize our GPG key with apt like this:

```
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

And then add the repository to your sources list:

```
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
```

If ROS 2 is not installed using at least `ros-foxy-ros-base` you will need the following packages to run the RMF demos:

```
sudo apt-get install ros-foxy-ros2cli ros-foxy-ros2run ros-foxy-ros2launch
```

In order to run ROS 2 commands we need to source the `setup.bash` file:

```
source /opt/ros/foxy/setup.bash
```

### Setup Ignition-Gazebo Sources

You will also need to add the Ignition-Gazebo apt repositories similar to the ROS 2 setup. Let's authorize the key:

```
curl -s http://packages.osrfoundation.org/gazebo.key | sudo apt-key add -
```

You will also need to add the repository to your sources list:

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

### Setup Sources and Installation of RMF

> To install RMF from source, please refer to [here](https://github.com/open-rmf/rmf)

Next you will need to setup the source of RMF packages similar to the previous two steps. First authorizing the key:

```
curl -s http://rmf.servehttp.com/repos.key | sudo apt-key add -
```

Secondly adding the repository to the apt sources:

```
sudo sh -c 'echo "deb http://rmf.servehttp.com/ubuntu/main/ `lsb_release -cs` main" > /etc/apt/sources.list.d/rmf.list'
```

And now we are ready to install. Let's update our packages:

```
sudo apt-get update
```

Finally, the following installs a basic set of packages that should get you going with RMF:

```
sudo apt-get install ros-foxy-ament-cmake-catch2 ros-foxy-building-gazebo-plugins ros-foxy-building-map-msgs ros-foxy-building-map-tools ros-foxy-rmf-cmake-uncrustify ros-foxy-rmf-dispenser-msgs ros-foxy-rmf-door-msgs ros-foxy-rmf-fleet-adapter ros-foxy-rmf-fleet-msgs ros-foxy-rmf-lift-msgs ros-foxy-rmf-task-msgs ros-foxy-rmf-traffic-msgs ros-foxy-rmf-traffic-ros2 ros-foxy-rmf-traffic ros-foxy-rmf-utils ros-foxy-traffic-editor
```

## Install and run RMF demos

You can install the provided RMF demos from their Debian package:

```bash
# Demos example with gazebo simulator, use ros-foxy-rmf-demos-ign for ignition
sudo apt-get install ros-foxy-rmf-demos-gz
```
