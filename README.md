# Overview

This package provides a template to send verbal commands to a robot from a raspberry pi 4. The raspberry pi runs a rospy node which uses the hotword detection engine ["Snowboy"](https://github.com/seasalt-ai/snowboy.git) to detect a set of commands said by the user.  Then, the node publishes a string corresponding to the command, which can then be used by another node to trigger an action for the robot to execute. For example, if the node "spot_cmd_node" was running and the user said "spot sit", the string "sit" will be published. This string message could then be subscribed to by a control node and trigger Spot to sit.

This project uses custom hotword models that were generated by following the guides listed below:

- Snowboy readme.md: https://github.com/seasalt-ai/snowboy

- Video by Blue Hippo: https://youtu.be/Fsh-4m8hsc4

These guides use different methods, but both result in the creation of a "pmdl" file containing the hotword model. The hotword models for this package can be found in the "models" directory.

Hotwords included for the node "spot_cmd_node" are:

- "spot sit" - command for Spot to sit
- "spot stand" - command for Spot to stand
- "spot down" - command for Spot to lay down

# Prerequisites

- Raspberry Pi 4 with a microphone connected
- Raspbian Buster
- Python 3
- ROS Noetic (Follow this [guide](https://varhowto.com/install-ros-noetic-raspberry-pi-4/#ROS_Noetic_Raspberry_Pi##). If you want ROS with most of the common dependencies used by other packages, I recommend using "desktop" instead of "ros_comm" as it is the least tedious way of getting them on Rasbian)

# Setting Up The ROS Package

## 1. Setting up Snowboy with Python 3 for Raspberry Pi ([source](https://youtu.be/nClsUOJXsTI))

Install PyAudio and sox:

```
sudo apt-get install python-pyaudio python3-pyaudio sox
```

Install libatlas:

```
sub apt-get install libatlas-base-dev
```

Clone Snowboy from the seasalt-ai git repository (kitt-ai is discontinued):

```
git clone https://github.com/seasalt-ai/snowboy.git
```

Navigate into the "scripts" directory and run "install_swig.sh":

```
cd snowboy/scripts
./install_swig.sh
```

Navigate to the newly created directory "swig" in scripts (/snowboy/scripts/swig) and run "configure":

```
cd swig
./configure
```

Then, run "make" and "make install":

```
make
sudo make install
```

Navigate to the swig Python 3 directory outside of scripts (/snowboy/swig/Python3), then "make":

```
cd ../../swig/Python3
make
```

Navigate to the Python 3 examples directory (/snowboy/examples/Python3):

```
cd ../../examples/Python3
```

 Open the "snowboydecoder.py" and change "from . import snowboydetect" at line 5 to "import snowboydetect":

```
nano snowboydecoder.py
```

Run one of the demo files to check if it works:

```
python3 demo.py
```

## 2. Creating Environment Variables for the Package

The environment variable "SNOWBOY_PYTHON3_PATH", which is the path for the header snowboydecoder.py, needs to be added to "~/.bashrc" so the rospy node can use snowboydecoder.HotwordDetector().

Open "~/.bashrc":

```
sudo nano ~/.bashrc
```

In "~/.bashrc" add the following:

```
export SNOWBOY_PYTHON3_PATH=/home/pi/snowboy/examples/Python3 # path to snowboydecoder.py
```

After saving and closing "~/.bashrc" Re-open the terminal or type "bash" to apply the changes:

```
bash
```

## 3. Running the node or launch file

Running the node:

```
rosrun snowboy_ros spot_cmd_node
```

Running the launch file:

```
roslaunch snowboy_ros spot_cmd.launch
```

Displaying what the node publishes:

```
rostopic echo /spot_cmds
```

