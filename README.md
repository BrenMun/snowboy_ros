# Overview
## Purpose of the Package
This package provides a means to send verbal commands to a robot from a raspberry pi 4. The raspberry pi runs a rospy node which uses the hotword detection engine ["Snowboy"](https://github.com/seasalt-ai/snowboy.git) to detect a set of hotwords supplied by the user. The node then publishes a string message corresponding to a detected hotword, which could then be subscribed to by another node to trigger the robot to perform an action.

## Sample Hotword Models
Hotword models included in the package are:
- "spot_sit.pmdl" - hotword model for the phrase "spot sit"
- "spot_stand.pmdl" - hotword model for the phrase "spot stand"
- "spot_down.pmdl" - hotword model for the phrase "spot down"

When the node is run, the hotword detector will wait until one of these phrases are detected. It will then publish the name of the file corresponding to the detected phrase. For example, if the phrase "spot sit" was detected, the node would then publish the string "spot_sit". These sample models have only been trained with one voice type (Australian male). Therefore, it is not a guarantee that they work for everyone.

## Using Your Own Personal Hotword Models
The node "hotword_publisher.py" is able to use a dynamic number of pmdl files as inputs. As long as there is at least 1 pmdl file in the "models" directory, the node will work. As such, the user is not limited to only using the pmdl files provided. If they wanted to use their own hotword models they could.

Creating a personal hotword model can be done by following one of the guides listed below:

- Snowboy readme.md: https://github.com/seasalt-ai/snowboy

- Video by Blue Hippo: https://youtu.be/Fsh-4m8hsc4

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

The environment variable "SNOWBOY_PYTHON3_PATH", which is the path for the header snowboydecoder.py, needs to be added to "~/.bashrc" so the rospy node can use snowboy's hotword detector.

Open "~/.bashrc":

```
sudo nano ~/.bashrc
```

In "~/.bashrc" add the environment variable with the path to snowboydecoder.py:

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
rosrun snowboy_ros hotword_publisher.py
```

Running the launch file:

```
roslaunch snowboy_ros hotword_pub.launch
```

Displaying what the node publishes:

```
rostopic echo /cmd_msgs
```

