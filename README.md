# Setting Up The ROS Package

## 1. Setting up Snowboy with Python 3 for Raspberry Pi

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

Navigate to the newly created directory "swig" in scripts and run "configure":

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

Run one of the demo files:

```
python3 demo.py
```

## 2. Creating Environment Variables for the Package

Environment variables are used so file directories are not hard coded in the node. Two environment variables are used in the ROS node, "CATKIN_WS_PATH" and "SNOWBOY_PYTHON3_PATH". 

CATKIN_WS_PATH: is the directory the catkin workspace source folder. This is needed for importing the pmdl files for hotword detection.

SNOWBOY_PYTHON3_PATH:  is the directory of "snowboydecoder.py", the class 

Open ~/.bashrc:

```
sudo nano ~/.bashrc
```

In ~/.bashrc add the following environment variables:

```
export SNOWBOY_PYTHON3_PATH=/home/pi/snowboy/examples/Python3 # path to snowboy py3 resources
export CATKIN_WS_PATH=/home/pi/catkin_ws/src				  # path to catkin_ws/src
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

