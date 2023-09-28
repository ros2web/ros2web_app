# ros2web_app

## Overview

`ros2web_app` is a web application framework that enables the development 
of web applications through YAML configuration and the creation of ROS2 packages.





[![Watch the video](https://img.youtube.com/vi/3-dwc0EN9TI/hqdefault.jpg)](https://www.youtube.com/embed/3-dwc0EN9TI)



## Installation

```bash
python3 -m pip install -r requirements.txt

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ros2web/ros2web_app.git
git clone https://github.com/ros2web/launch_api.git
cd ~/ros2_ws
colcon build
. ./install/local_setup.bash
```

## Usage

How to start the example package. Tutorial is under construction.

```bash
ros2 web server
ros2 run ros2web_app_examples turtlesim
```
