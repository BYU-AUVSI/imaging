# Client Graphical User Interface

## Setup
This gui was built with tkinter in python3 and relies on a few packages

Clone the [inertial_sense_ros](https://github.com/BYU-AUVSI/inertial_sense_ros)
and [uav_msgs](https://github.com/BYU-AUVSI/uav_msgs) repos from the
[BYU-AUVSI](https://github.com/BYU-AUVSI) if you haven't already.

Install the needed dependencies:  
```
sudo apt install python3-tk python-tk
pip install Pillow opencv-python ttkthemes requests imutils
```
## Use
To run the gui note you must be in its local directory:
```
cd ~/<catkin_ws location>/src/imaging/client
python gui.py
```

## Sub Functions
This gui is built with with a tkinter tab framework. Each tab is initialized at startup which creates each container label and widget on each tab. Only one tab runs at a time.

`lib/client_rest.py` contains all functions that interact with the server  
`lib/tabX.py` contains the functions for tab X  
`lib/tab_tools.py` contains helper functions used by multiple tabs  

## Future Updates
The header of each file contains possible future improvements
