# Client Graphical User Interface

## Setup
This gui was built with tkinter in python3 and relies on a few packages

Install the following python modules:  
```
pip install ttkthemes
```


```
sudo apt install python3-tk
pip install Pillow, opencv-python, ttkthemes, imutils
```
## 
## Functionality
To run the gui:
`python3 gui.py`

This gui is built with with a tkinter tab framework. Each tab is initialized at startup which creates each container label and widget on each tab. Only one tab runs at a time.

## Sub Functions
`lib/client_rest.py` contains all functions that interact with the server  
`lib/tabX.py` contains the functions for tab X  
`lib/tab_tools.py` contains helper functions used by multiple tabs  

## Future Updates
The header of each file contains possible future improvements
