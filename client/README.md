# Client Graphical User Interface

This client GUI is used for manual imaging target classification through the BYU AUVSI
imaging server.

## Setup

## End-user

If you just want to use the client to classify images, and don't plan on doing
any development with it, here's how you can set it up.

Ubuntu:

```bash
sudo apt install python3-pip python3-tk
pip3 install byu-auvsi-imaging-client
```

And to run you can simply enter:

```bash
./.local/bin/img_gui
```

You may also consider adding `~/.local/bin/` to your PATH:

```bash
export PATH="$HOME/.local/bin:$PATH"
```

## Development

To develop and contribute to this gui, clone the imaging repository.

This gui was built with tkinter in python3 and relies on a few packages

```bash
sudo apt install python3-tk python3-tk
./setup/dev_setup.sh
```

## Use

For development you should run the gui in its local directory:

```bash
cd ...../src/imaging/client
python gui.py
```

If installed via pip, simply run it as a script:

```bash
./~/.local/bin/img_gui
```

## Sub Functions

This gui is built with with a tkinter tab framework. Each tab is initialized at startup which creates each container label and widget on each tab. Only one tab runs at a time.

`lib/client_rest.py` contains all functions that interact with the server  
`lib/tabX.py` contains the functions for tab X  
`lib/tab_tools.py` contains helper functions used by multiple tabs  

## Future Updates

The header of each file contains possible future improvements
