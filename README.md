# Using Nengo with V-REP

## Installing V-REP

Go to http://www.coppeliarobotics.com/downloads.html

Click the appropriate link for your system for V-REP PRO EDU V3.4.0

#### Mac/Linux

Unzip the file and you are good to go. Run it by running the vrep.sh script that it comes with. This can be done by navigating to the unzipped folder and running ./vrep.sh

#### Windows

After you install it there should be an exe you can use to run it

## Setting up the Python API

Instructions can be found [here](http://www.coppeliarobotics.com/helpFiles/en/remoteApiClientSide.htm).
Basically Python needs to know the location of three files (vrep.py, vrepConst.py and remoteApi.extension)

The first two are found in /path/to/vrep/programming/remoteApiBindings/python/python

The last file is in /path/to/vrep/programming/remoteApiBindings/lib/lib/

There is a 32 bit and 64 bit version of this file. I think it has to match with the version of Python you have installed. If one doesn't work, just try the other.

The easiest thing to do is to copy these files into the directory where you are running your Python scripts from. On Mac/Linux you can also just soft link them to your current directory with:

ln -s /path/to/file/filename.extension

If V-REP is extracted to your Downloads directory and you are currently in the directory with your code (e.g. robotics_simulator) you can set everything up with the following commands:

`ln -s ~/Downloads/V-REP_PRO_EDU_V3_4_0_Linux/programming/remoteApiBindings/python/python/vrep.py .`
`ln -s ~/Downloads/V-REP_PRO_EDU_V3_4_0_Linux/programming/remoteApiBindings/python/python/vrepConst.py .`
`ln -s ~/Downloads/V-REP_PRO_EDU_V3_4_0_Linux/programming/remoteApiBindings/lib/lib/64Bit/remoteApi.so .`

In some versions of V-REP the `vrep.py` file does a bad job of trying to find `remoteApi.so`.
You'll see an error when importing `vrep` if this happens.
To fix this, edit the `vrep.py` file and change one line (likely line 48) from:

`libfullpath = os.path.join(os.path.dirname(__file__), 'remoteApi' + file_extension)`

to:

`libfullpath = './remoteApi' + file_extension`

A list of available commands you can use from Python can be found [here](http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm) and organized by category [here](http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionListCategory.htm)

A list of available commands you can use from Python can be found [here](http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm) and organized by category [here](http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionListCategory.htm)

## Scene Files for the Tutorial

Download those [here](https://www.dropbox.com/sh/d5uhpu0inp1p4jo/AABFN2Eo3cIHfF6F5I3p3_Pza?dl=0). You can download each of them individually, or all at once by multi-selecting them.

## Running within Nengo GUI

There is no explicit support for using V-REP with `nengo_gui`, it just happens to work (most of the time).
With this in mind, there is a specific order of steps for running both simulators to ensure they play nice together.

1. Launch V-REP (`./vrep.sh`) and open the specific scene file of interest (`my_scene.ttt`).
2. Make sure the V-REP simulation is stopped (press the blue square).
  * It defaults to this state when first opened.
  * You may have to close your nengo script if it is open at this point.
3. Launch your nengo script with the gui (`nengo my_script.py`). 
  * When the script first opens it will establish a connection with V-REP and start the simulation (the blue triangle in V-REP will become pressed).
  * Nothing in V-REP should be moving at this point, even though the simulation has started. This is because nengo has taken control of the time step.
  * If you set the `sync` flag to `False` on the `Robot` object in python, things will move at this point, but it defaults to `True`.
4. You are good to go, press play in the `nengo_gui` and enjoy!

If something goes wrong, which can happen sometimes when editing the code live, repeat steps 2-4.
