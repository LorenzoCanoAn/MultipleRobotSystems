## config file for small_worl2d
## Version: 1.0.20210517
## Author: Enrique Teruel (ET) eteruel@unizar.es
## License: CC-BY-SA

# USE: from small_worl2d_config import loginfo, logerror, shoul, showconn, SS, W, H, room, TS, RT, vN, wN

import os
from pathlib import Path
from random import seed
from numpy import pi

## Personalities
# Random seed for reproducibility
seed(0)
# Working_dir
working_dir = str(Path.home())+'/dev/MultipleRobotSystems/p3'
os.chdir(working_dir) # for files

## To info or not to info
loginfo=True # True for debugging
logerror=True
visual=True 
shoul=False # to show Soul's or not (graphically)
showconn=False # to show the graph of connections

## Window sizing; default aspect ratio W:H; coords (-W:W,-H:H); 
SS=15 # Screen Size SS in ches
if visual:
    W=16
    H=9
else:
    W=100
    H=100
room=[(-W,-H),(W,-H),(W,H),(-W,H)]

## Time Scale

# The update time should be "negligible" wrt. the faster cycle times.
# High-level Soul's should typically update about once per second,
# while Low-level Soul's would update about 10 times faster --- if not asap (T=0).
# Therefore, 0.1 s real-time must take some more time than the typical update,
# which can be measured. For my set-up, the update of a few tens of Soul's
# takes a fraction of a second, hence the simulation of 0.1 s shoud take about 1 s, so:
TS=5 # simulation-time seconds per real-time second

# For a movie with 10 fps, each redraw at "real-time" would happen each 0.1 s
# so, with TS=1, the redraw time should be RT=0.1:
RT=0.1*TS # simulation-time seconds per frame, for a 10 fps movie

# Normal max vels, in units per sec
# In accordance with the above on time scale, vels in units per second should be scaled
# The normal velocity must be a small displacement (wrt. the 2Wx2H sized canvas or np.pi) per redraw
# Max vel's should be a fraction of vN and wN
vN=1/TS # traverse 1 unit distance in 1 sec real-time
wN=60*2*pi/(60*TS) # 60 rpm real-time