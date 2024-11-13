# ubxBase

Simple python script to configure / survey ublox base stations. 

Ugly code, based on an example in [pyubx](https://github.com/semuconsulting/pyubx2).


## Why?

Because fuck u-center

## Workflow


- go to the test location, place the base station
- ssh into the base station
- run this script, e.g. `./ubxBase.py --port /dev/ttyACM0 --survey_in 500 40` which will set the ublox receiver in survey mode for at least 500 seconds until an accuracy of 40 cm is reached.
- statistics like fix and survey-in status are displayed in the terminal
  - if you exit the program (`ctrl-c`), the survey-in mode will still run. If you want to see the status again, w/o starting a new survey-in from scratch, simply run `./ubxBase.py --port /dev/ttyACM0 --mon True` (monitor)
  - you can also (_almost_) tell the receiver where it is at, using the `--fixed_pos_ecef` or `--fixed_pos_llh` flags, **but there is a bug**
- send RTCM data from the base to the rover, using e.g. RTKLIBs str2str (ubxBase does not do this for you) 
- deploy robot, or whatever

This script does not do any config of the receiver (i.e. setting up what RTCM messages to send), or forward RTCM messages to the robot. For that, see e.g. [ublox F9P moving base app note](https://content.u-blox.com/sites/default/files/documents/ZED-F9P-MovingBase_AppNote_UBX-19009093.pdf) and [RTKLIB](https://github.com/rtklibexplorer/RTKLIB/) str2str.

## Status

WIP, but it works for my needs. PRs and forks welcome!


