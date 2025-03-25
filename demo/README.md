NOTES:
1. `horizontal_radius` (horizontal distance at which collision avoidance will be triggered) is tuned for 2m/s maximum horizontal speed (`WPNAV_SPEED 200`) - THIS MAY NEED TO BE CHANGED
2. Wait until `Avoidance detection resumed.` is displayed before inputting another command

Running the simulation:
1. Add `--instance 0` and `--instance 1` arguments to xr301 protos in webots
2. Run webots sim
3. Run following commands in order (and in seperate terminal windows)

`pathToArdupilot/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-python --add-param-file=pathToArdupilot/ardupilot/libraries/SITL/examples/Webots_XR_301/params/xr301.parm --instance 0`

`pathToArdupilot/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-python --add-param-file=pathToArdupilot/ardupilot/libraries/SITL/examples/Webots_XR_301/params/xr301.parm --instance 1`

`python3 tower_2.py`

`python3 mission.py 14550`

`python3 mission.py 14560`

Last updated: 3/16/25
