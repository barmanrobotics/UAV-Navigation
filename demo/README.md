Last updated: 3/10/25
1. Run webots sim
2. Add `--instance 0` and `--instance 1` arguments to xr301 protos in webots
3. Run following commands in order (and in seperate terminal windows)
`pathToArdupilot/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-python --add-param-file=pathToArdupilot/ardupilot/libraries/SITL/examples/Webots_XR_301/params/xr301.parm --instance 0`
`pathToArdupilot/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-python --add-param-file=pathToArdupilot/ardupilot/libraries/SITL/examples/Webots_XR_301/params/xr301.parm --instance 1`
`python3 tower_2.py`
`python3 mission_2.py 14550`
`python3 mission_2.py 14560`
