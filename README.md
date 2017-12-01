## A companion computer and external nav sensor emulator

**Ensure you are using the correct MAVLink repo for both Ardupilot and MAVProxy:** https://github.com/SamuelDudley/mavlink/tree/slam 

  - Launch ArduPilot SITL (ArduPlane):
  ``-S -I0 --home -30.93484,136.54492,138,0 --model jsbsim –speedup=1``

  - Launch MAVProxy in SITL mode:
  ``--master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14551 --out 127.0.0.1:14550  --console --map --mav20 --moddebug=3``
```
param set EXTNAV_TYPE 1
param set EK3_ENABLE 1
param set AHRS_EKF_TYPE 3
param set CAM_SIM_CAP_RATE 10
```

  - Launch companion_emulator.py:
Assuming the correct MAVLink version is used the emulator will connect to the SITL and begin providing position estimates

  - Edit companion_errors.py to control the bias, noise, walk rate and direction of the sensor estimate. This code uses a synthetic  clock which is based off the AP time. If the SITL is ran faster than real time the errors will grow accordingly.

For help with the launch options of companion_emulator.py use `-h`
