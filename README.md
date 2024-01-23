# Robotiq
This package contains Robotiq resources.

# FTS-300
To bump the data rate to 100 Hz instead of the default 62.5 Hz:

``` sh
# checking for the delay
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
16

# changing the delay (non-persistant)
sudo -i
echo 2 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
2

logout
```

The output rate should now be 100 Hz.

``` sh
rostopic hz /robotiq_ft_sensor/wrench

subscribed to [/robotiq_ft_sensor/wrench]
average rate: 100.016
```

# Contribution
Originally forked from multiple sources:
- [Danfoa](https://github.com/Danfoa/robotiq_2finger_grippers)
- [TAMS-Group](https://github.com/TAMS-Group/robotiq)
- [beta-robots](https://github.com/beta-robots/robotiq)
