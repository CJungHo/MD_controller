# MD_controller
This is a package that makes MDROBOT's motor driver available in ROS2(humble).

## motor driver setup(port, buadrate ...)
in launch/md_controller.launch.py

Change parameters suitable for motor driver and motor.

<img src="https://github.com/CJungHo/MD_controller/assets/91372509/191e4049-032f-4910-bbbc-4b158db60aea"  width="300" height="250"/>

## run
```
#run motor controller
        
~$ ros2 launch md_controller md_controller.launch.py

#control motor

~$ ros2 run md_teleop md_teleop_key_node
```
