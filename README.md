# cargo_beep
### Connectivity
To connect to the orin nano via SSH, check the discord for the ip

### Custom Bash Aliases
```refresh``` - Resources ~/.bashrc this is important because ROS alters file paths on build.
        So if you change and build the code without refreshing it will run the old version
```beep_start``` - Refreshes terminal and runs the cargo_beep startup file.
    The startup file initializes the IMU, Motor, and both Joystick nodes.
```beep_pid``` - Refreshes terminal and starts up all pid processes
```beep_bag``` - Will begin recording a ROS Bag and all topics on robot.
```beep_build``` - Builds the ROS packages and refreshes your terminal

### Need to be debugged aliases
hotspot_connect - Enables the Orin's hotspot for non-MWireless connection
wifi_connect - Should make the Orin reconnect to MWireless

### How to run the robot
1. Prepare to open 3ish terminals. Many of the commands will block your terminal.
2. Run ```beep_start```
3. Optional: Run ```beep_bag in a new terminal```
4. Run ```ros2 run [cargo_beep] (file_name)```
