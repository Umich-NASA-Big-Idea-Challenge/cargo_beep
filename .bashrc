# Aliases
alias refresh="source ~/.bashrc"
alias beep_start="refresh && ros2 launch cargo_beep start_up.launch.py"
alias beep_pid="refresh && ros2 launch cargo_beep pid.launch.py"
alias beep_bag="cd ~/test_ws/src/cargo_beep/bags && ros2 bag record -a" 
alias beep_build="cd ~/test_ws && colcon build && source install/setup.sh"

alias hotspot_connect="nmcli connection up Hotspot"
alias wifi_connect="sudo nmcli dev wifi connect MWireless" 

source /opt/ros/humble/setup.sh
