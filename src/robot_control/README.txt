Test Instruction:
After compilation, 4 executable will be created: 
robot_control_node  sim_head  sim_robot_commander  sim_wheel

First start roscore, and then
cd into your ROS workspace dir, run:
roslaunch common_msgs robot.launch
This will launch wifi_msg_svr robot_control_node, sim_head, sim_wheel in dedicated terminal window.
Here to demo how the msgs start from wifi_msg_svr to robot_control and finally to hardware (sim_head and sim_wheel)
In the terminal window that run roslaunch, press 2, that will send cmds to head and wheel.
