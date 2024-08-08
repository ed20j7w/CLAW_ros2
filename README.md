1) Run microros agent on PC so PC can see microros publishing
  (https://github.com/micro-ROS/micro_ros_platformio)
	$ ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
	OR
	$ ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888


3) Upload micro-ros_publisher_wifi to the esp32 (req. vscode-platformio)
	It should connect to the wifi (look for FIXME tags in the code to update with correct wifi)
	This package translates between ros2 and microros


4) Upload dxl_motor_controller_serial (arduino ide) to the openrb150 board and connect the serial to esp32
	Debug serial will output the data received by the RB over serial (a.aa,b.bb;c.cc)


5) At this point Twist msgs can be sent from the pc command line
	$ ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 10.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
	(linear x - forward, linear y - claw, angular z - turning)


6) Plug in xbox360 controller and check the port number
	$ evtest


7) Update remote_control.py FIXME with the correct event number (eg 26)
8) Run remote_control.py on PC
	$ ros2 run servo_control_py remote_control


9) View the Twist msgs being sent
	ros2 topic echo /cmd_vel

	
