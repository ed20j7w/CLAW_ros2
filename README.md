Ubuntu Linux - Jammy Jellyfish (22.04)
ROS2 Humble Desktop
VS Code w/ platformio

OpenRB-150 to control dynamixel motors (XC330-M288-T).
ESP32-Feather V1 (Huzzah32) running Microros for communication over WiFi.
Arduino NiclaVision streaming video to local network port.

Setup:

1) Run microros agent on PC so PC can see microros publishing (https://github.com/micro-ROS/micro_ros_platformio)

	$ ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
	OR
	$ ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888


2) Upload micro-ros_publisher_wifi to the esp32 (req. vscode-platformio)

	It should connect to the wifi (look for FIXME tags in the code to update with correct wifi)
	This package translates between ros2 and microros


3) Upload dxl_motor_controller_serial (arduino ide) to the openrb150 board and connect the serial to esp32
   
	Debug serial will output the data received by the RB over serial (a.aa,b.bb;c.cc)


4) At this point Twist msgs can be sent from the pc command line:	 (linear x - forward, linear y - claw, angular z - turning)

	$ ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 10.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"


5) Plug in xbox360 controller and check the port number

	$ evtest


6) Update remote_control.py FIXME with the correct event number (eg 26)


7) Run remote_control.py on PC

	$ ros2 run servo_control_py remote_control


8) View the Twist msgs being sent

	ros2 topic echo /cmd_vel

	
