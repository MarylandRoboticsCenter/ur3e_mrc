# intermediary package for labs in ENME480

main steps to run 
1. turn ON UR3e
2. run the official UR ROS2 driver
3. run the external control on UR3e pendant
4. run this package code: ros2 launch ur3e_mrc ur3e_enme480.launch
5. run python code that publishes to ur3/command


## test command
ros2 topic pub --once /ur3/command ur3e_mrc/msg/CommandUR3e "destination: [0, -1.57, -1.57, 0, 0, 0]
v: 1.0
a: 1.0
io_0: false" 
