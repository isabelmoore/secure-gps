#MKZ CONTROLLER 
MKZ ROS2 communication as of Spring 2023, Translation of MKZ waypoint controller from https://github.com/Kchour/MKZ_controller/tree/controllers/vehicle_controllers

INSTRUCTIONS FOR RUNNING SIMULATOR: 
1) source ws 
2) source Gazebo (. /usr/share/gazebo-11/setup.bash), run Gazebo sim (ros2 launch dbw_gazebo_mkz Gazebo_mkz_initialize_launch.xml)
3) ros2 run vehicle_controllers_mkz odompubtest.py 
4) Run controllers>
- To test waypoint generation, run ros2 launch vehicle_control_mkz controllaunch.xml IS_CP:=TRUE and set file to /odom_waypoints.dat (in launch file)
- To run controller, run ros2 launch vehicle_control_mkz controllaunch.xml IS_CP:= FALSE DESIRED_SPEED:='5.0' (depends on speed you want)

INSTRUCTIONS FOR RUNNING REAL MKZ: 
1) source controller ws 
2) STARTING MTI ROS2 FILTER: in another terminal, source MTI ws (It will start with MTI)
3) ros2 launch bluespace_ai_xsens_mti_driver xsens_mti_node.launch.py 
4) run dbw (ros2 launch dbw_ford_can dbw.launch.xml)
6) For time being, to start vehicle in dbw mode, run ros2 topic pub /vehicle/enable std_msgs/msg/Empty '{}'  (TROUBLESHOOT WHY UP/DOWN CONTROLS NOT STARTING DBW ON MKZ) 
7) In controller ws, Run controllers>
- To test waypoint generation, run ros2 launch vehicle_control_mkz controllaunch.xml IS_CP:=TRUE and set file to /odom_waypoints.dat (in launch file)
- To run controller, run ros2 launch vehicle_control_mkz controllaunch.xml IS_CP:= FALSE DESIRED_SPEED:='5.0' (depends on speed you want)

To verify waypoint generation/following: graph is availible when running ros2 run vehicle_control_mkz Waypoint_plotter.py 


To switch gears in dbw mode, run ros2 topic pub /vehicle/gear_cmd dbw_ford_msgs/msg/GearCmd 'header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: '\'''\''
cmd:
  gear: 0
clear: false' 

gear: 0 = none 
gear: 1 = park
gear: 2 = reverse
gear: 3 = neutral
gear: 4 = drive
gear: 5 = low 


