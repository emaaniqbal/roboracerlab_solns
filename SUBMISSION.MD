ROS 2 CMD TEST:

1: ros2 topic info drive (OR: ros2 topic info /drive)
Type: ackermann_msgs/msg/AckermannDriveStamped
Publisher count: 1
Subscription count: 1

2: ros2 topic echo drive (OR: ros2 topic echo   
/drive)
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
drive:
  steering_angle: 1.0
  steering_angle_velocity: 0.0
  speed: 2.0
  acceleration: 0.0
  jerk: 0.0
---
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
drive:
  steering_angle: 1.0
  steering_angle_velocity: 0.0
  speed: 2.0
  acceleration: 0.0
  jerk: 0.0

3: ros2 node list
/relay
/talker

4: ros2 node info talker (OR: ros2 node info /talker)
/talker
  Subscribers:

  Publishers:
    /drive: ackermann_msgs/msg/AckermannDriveStamped
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /talker/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /talker/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /talker/get_parameters: rcl_interfaces/srv/GetParameters
    /talker/list_parameters: rcl_interfaces/srv/ListParameters
    /talker/set_parameters: rcl_interfaces/srv/SetParameters
    /talker/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:

5: ros2 node info relay  (OR: ros2 node info /relay )
/relay
  Subscribers:
    /drive: ackermann_msgs/msg/AckermannDriveStamped
  Publishers:
    /drive_relay: ackermann_msgs/msg/AckermannDriveStamped
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /relay/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /relay/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /relay/get_parameters: rcl_interfaces/srv/GetParameters
    /relay/list_parameters: rcl_interfaces/srv/ListParameters
    /relay/set_parameters: rcl_interfaces/srv/SetParameters
    /relay/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
  
Bonus...
 ros2 topic echo /drive_relay
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
drive:
  steering_angle: 3.0
  steering_angle_velocity: 0.0
  speed: 6.0
  acceleration: 0.0
  jerk: 0.0
---