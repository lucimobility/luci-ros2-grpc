# luci_grpc_interface

## Summary:

This is the primary package used in the SDK. This package also allows for the bidirectional transfer of information between LUCI and the ROS ecosystem. This package's node allows for you to read sensor data as well as send remote joystick commands to the system.

This package points at a LUCI gRPC server and operates on a multi-threaded approach which will grab all specified LUCI sensor streams then pass them through a message type conversion and then publish them on ROS topics.

You are likely to always use this node if using the LUCI SDK.

## Use

To run a basic interface with no IR frames you need to run `ros2 run luci_grpc_interface grpc_interface_node -a <chairs-ip-address>`

To run the node while streaming IR frames add the -f flag followed by the rate you would like.

Example of streaming at 5fps per camera `ros2 run luci_grpc_interface grpc_interface_node -a <chairs-ip-address> -f 5`

## Running Modes

There are 4 ROS services set up for the user. They can be used to set annd remove two drive modes which are Autonomous Remote and Shared Remote. 

  - Autonomous Remote - This mode allows for a fully autonomous control of the chair by publishing a joystick messsage on the `/luci/remote_joystick` topic. The physical joystick wont stream any information and cant be used when this mode is activated. This mode can be set/remove by using `ros2 service call /luci/<set/remove>_auto_remote_input std_srvs/srv/Empty`. **Note** - While setting remote joystick, dont forget to set the input source to 1. If the input source is not set right, LUCI will disregard the command. 
  - Shared Remote - This mode allows for a shared autonomous control of the chair. This mode will allow the user joystick to still stream on `/luci/joystick_position` topic. This allows for a shared autonomy where the input joystick data can be used for processing and new data to control the chair can be published on the `/luci/remote_joystick` topic. This mode can be set/remove by using `ros2 service call /luci/<set/remove>_shared_remote_input std_srvs/srv/Empty`. **Note** - While setting remote joystick, dont forget to set the input source to 5. If the input source is not set right, LUCI will disregard the command. 

<b>Notes:</b>

- The -f flag is optional and if nothing is passed in, the IR frames and the camera info will "stream" at 0 fps. You will not see the ros topics if the flag is not set.
- The best way to see the IR frames is with foxglove. See the main LUCI SDK docs for more information.
- Camera info topic streams camera intrinsics, traslations and rotation and can be checked by performing `ros2 topic echo /luci/<camera_position>_camera_info` where ros is sourced. The `camera_position` parameter can be `left`, `right`, `rear`.
- The maximum frame rate that can be sent is 15fps. This is the rate LUCI cameras naturally stream at.
- All requested frame rates are best effort requests. This means that LUCI will do its best to maintain the FPS but network and other factors may slow the actual observed rate. In addition any rate that is not cleanly divided into 15 will not be guaranteed.
  For example a rate request of 5 will be honored minus any network delay but a request of 7 will result in either 7 or 8 fps.

| Package             | Node                     |
| ------------------- | ------------------------ |
| luci_grpc_interface | luci_grpc_interface_node |

|Implemented | Topics                 | ROS2 Type | Message Type                       | Description                                                                                                                                                                                                        |
| --------------------- | ---------------------- | ---------------------- | ---------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| yes                   | luci/set_auto_remote_input   | service           | std_srvs::srvs::Empty   | Sets the input source to the autonomous remote source for joystick commands                                                                                                                        |
| yes                   | luci/remove_auto_remote_input   | service           | std_srvs::srvs::Empty   | Removes the autonomous remote joystick input source and set it to default                                                                                                                  |
| yes                   | luci/set_shared_remote_input   | service           | std_srvs::srvs::Empty   | Sets the input source to the shared remote source for joystick commands                                                                                                                        |
| yes                   | luci/remove_shared_remote_input   | service           | std_srvs::srvs::Empty   | Removes the shared remote joystick input source and set it to default                                                                                                                  |
| yes                   | luci/remote_joystick   | subscription           | luci_messages::msg::LuciJoystick   | Remote joystick values used to drive the chair (front_back:xxx, left_right: xxx, joystick_zone: y, input_source: z). xxx range: [-100, 100], y range: [1, 8], input_source: [0, 5]   **NOTE** - Publishing to this topic should be faster than 10Hz, or else chair will engage and disengage brakes constantly.                                                                                                                      |
| yes                   | luci/joystick_position | publisher              | luci_messages::msg::LuciJoystick   | Joystick values of the chair (front_back:xxx, left_right: xxx, joystick_zone: y, input_source: z). xxx range: [-100, 100], y range: [1, 8], input_source: [0, 5].                                                                                                                                                                    |
| partially             | luci/odom              | publisher              | nav_msgs::msg::Odometry            | AHRS odom reading                                                                                                                                                                                                  |
| yes                   | luci/imu               | publisher              | luci_messages::msg::LuciImu        | Raw IMU data from the LUCI system                                                                                                                                                                                  |
| yes                   | luci/ultrasonic_points | publisher              | sensor_msgs::msg::PointCloud2      | Ultrasonic pointcloud                                                                                                                                                                                              |
| yes                   | luci/radar_points      | publisher              | sensor_msgs::msg::PointCloud2      | Radar pointcloud                                                                                                                                                                                                   |
| yes                   | luci/camera_points     | publisher              | sensor_msgs::msg::PointCloud2      | Fused and Decimated pointcloud from all the LUCI cameras                                                                                                                        |
| yes                   | luci/collision_points     | publisher              | sensor_msgs::msg::PointCloud2      | Collision points extracted from camera points                                                                                                                        |
| yes                   | /luci/dropoff_points    | publisher              | sensor_msgs::msg::PointCloud2      | Dropoff points extracted from camera points                                                                                                                        |
| yes                   | luci/scaling           | publisher              | luci_messages::msg::LuciScaling    | Scaling percentage of each zone LUCI sees. [0, 1] output for each zone (percentage scaling)                                                                                                                                        |
| yes                   | luci/joystick_scaling  | publisher              | luci_messages::msg::LuciJoystick   | Scaled final Joystick values of the chair including the scaling applied                                                                                                                                                              |
| yes                   | luci/encoder           | publisher              | luci_messages::msg::LuciEncoders   | Raw Encoder data from the LUCI system                                                                                                                                                                              |
| yes                   | luci/chair_profile           | publisher              | std_msgs::msg::Int32   | Streams wheelchair drive profile                                                                                                                                                                                |
| yes                   | luci/speed_setting           | publisher              | std_msgs::msg::Int32   | Streams wheelchair drive speed                                                                                                                                                                                |
| yes             | luci/depth_left_camera    | publisher              | sensor_msgs::msg::Image            | Left camera’s depth image frame                                                                                                                                                                                             |
| yes             | luci/depth_right_camera   | publisher              | sensor_msgs::msg::Image            | Right camera’s depth image frame                                                                                                                                                                                            |
| yes             | luci/depth_rear_camera    | publisher              | sensor_msgs::msg::Image            | Rear camera’s depth image frame                                                                                                                                                                                             |

**NOTE: The Following topics will only be published when -f Flag is set** 

|Implemented | Topics                 | ROS2 Type | Message Type                       | Description                                                                                                                                                                                                        |
| --------------------- | ---------------------- | ---------------------- | ---------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| yes             | luci/ir_left_camera    | publisher              | sensor_msgs::msg::Image            | Left camera’s IR frame                                                                                                                                                                                             |
| yes             | luci/ir_right_camera   | publisher              | sensor_msgs::msg::Image            | Right camera’s IR frame                                                                                                                                                                                            |
| yes             | luci/ir_rear_camera    | publisher              | sensor_msgs::msg::Image            | Rear camera’s IR frame                                                                                                                                                                                             |
| yes             | luci/left_camera_info  | publisher              | luci_messages::msg::LuciCameraInfo | Left camera’s IR frame meta data                                                                                                                                                                                   |
| yes             | luci/right_camera_info | publisher              | luci_messages::msg::LuciCameraInfo | Right camera’s IR frame meta data                                                                                                                                                                                  |
| yes             | luci/rear_camera_info  | publisher              | luci_messages::msg::LuciCameraInfo | Rear camera’s IR frame meta data                                                                                                                         |

### Removed Implementations
|  | Topics                 | ROS2 Type | Message Type                       | Description |
| --------------------- | ---------------------- | ---------------------- | ---------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| removed                   | luci/drive_mode        | subscription           | luci_messages::msg::LuciDriveMode  | Mode of chair for drive controls (USER = user drives with joystick, ENGAGED = remote command drive the chair if user is holding joystick forward, AUTO = remote commands drive chair no matter what user is doing) |

### Future Implementations
|  | Topics                 | ROS2 Type | Message Type                       | Description |
| --------------------- | ---------------------- | ---------------------- | ---------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| coming soon                    | luci/chair_velocity    | publisher              | geometry_msgs::msg::Twist          | Linear and angular velocity of the chair according to onboard AHRS **Note: “linear velocity” will be speed not velocity**|
| coming soon           | luci/all_sensor_points | publisher              | sensor_msgs::msg::PointCloud2      | Full pointcloud (All LUCI sensors)|