# DJI MSDK ROS 2 Driver

ROS 2 Driver for DJI drones using DJI Mobile SDK V5

## Roadmap

- [x] Piloting
  - [x] Taking off
  - [x] Landing
  - [x] Movement
- [x] Positioning
  - [x] Motive + OptiTrack
  - [x] RTK-GNSS
- [x] Battery
- [x] Gimbal
  - [x] Monitor
  - [x] Control
- [x] Media
  - ~~[x] Decoding~~
  - [x] Publishing
     
## Dependencies
[ffmpeg_image_transport](https://github.com/ros-misc-utilities/ffmpeg_image_transport)

## ROS messages

> Drone ids are distinguished by namespaces like `/mavic_0`

### Takeoff

#### Control

Topic name: `takeoff`  
Message type: `std_msgs/Empty`  
Command:  

```bash
ros2 topic pub --once /mavic_0/takeoff std_msgs/msg/Empty
```

### Land

#### Control

Topic name: `land`  
Message type: `std_msgs/Empty`  
Command:  

```bash
ros2 topic pub --once /mavic_0/land std_msgs/msg/Empty
```

### Emergency

#### Control

This command stops drone and rejects `cmd_vel` topic.

Topic name: `emergency`  
Message type: `std_msgs/Empty`  
Command:  

```bash
ros2 topic pub --once /mavic_0/emergency std_msgs/msg/Empty
```

### Piloting

#### Control

Topic name: `cmd_vel`  
Message type: `geometry_msgs/Twist`

```yml
linear:
  x: pitch -> forward/backward movement velocity (m/s)
  y: roll -> left/right movement velocity (m/s)
  z: vertical throttle -> vertical movement velocity (m/s)
angular:
  x: nan
  y: nan
  z: yaw -> horizontal rotation velocity (rad/s)
---
```

Command:  

```bash
ros2 topic pub --once /mavic_0/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Positioning

#### Motive + OptiTrack or RTK-GNSS

Topic name: `pos`  
Message type: `geometry_msgs/Twist`

```yml
linear:
  x: 
  y: 
  z: 
angular:
  x: alpha
  y: beta
  z: gamma
---
```

Command:  

```bash
ros2 topic echo /mavic_0/pos
```

### RTK-GNSS

#### Set origin location
>
> [!NOTE]
> This topic is global

Topic name: `/setOriginLocation`
Message type: `stds_msgs/Float64MultiArray`

```yml
data: [latitude, longitude, altitude]
---
```

Command:

```bash
ros2 topic pub --once /setOriginLocation /std_msgs/msg/Float64MultiArray "data: [35.6024, 139.6836922, 0.0]"
```

#### Monitor

Topic name: `GPS`
Message type: `sensor_msgs/NavSatFix`
Command:

```bash
ros2 topic echo /mavic_0/GPS
```

### Battery

#### Monitor

Topic name: `battery_state`  
Message type: `sensor_msgs/BatteryState`  
Command:  

```bash
ros2 topic echo /mavic_0/battery_state
```

### Gimbal

#### Monitor

Topic name: `gimbal_state`  
Message type: `geometry_msgs/Vector3`  
Command:  

```bash
ros2 topic echo /mavic_0/gimbal_state
```

#### Position control

Topic name: `gimbal_cmd`  
Message type: `geometry_msgs/Vector3`  
Command:

```bash
ros2 topic pub --once /mavic_0/gimbal_cmd geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 0.0}" 
```

#### Velocity control

Topic name: `gimbal_vel_cmd`  
Message type: `geometry_msgs/Vector3`  
Command:

```bash
ros2 topic pub --once /mavic_0/gimbal_vel_cmd geometry_msgs/msg/Vector3 "{x: 0.1, y: 0.1, z: 0.1}" 
```

### Velocity

#### Monitor

Topic name: `vel`  
Message type: `geometry_msgs/Vector3`  
Command:

```bash
ros2 topic echo /mavic_0/vel
```

### Motor state

#### Monitor

Topic name: `motor_state`  
Message type: `std_msgs/Bool`  
Command:

```bash
ros2 topic echo /mavic_0/motor_state
```

### Above Ground Level

#### Monitor

Topic name: `AGL`
Message type: `std_msgs/Int32`
Unit: `dm (10cm)`
Command:

```bash
ros2 topic echo /mavic_0/AGL
```

### Camera

#### Stream

Topic name: `image/ffmpeg`  
Message type: `ffmpeg_image_transport_msgs/FFMPEGPacket`
Command:

```bash
ros2 run image_transport republish ffmpeg in/ffmpeg:=/mavic_0/image/ffmpeg raw out:=/mavic_0/image/raw
ros2 run rqt_image_view rqt_image_view
```

#### Shoot

Topic name: `shoot`
Message type: `std_msgs/Empty`
Command:

```bash
ros2 topic pub --once /mavic_0/shoot std_msgs/msg/Empty
```

#### Record

Topic name: `record`
Message type: `sensor_msgs/Bool`
Command:

```bash
# record start
ros2 topic pub --once /mavic_0/record std_msgs/msg/Bool "{data:true}"

# record stop
ros2 topic pub --once /mavic_0/record std_msgs/msg/Bool "{data:false}"
```

## Setting of Android Studio
https://github.com/dji-sdk/Mobile-SDK-Android-V5/issues/342#issuecomment-2144628530

## Check also

[DJI Mobile SDK V5](https://github.com/dji-sdk/Mobile-SDK-Android-V5)  
[ROS2 Java](https://github.com/ros2-java/ros2_java)
