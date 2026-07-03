# tm_imu
This package implements a ROS2 node of TransducerM AHRS/IMU from SYD Dynamics. 

# Acknowledge
New updates: The package converts the original ROS1 node to work in ROS2. The function `onSerialRX` that reads the values from the sensor is called in a timer callback function. Special thanks to Antonis and fellow colleagues from ROICO Solutions ApS for providing value contributions for the conversion.

# Dependencies
All the required library are contained inside the `src/` folder and the 'external/' folder.
The 'external/EasyProfile' folder includes the TransducerM communication library from SYD Dynamics.
The 'external/seriallib'   folder includes the [seriallib](https://github.com/imabot2/serialib?tab=MIT-1-ov-file), version 1.2 (from april 2011).

# Messages
This package publishes message types:
- sensor_msgs/Imu :            The standard Imu message from ROS
- sensor_msgs/MagneticField:   The standard Imu mag message from ROS

# Topics
This package publishes three topics:
-imu_data     : Standard ROS imu data package, including quaternion data, acceleration data, and gyroscope data
-imu_data_mag : Standard ROS magneticfield data package
-imu_data_rpy : This topic shares the same data structure as the standard ROS imu_data_mag topic, while it is used for publishing Roll, Pitch and Yaw output from TransducerM. The definition of data fields shown as below:
                imu_data_raw_rpy.magnetic_field.x =  roll                    
                imu_data_raw_rpy.magnetic_field.y =  pitch
                imu_data_raw_rpy.magnetic_field.z =  yaw
                
# How to use it
1. Create a work folder:
       $mkdir -p ~/imu_ws/src
2. Copy all source code of this ROS2 package into the src folder created above
3. Setup parameters in ./config/params.yaml
       For example: change imu_port value to your actual serial port name and setup baudrate
4. Compile 
       $cd .. 
           Then if you run $pwd the path should be similar as below:
           /home/your_user_name/imu_ws
       $colcon build 
       $cd ~
       $source ~/imu_ws/install/setup.sh 
       $ros2 launch tm_imu imu.launch.py
5. View topics
       $ros2 topic list 
       $ros2 topic echo /imu_data
6. View topics in RVIZ
       $ros2 run rviz2 rviz2
       Add tf display, Set fixed axis to 'world'.
       Please also refer to the screenshot of an example setup ./screenshot_rviz2.png
   
# Parameters
./config params.yaml includes the following parameters:
- `imu_baudrate` default value `115200` :        Set the communication speed between the IMU and the computer.
- `imu_port`     default value `"/dev/ttyUSB0"`: Specify the serial port where the sensor is connected.
- `imu_frame_id` default value `"imu"`  :        Specify the frame id of the sensor. 
- `timer_period` default value `5`      :        Set the timer period reading TransducerM serial port data in ms (Please rebuild this package after changing this value for it to take effect).
- `publish_tf`   default value `true`   :        Enable/disable TF broadcast of world→imu_link transform. Set to `false` when using `robot_localization` to avoid TF tree conflicts.


# robot_localization Compatibility
This node is compatible with the `robot_localization` package (EKF/UKF sensor fusion). 

When using with `robot_localization`:
1. Set `publish_tf: false` in `params.yaml` to prevent TF tree conflicts (`robot_localization` manages its own `odom → base_link` transform).
2. Point your EKF config to the `/imu_data` topic. The message contains:
   - `orientation` (quaternion) — derived from onboard RPY via `tf2::Quaternion::setRPY()`
   - `angular_velocity` (rad/s)
   - `linear_acceleration` (m/s²)
3. Covariance matrices are proper 3×3 diagonal matrices (off-diagonal = 0) as required by `robot_localization`.
4. Ensure `imu_frame_id` matches the IMU link name in your URDF.


If you find any issue please leave a message at [our website](https://www.syd-dynamics.com/contact-us/)

Last updated on Aug 1, 2024


# Changelog

## July 1, 2026

### Bug Fix: Orientation not working — no quaternion data from sensor
**Problem:** RViz showed the acceleration arrow but the IMU box did not rotate. 
The `orientation` field in `sensor_msgs/Imu` was always `(0,0,0,0)` (invalid quaternion).

**Root Cause:** The TM151 sensor (in its default configuration) sends **separate** `EP_CMD_Raw_GYRO_ACC_MAG_` and `EP_CMD_RPY_` packets — it does **not** send `EP_CMD_COMBO_` or `EP_CMD_Q_S1_E_` quaternion packets. The original code only populated `imu_data_msg.orientation` inside the COMBO and Q_S1_E handlers, which never fired. The orientation remained at its default zero value.

**Fix:** Added RPY-to-quaternion conversion in the `EP_CMD_RPY_` handler using `tf2::Quaternion::setRPY()`. The sensor's roll/pitch/yaw (degrees) are converted to radians and then to a normalized quaternion that populates `imu_data_msg.orientation`.

### Bug Fix: Quaternion field mapping in COMBO handler (q1–q4 swapped)
**Problem:** In the `EP_CMD_COMBO_` handler, quaternion fields were mapped as `q1→x, q2→y, q3→z, q4→w`.

**Root Cause:** SYD Dynamics documents `q1=w, q2=x, q3=y, q4=z` (w,x,y,z format) in `EasyObjectDictionary.h`, but the code assigned them to the wrong `orientation` fields.

**Fix:** Corrected the mapping to `q1→w, q2→x, q3→y, q4→z`.

### Bug Fix: Acceleration units wrong (g instead of m/s²)
**Problem:** `sensor_msgs/Imu` specifies `linear_acceleration` in m/s², but the raw data from the sensor is in g (1g = 9.80665 m/s²).

**Fix:** Added `* 9.80665` conversion in both the `EP_CMD_Raw_GYRO_ACC_MAG_` and `EP_CMD_COMBO_` handlers.

### Bug Fix: Covariance matrices had non-zero off-diagonal elements
**Problem:** All 9 elements of each covariance matrix were set to `0.1`, implying false cross-correlations between axes. `robot_localization` expects proper diagonal covariance.

**Fix:** Changed to proper diagonal covariance matrices — off-diagonal elements are `0.0`, diagonal elements use reasonable variance estimates (orientation: 0.0025, gyro: 0.0003, accel: 0.01).

### Feature: Configurable TF broadcast (`publish_tf` parameter)
**Problem:** The node always broadcast a `world → imu_link` transform, which conflicts with `robot_localization`'s own TF output.

**Fix:** Added `publish_tf` parameter (default: `true`). Set to `false` in `params.yaml` when using `robot_localization`.

### Initialization: Identity quaternion
**Fix:** Orientation is now initialized to a valid identity quaternion `(w=1, x=0, y=0, z=0)` instead of all-zeros `(0,0,0,0)` which is not a valid quaternion.
