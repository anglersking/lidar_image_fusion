# Robosense128
The driver of vlp-128

# Depenpent 
- hamap_msgs: https://gitlab.wanshannnt.works/driver/hamap_msgs
- param_server: https://gitlab.wanshannnt.works/driver/param_server
- gps_imu_driver(motion compensation): https://gitlab.wanshannnt.works/driver/gps_imu_driver
- sensor_conversion_tools(motion compensation): https://gitlab.wanshannnt.works/driver/sensor_conversion_tools

# Usage
```bash
$ roslaunch robosense_pointcloud wanshannnt-128.launch
```

# Message
- "robosense_multi_scans": 10hz 7M/s
- "robosense_points": 10hz 70M/s
- "robosense_packet": 6250hz 8M/s
- "robosense_points_no_compensate": 10hz 70M/s

# Feature
- Packets rate: 6250
- Cut angle: -180 degree
- If the phase is locked, the whole 100 milliseconds rotates to the front. And the timestamp of "robosense_points" is recorded to that moment.
- Each "robosense_points" contains 1875×128 points (maybe 1872×128 sometimes)
- motion compensation: "/robosense/using_compensation" need tf between ‘enu’ and ‘car’ 

# Time (using gps)
- Packets: gps
- robosense_multi_scans: gps
- robosense_points_no_compensate: gps
- robosense_points: gps

# Exceptions
- Back in time -> Close motion compensation 
- Large time difference between system and GPS. -> TODO
- Phase locking failure -> TODO
- No "tf" message (car to enu) received -> Close motion compensation
- No "tf" message (car to lidar) received -> No output 

# Data type
```bash
POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_pointcloud::PointXYZIT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (double, timestamp, timestamp)
                                  (uint8_t, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (float, azimuth, azimuth))
```

# Bag to pcd
```bash
$ rosrun pcl_ros bag_to_pcd <input_file.bag> <topic> <output_directory>
$ rosrun pcl_ros pointcloud_to_pcd input:=/robosense_points
```

# TF tool 
```bash
$ rosrun rqt_tf_tree rqt_tf_tree
$ rosrun tf tf_echo /frameA /freameB
```


