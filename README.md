# iXCOM-ROS2-driver   _- C++ Implementation -_

**_ixcom_driver_lc_** is a _ROS2_ node developed in _C++_. It establishes a connection to an _iNAT_, activates _ixcom logs_, receives the data and publishes them using topics.<br><br>

The default namespace used by the node is `ixcom`. To change this namespace, the top line of the configuration file must be modified:<br>

```console
/ixcom/ixcom_driver_lifecycle_node:
```
becomes
```console
/your_ns/ixcom_driver_lifecycle_node:
```

A modified namespace can also be passed as argument to the launch file with `namespace:=your_ns` which must fit the namespace definition in the configuration file. Also, a different configuration file can be passed as argument with `publisher_config_file:='/your/path/to/file.yml'`. So, if multiple configuration files are available, multiple nodes can be launched using different namespaces and configuration files, which these namespaces are defined in, as arguments.<br>

Example:
```console
ros2 launch ixcom_driver_lc autostart.launch.py namespace:=your_ns
```

If no launch file is used, a subscriber must be launched with `--ros-args -r __ns:=/your_ns` as argument.<br>

Example:
```console
ros2 run your_node sub --ros-args -r __ns:=/your_ns
```



## General Information

While running, the _ixcom_driver_lc_ will appear in the _ROS2_ node list.

```console
~$ ros2 node list
/ixcom/ixcom_driver_lifecycle_node
```

The node info shows the implemented features (not configured topics are not visible in this view, see configuration section for details).

```console
~$ ros2 node info /ixcom/ixcom_driver_lifecycle_node
/ixcom/ixcom_driver_lifecycle_node
  Publishers:
    /ixcom/Imu: sensor_msgs/msg/Imu
    /ixcom/MagneticField: sensor_msgs/msg/MagneticField
    /ixcom/NavSatFix_GNSS: sensor_msgs/msg/NavSatFix
    /ixcom/NavSatFix_INS: sensor_msgs/msg/NavSatFix
    /ixcom/NavSatStatus: sensor_msgs/msg/NavSatStatus
    /ixcom/Odometry: nav_msgs/msg/Odometry
    /ixcom/PoseWithCovarianceStamped: geometry_msgs/msg/PoseWithCovarianceStamped
    /ixcom/TimeReference: sensor_msgs/msg/TimeReference
    /ixcom/TwistStamped: geometry_msgs/msg/TwistStamped
    /ixcom/tf_static: tf2_msgs/msg/TFMessage
  Service Servers:
    /ixcom/ext_heading: ixcom_interfaces/srv/ExtAidHdg
    /ixcom/ext_height: ixcom_interfaces/srv/ExtAidHeight
    /ixcom/ext_position_ecef: ixcom_interfaces/srv/ExtAidPosEcef
    /ixcom/ext_position_llh: ixcom_interfaces/srv/ExtAidPosLlh
    /ixcom/ext_position_mgrs: ixcom_interfaces/srv/ExtAidPosMgrs
    /ixcom/ext_position_utm: ixcom_interfaces/srv/ExtAidPosUtm
    /ixcom/ext_velocity: ixcom_interfaces/srv/ExtAidVel
    /ixcom/ext_velocity_body: ixcom_interfaces/srv/ExtAidVelBody

    [...]
```

## Build

```console
git clone git@github.com:imar-navigation/ROS2_iNAT.git ixcom-ros2-driver
cd ixcom-ros2-driver
colcon build
or
colcon build --packages-select ixcom_driver_lc   (which builds only the specified packages, ixcom_driver_lc in this case)
```

## Configuration

The _ixcom_driver_lc_ configuration is located in `src/ixcom_driver_lc/params/publisher_config.yml`. If changes have been done in this file,
the node has to be rebuilt since this file will be copied into the install directory where it will be used from. If the install directory
already exists, the configuration file can be modified directly in the directory `install/ixcom_driver_lc/share/ixcom_driver_lc/params/publisher_config.yml`.
_NOTE_: The configuration file in the install directory will be overwritten once a rebuild has been performed.
The configuration file has currently the following structure.


```yaml
/ixcom/ixcom_driver_lifecycle_node:     # used by ixcom_driver_lifecycle_node in the namespace ixcom
# /**:                                  # one fits all
  ros__parameters:
    ip:
      address: 192.168.1.30
      port: 3000
    serial:
      ignore: true
      port: 1
      baud: 115200
      enable: true
    timestamp_mode: GPS
    imudata_mode: IMURAW
    qos: SYSTEMDEFAULTS
    leap_seconds: 18
    manual_local_tangential_plane:
      enable: false
      lon: 0.0
      lat: 0.0
      alt: 0.0
    magnetometer_scale_factor: 1.0000000116860974e-07
    topics:
      Imu:
        frequency_hz: 100
        remap_to: ""
      NavSatStatus:
        frequency_hz: 0
        remap_to: ""
      NavSatFix_GNSS:
        frequency_hz: 0
        remap_to: "NavSatFix"
      NavSatFix_INS:
        frequency_hz: 10
        remap_to: ""
      TimeReference:
        frequency_hz: 0
        remap_to: ""
      MagneticField:
        frequency_hz: 0
        remap_to: ""
      Odometry:
        frame_id: "enu"
        frequency_hz: 250
        remap_to: ""
      PoseWithCovarianceStamped:
        frequency_hz: 0
        remap_to: ""
      TwistStamped:
        frequency_hz: 0
        remap_to: ""

```
- The first line names the node (`/ixcom_driver_lifecycle_node`) that uses this configuration and the namespace (/ixcom) this configuration is valid in.
- `ip`  
  contains the connection data of an _iNAT_
- `serial`  
  the serial interface of the connected _iNAT_ can be configured with these data which can be skipped if `ignore` is set to `true`
- `timestamp_mode`  
  contains the timestamp mode that can be the ROS System Time (value: `ROS`) or the _iNAT_ GPS Time (value: `GPS`)
- `imudata_mode`  
  this parameter defines the kind of inertial data (valid values: `IMURAW` (calibrated IMU data), `IMUCORR` (calibrated IMU data, additionally corrected for bias, scale factor and earth rate), `IMUCOMP` (Calibrated IMU data, corrected for bias and scale factor, additionally compensated for earth rate and gravity))
- `qos`  
  quality of service configuration corresponding to the _ROS2_ QoS profiles (valid values: `SYSTEMDEFAULTS`, `SENSORDATA`, `ACTIONSTATUS`, `PARAMETEREVENTS`, `PARAMETERS`, `SERVICES`)
- `leap_seconds`  
  set current leap seconds here if GNSS is not available or set to 0 otherwise
- `topics`  
  activate topics using a value `> 0` for `frequency_hz` and an optionally different topic name as a value for `remap_to`
- the `frame_id` of the topic `Odometry` is configurable

## Run

- Change to the directory where the _ixcom_driver_lc_ is located:
  ```console
  cd ixcom-ros2-driver
  ```
  
- Source and Run Publisher:
  ```console
  . install/setup.bash
  ros2 launch ixcom_driver_lc autostart.launch.py
  ```

- Run Subscriber (optional; this is just an example that shows some topic contents):
  ```console
  ros2 run ixcom_driver sub
  ```
  ```console
  ros2 run ixcom_driver sub -h
  usage: sub [-h] [-f] [-ns NAMESPACE]
  ROS2 iXCOM Driver
  options:
    -h, --help            show this help message and exit
    -f, --measure_frequency
                          measure frequencies of incoming topics
    -ns NAMESPACE, --namespace NAMESPACE
                          node namspace
  ```

## Publishers 

Publishers with the following topics using standard _ROS2_ messages are implemented:  
  `/ixcom/Imu:                         sensor_msgs/msg/Imu`  
  `/ixcom/MagneticField:               sensor_msgs/msg/MagneticField`  
  `/ixcom/NavSatFix_GNSS:              sensor_msgs/msg/NavSatFix`  
  `/ixcom/NavSatFix_INS:               sensor_msgs/msg/NavSatFix`  
  `/ixcom/NavSatStatus:                sensor_msgs/msg/NavSatStatus`  
  `/ixcom/PoseWithCovarianceStamped    geometry_msgs.msg/PoseWithCovarianceStamped`  
  `/ixcom/TwistStamped                 geometry_msgs.msg/TwistStamped`  
  `/ixcom/Odometry:                    nav_msgs/msg/Odometry`  
  `/ixcom/TimeReference:               sensor_msgs/msg/TimeReference`

## Service Servers

The following Service Servers are implemented:  
  `/ixcom/ext_heading:                 ixcom_interfaces/srv/ExtAidHdg`  
  `/ixcom/ext_height:                  ixcom_interfaces/srv/ExtAidHeight`  
  `/ixcom/ext_position_ecef:           ixcom_interfaces/srv/ExtAidPosEcef`  
  `/ixcom/ext_position_llh:            ixcom_interfaces/srv/ExtAidPosLlh`  
  `/ixcom/ext_position_mgrs:           ixcom_interfaces/srv/ExtAidPosMgrs`  
  `/ixcom/ext_position_utm:            ixcom_interfaces/srv/ExtAidPosUtm`  
  `/ixcom/ext_velocity:                ixcom_interfaces/srv/ExtAidVel`  
  `/ixcom/ext_velocity_body:           ixcom_interfaces/srv/ExtAidVelBody`
  
- The interface of the Service Server `/ixcom/ext_heading`:  
  ```console
  ~$ ros2 interface show ixcom_interfaces/srv/ExtAidHdg  
  ```
  ```console
  float64 time_stamp  
  uint16 time_mode  
  float64 heading  
  float64 heading_stddev  
  - - -  
  bool success
  ```
  
  `time_stamp     :` Time at which the measurement was valid in _s_  
  `time_mode      :` Timestamp mode: 0 = Absolute GPS timestamp OR 1 = Latency  
  `heading        :` Heading in _rad_  
  `heading_stddev :` Heading standard deviation in _rad_  
  
  The following command can be used to send a request to this Service Server (_NOTE_: uses default values):  
  ```console
  ~$ ros2 service call /ixcom/ext_heading ixcom_interfaces/srv/ExtAidHdg
  ```
  ```console
  requester: making request: ixcom_interfaces.srv.ExtAidHdg_Request(time_stamp=0.0, time_mode=0, heading=0.0, heading_stddev=0.0)  
  
  response:  
  ixcom_interfaces.srv.ExtAidHdg_Response(success=False)
  ```

- The interface of the Service Server `/ixcom/ext_height`:  
  ```console
  ~$ ros2 interface show ixcom_interfaces/srv/ExtAidHeight  
  ```
  ```console
  float64 time_stamp
  uint16 time_mode
  float64 height
  float64 height_stddev
  - - -
  bool success
  ```
  
  `time_stamp     :` Time at which the measurement was valid in _s_  
  `time_mode      :` Timestamp mode: 0 = Absolute GPS timestamp OR 1 = Latency  
  `height         :` Height in _m_  
  `height_stddev  :` Height standard deviation in _m_  
  
  The following command can be used to send a request to this Service Server (_NOTE_: uses default values):  
  ```console
  ~$ ros2 service call /ixcom/ext_height ixcom_interfaces/srv/ExtAidHeight
  ```
  ```console
  requester: making request: ixcom_interfaces.srv.ExtAidHeight_Request(time_stamp=0.0, time_mode=0, height=0.0, height_stddev=0.0)
  
  response:  
  ixcom_interfaces.srv.ExtAidHeight_Response(success=False)
  ```
  
- The interface of the Service Server `/ixcom/ext_position_ecef`:  
  ```console
  ~$ ros2 interface show ixcom_interfaces/srv/ExtAidPosEcef  
  ```
  ```console
  float64 time_stamp
  uint16 time_mode
  float64[3] position
  float64[3] position_stddev
  float64[3] lever_arm
  float64[3] lever_arm_stddev
  - - -
  bool success
  ```
  
  `time_stamp        :` Time at which the measurement was valid in _s_  
  `time_mode         :` Timestamp mode: 0 = Absolute GPS timestamp OR 1 = Latency  
  `position          :` Position in ECEF frame in _m_  
  `position_stddev   :` Standard deviation of the position in _m_  
  `lever_arm         :` Lever arm in x,y,z direction in _m_  
  `lever_arm_stddev  :` Lever arm standard deviation in x,y,z direction in _m_  
  
  The following command can be used to send a request to this Service Server (_NOTE_: uses default values):  
  ```console
  ~$ ros2 service call /ixcom/ext_position_ecef ixcom_interfaces/srv/ExtAidPosEcef
  ```
  ```console
  requester: making request: ixcom_interfaces.srv.ExtAidPosEcef_Request(time_stamp=0.0, time_mode=0, position=array([0., 0., 0.]), position_stddev=array([0., 0., 0.]), lever_arm=array([0., 0., 0.]), lever_arm_stddev=array([0., 0., 0.]))
  
  response:  
  ixcom_interfaces.srv.ExtAidPosEcef_Response(success=False)
  ```

- The interface of the Service Server `/ixcom/ext_position_llh`:  
  ```console
  ~$ ros2 interface show ixcom_interfaces/srv/ExtAidPosLlh  
  ```
  ```console
  float64 time_stamp
  uint16 time_mode
  float64[3] position
  float64[3] position_stddev
  float64[3] lever_arm
  float64[3] lever_arm_stddev
  uint32 enable_msl_altitude
  - - -
  bool success
  ```
  
  `time_stamp        :` Time at which the measurement was valid in _s_  
  `time_mode         :` Timestamp mode: 0 = Absolute GPS timestamp OR 1 = Latency  
  `position          :` Longitude, latitude, altitude in _rad_ and _m_  
  `position_stddev   :` Standard deviation in _m_ (longitude, latitude, altitude)  
  `lever_arm         :` Lever arm in _m_  
  `lever_arm_stddev  :` Lever arm standard deviation in _m_  
  
  The following command can be used to send a request to this Service Server (_NOTE_: uses default values):  
  ```console
  ~$ ros2 service call /ixcom/ext_position_llh ixcom_interfaces/srv/ExtAidPosLlh
  ```
  ```console
  requester: making request: ixcom_interfaces.srv.ExtAidPosLlh_Request(time_stamp=0.0, time_mode=0, position=array([0., 0., 0.]), position_stddev=array([0., 0., 0.]), lever_arm=array([0., 0., 0.]), lever_arm_stddev=array([0., 0., 0.]), enable_msl_altitude=0)
  
  response:  
  ixcom_interfaces.srv.ExtAidPosLlh_Response(success=False)
  ```

- The interface of the Service Server `/ixcom/ext_position_mgrs`:  
  ```console
  ~$ ros2 interface show ixcom_interfaces/srv/ExtAidPosMgrs  
  ```
  ```console
  float64 time_stamp
  uint16 time_mode
  string mgrs
  float64 altitude
  float64[3] position_stddev
  float64[3] lever_arm
  float64[3] lever_arm_stddev
  - - -
  bool success
  ```
  
  `time_stamp        :` Time at which the measurement was valid in _s_  
  `time_mode         :` Timestamp mode: 0 = Absolute GPS timestamp OR 1 = Latency  
  `mgrs              :` MGRS string (e.g. 32U LV 66136 59531)  
  `altitude          :` Altitude in _m_    
  `position_stddev   :` Position standard deviation in _m_ (easting, northing, altitude)  
  `lever_arm         :` Lever arm in x,y,z direction _m_  
  `lever_arm_stddev  :` Lever arm standard deviation in x,y,z direction in _m_  
  
  The following command can be used to send a request to this Service Server (_NOTE_: uses default values):  
  ```console
  ~$ ros2 service call /ixcom/ext_position_mgrs ixcom_interfaces/srv/ExtAidPosMgrs
  ```
  ```console
  requester: making request: ixcom_interfaces.srv.ExtAidPosMgrs_Request(time_stamp=0.0, time_mode=0, mgrs='', altitude=0.0, position_stddev=array([0., 0., 0.]), lever_arm=array([0., 0., 0.]), lever_arm_stddev=array([0., 0., 0.]))
  
  response:  
  ixcom_interfaces.srv.ExtAidPosMgrs_Response(success=False)
  ```

- The interface of the Service Server `/ixcom/ext_position_utm`:  
  ```console
  ~$ ros2 interface show ixcom_interfaces/srv/ExtAidPosUtm  
  ```
  ```console
  float64 time_stamp
  uint16 time_mode
  uint32 zone
  uint8 north_hp
  float64 easting
  float64 northing
  float64 altitude
  float64[3] position_stddev
  float64[3] lever_arm
  float64[3] lever_arm_stddev
  - - -
  bool success
  ```
  
  `time_stamp        :` Time at which the measurement was valid in _s_  
  `time_mode         :` Timestamp mode: 0 = Absolute GPS timestamp OR 1 = Latency  
  `zone              :` UTM zone (0 = UPS)  
  `north_hp          :` Hemisphere (true = north, false = south)  
  `easting           :` East component in _m_  
  `northing          :` North component in _m_  
  `altitude          :` Altitude in _m_  
  `position_stddev   :` Position standard deviation in _m_ (easting, northing, altitude)  
  `lever_arm         :` Lever arm in x,y,z direction _m_  
  `lever_arm_stddev  :` Lever arm standard deviation in x,y,z direction in _m_  
  
  The following command can be used to send a request to this Service Server (_NOTE_: uses default values):  
  ```console
  ~$ ros2 service call /ixcom/ext_position_utm ixcom_interfaces/srv/ExtAidPosUtm
  ```
  ```console
  requester: making request: ixcom_interfaces.srv.ExtAidPosUtm_Request(time_stamp=0.0, time_mode=0, zone=0, north_hp=0, easting=0.0, northing=0.0, altitude=0.0, position_stddev=array([0., 0., 0.]), lever_arm=array([0., 0., 0.]), lever_arm_stddev=array([0., 0., 0.]))
  
  response:  
  ixcom_interfaces.srv.ExtAidPosUtm_Response(success=False)
  ```

- The interface of the Service Server `/ixcom/ext_velocity`:  
  ```console
  ~$ ros2 interface show ixcom_interfaces/srv/ExtAidVel  
  ```
  ```console
  float64 time_stamp
  uint16 time_mode
  float64[3] velocity
  float64[3] velocity_stddev
  - - -
  bool success
  ```
  
  `time_stamp       :` Time at which the measurement was valid in _s_  
  `time_mode        :` Timestamp mode: 0 = Absolute GPS timestamp OR 1 = Latency  
  `velocity         :` Velocity east, north, dowm in _m/s_  
  `velocity_stddev  :` Velocity standard deviation in _m/s_  
  
  The following command can be used to send a request to this Service Server (_NOTE_: uses default values):  
  ```console
  ~$ ros2 service call /ixcom/ext_velocity ixcom_interfaces/srv/ExtAidVel
  ```
  ```console
  requester: making request: ixcom_interfaces.srv.ExtAidVel_Request(time_stamp=0.0, time_mode=0, velocity=array([0., 0., 0.]), velocity_stddev=array([0., 0., 0.]))
  
  response:  
  ixcom_interfaces.srv.ExtAidVel_Response(success=False)
  ```

- The interface of the Service Server `/ixcom/ext_velocity_body`:  
  ```console
  ~$ ros2 interface show ixcom_interfaces/srv/ExtAidVelBody  
  ```
  ```console
  float64 time_stamp
  uint16 time_mode
  float64[3] velocity
  float64[3] velocity_stddev
  float64[3] lever_arm
  float64[3] lever_arm_stddev
  - - -
  bool success
  ```
  
  `time_stamp        :` Time at which the measurement was valid in _s_  
  `time_mode         :` Timestamp mode: 0 = Absolute GPS timestamp OR 1 = Latency  
  `velocity          :` Velocity in body x,y,z direction in _m/s_  
  `velocity_stddev   :` Velocity standard deviation in _m/s_  
  `lever_arm         :` Lever arm in x,y,z direction _m_  
  `lever_arm_stddev  :` Lever arm standard deviation in x,y,z direction in _m_  
  
  The following command can be used to send a request to this Service Server (_NOTE_: uses default values):  
  ```console
  ~$ ros2 service call /ixcom/ext_velocity_body ixcom_interfaces/srv/ExtAidVelBody
  ```
  ```console
  requester: making request: ixcom_interfaces.srv.ExtAidVelBody_Request(time_stamp=0.0, time_mode=0, velocity=array([0., 0., 0.]), velocity_stddev=array([0., 0., 0.]), lever_arm=array([0., 0., 0.]), lever_arm_stddev=array([0., 0., 0.]))
  
  response:  
  ixcom_interfaces.srv.ExtAidVelBody_Response(success=False)
  ```

_**NOTE:**_ The data sent to the node will be forwarded to the _iNAT_. The _iNAT_ must send a response back to the node within a second. Otherwise, the requester receives `success=False` due to the timeout. 
    


