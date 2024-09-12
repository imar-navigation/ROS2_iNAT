
# iXCOM-ROS2-driver   _- C++ Implementation -_

The **_ixcom_driver_** is a _ROS2_ node that is developed in _C++_. It establishes a connection to an _iNAT_, activates logs, receives the data and publishes them using topics.

## Dependencies

Install iXCOM Client:
```console
pip install ixcom
```

## General Information

While the _ixcom_driver_ is running it will appear in the _ROS2_ node list.

```console
~$ ros2 node list
/ixcom_driver_lifecycle_node
```

The node info shows the implemented features (not configured topics are not visible in this view, see [Configuration](#configuration-1)).

```console
~$ ros2 node info /ixcom_driver_lifecycle_node
/ixcom_driver_lifecycle_node
  Publishers:
    /Imu: sensor_msgs/msg/Imu
    /MagneticField: sensor_msgs/msg/MagneticField
    /NavSatFix_GNSS: sensor_msgs/msg/NavSatFix
    /NavSatFix_INS: sensor_msgs/msg/NavSatFix
    /NavSatStatus: sensor_msgs/msg/NavSatStatus
    /Odometry: nav_msgs/msg/Odometry
    /PoseWithCovarianceStamped: geometry_msgs/msg/PoseWithCovarianceStamped
    /TimeReference: sensor_msgs/msg/TimeReference
    /TwistStamped: geometry_msgs/msg/TwistStamped
    /tf_static: tf2_msgs/msg/TFMessage
```

## Build

```console
git clone git@gitlab.my.imar.de:automotive/okular/ixcom-ros2-driver.git ixcom-ros2-driver
cd ixcom-ros2-driver
colcon build
or
colcon build --packages-select ixcom_driver_lc   (which builds only the specified packages, ixcom_driver_lc in this case)
```

## Configuration

The _ixcom_driver_ configuration is located in `src/ixcom_driver_lc/params/default.yml`. If changes have been done in this file,
the node has to be rebuilt since this file wil be copied into the install directory where it will be used from. If the install directory
already exists, the configuration file can be modified directly in the directory `install/ixcom_driver_lc/share/ixcom_driver_lc/params/default.yml`.
NOTE: The configuration file in the install directory will be overwritten once a rebuild has been performed.
The configuration file has currently the following structure.


```yaml
ixcom_driver_lifecycle_node:
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
        frequency_hz: 250
        remap_to: ""
      PoseWithCovarianceStamped:
        frequency_hz: 0
        remap_to: ""
      TwistStamped:
        frequency_hz: 0
        remap_to: ""

```
- The first two lines are required for the _ROS2_ environment to know that this node and this configuration file belong togehter.
- `ip`  
  contains the connection data of an _iNAT_
- `serial`  
  the serial interface of the connected _iNAT_ can be configured with these data which can be skipped if `ignore` is set to `true`
- `timestamp_mode`  
  contains the timestamp mode that can be the ROS System Time (value: `ROS`) or the _iNAT_ GPS Time (value: `GPS`)
- `qos`  
  quality of service configuration corresponding to the _ROS2_ QoS profiles (valid values: `SYSTEMDEFAULTS`, `SENSORDATA`, `ACTIONSTATUS`, `PARAMETEREVENTS`, `PARAMETERS`, `SERVICES`)
- `leap_seconds`  
  set current leap seconds here if GNSS is not available or set to 0 otherwise
- `topics`  
  activate topics using a value `> 0` for `frequency_hz` and an optionally different topic name as a value for `remap_to`

## Run

- Change to the directory where the _ixcom_driver_ is located:
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

## Topics 

The topics with the following standard _ROS2_ messages are implemented so far:  
  `/Imu:                         sensor_msgs/msg/Imu`  
  `/MagneticField:               sensor_msgs/msg/MagneticField`  
  `/NavSatFix_GNSS:              sensor_msgs/msg/NavSatFix`  
  `/NavSatFix_INS:               sensor_msgs/msg/NavSatFix`  
  `/NavSatStatus:                sensor_msgs/msg/NavSatStatus`  
  `/PoseWithCovarianceStamped    geometry_msgs.msg/PoseWithCovarianceStamped`  
  `/TwistStamped                 geometry_msgs.msg/TwistStamped`  
  `/Odometry:                    nav_msgs/msg/Odometry`  
  `/TimeReference:               sensor_msgs/msg/TimeReference`



# iXCOM-ROS2-driver   _- PYTHON Implementation -_

The **_ixcom_driver_** is a _ROS2_ node that is developed in _Python_. It establishes a connection to an _iNAT_, activates logs, receives the data and publishes them using topics.

## Dependencies

Install iXCOM Client:
```console
pip install ixcom
```

## General Information

While the _ixcom_driver_ is running it will appear in the _ROS2_ node list.

```console
~$ ros2 node list
/ixcom_driver_publisher
```

The node info shows the implemented features (not configured topics are not visible in this view, see [Configuration](#configuration)).

```console
~$ ros2 node info /ixcom_driver_pub
/ixcom_driver_pub

  Publishers:
    /Imu: sensor_msgs/msg/Imu
    /MagneticField: sensor_msgs/msg/MagneticField
    /NavSatFix: sensor_msgs/msg/NavSatFix
    /NavSatFix_INS: sensor_msgs/msg/NavSatFix
    /NavSatStatus: sensor_msgs/msg/NavSatStatus
    /Odometry: nav_msgs/msg/Odometry
    /PoseWithCovarianceStamped: geometry_msgs/msg/PoseWithCovarianceStamped
    /TimeReference: sensor_msgs/msg/TimeReference
    /TwistStamped: geometry_msgs/msg/TwistStamped
    /rosout: rcl_interfaces/msg/Log

  Action Servers:
    /realign: interfaces/action/Realign
    /reboot: interfaces/action/Reboot
    /stop: interfaces/action/Stop
```

## Build

```console
git clone git@gitlab.my.imar.de:automotive/okular/ixcom-ros2-driver.git ixcom-ros2-driver
cd ixcom-ros2-driver
colcon build
or
colcon build --packages-select interfaces ixcom_driver   (which builds only the specified packages, interfaces and ixcom_driver in this case)
```

## Configuration

The _ixcom_driver_ configuration is located in `src/ixcom_driver/params/config.json` and currently has the following structure.

```json
{
  "ip_config": {
    "ip_address": "192.168.1.30",
    "ip_port": "3000"
  },
  "serial_config": {
    "ignore": "1",
    "serial_port": "1",
    "baud_rate": "115200",
    "enable": "1"
  },
  "timestamp_mode": "ROS_EXECUTION",
  "qos": "SYSTEMDEFAULTS",
  "leap_seconds": "18",
  "topics": {
    "Imu": {"frequency_hz": "10", "remap_to": ""},
    "NavSatStatus": {"frequency_hz": "10", "remap_to": ""},
    "NavSatFix": {"frequency_hz": "10", "remap_to": ""},
    "NavSatFix_INS": {"frequency_hz": "10", "remap_to": ""},
    "TimeReference": {"frequency_hz": "10", "remap_to": ""},
    "MagneticField": {"frequency_hz": "10", "remap_to": ""},
    "Odometry": {"frequency_hz": "10", "remap_to": ""},
    "PoseWithCovarianceStamped": {"frequency_hz": "10", "remap_to": ""},
    "TwistStamped": {"frequency_hz": "10", "remap_to": ""}
  }
}
```

- `ip_config`  
  contains the connection data of an _iNAT_
- `serial_config`  
  the serial interface of the connected _iNAT_ can be configured with these data which can be skipped if `ignore` is set to `1`
- `timestamp_mode`  
  contains the timestamp mode that can be the ROS System Time (value: `ROS_EXECUTION`) or the _iNAT_ GPS Time (value: `GPS_SYNC`)
- `qos`  
  quality of service configuration corresponding to the _ROS2_ QoS profiles (valid values: `SYSTEMDEFAULTS`, `SENSORDATA`, `ACTIONSTATUS`, `PARAMETEREVENTS`, `PARAMETERS`, `SERVICES`)
- `leap_seconds`  
  set current leap seconds here if GNSS is not available or set to 0 otherwise
- `topics`  
  activate topics using a value `> 0` for `frequency_hz` and an optionally different topic name as a value for `remap_to`

## Run

- Change to the directory where the _ixcom_driver_ is located:
  ```console
  cd ixcom-ros2-driver
  ```
  
- Source and Run Publisher:
  ```console
  . install/setup.bash
  ros2 run ixcom_driver pub
  ```

- Run Subscriber (optional; this is just an example that shows some topic contents):
  ```console
  ros2 run ixcom_driver sub
  ```
  When started with the optional argument `-f`, the outputs show the frequencies the topic messages arrive with at the subscriber.

- Run Action Client (optional):
  ```console
  ros2 run ixcom_driver cli
  ```
    This command requires an argument. Possible arguments will be shown after entering the command without any.

## Topics 

The topics with the following standard _ROS2_ messages are implemented so far:  
  `/Imu:                         sensor_msgs/msg/Imu`  
  `/MagneticField:               sensor_msgs/msg/MagneticField`  
  `/NavSatFix:                   sensor_msgs/msg/NavSatFix`  
  `/NavSatFix_INS:               sensor_msgs/msg/NavSatFix`  
  `/NavSatStatus:                sensor_msgs/msg/NavSatStatus`  
  `/PoseWithCovarianceStamped    geometry_msgs.msg/PoseWithCovarianceStamped`  
  `/TwistStamped                 geometry_msgs.msg/TwistStamped`  
  `/Odometry:                    nav_msgs/msg/Odometry`  
  `/TimeReference:               sensor_msgs/msg/TimeReference`

## Action Servers

The following _Action Servers_ are implemented so far:  
  `/stop:      interfaces/action/Stop    ` - stops the publisher  
  `/realign:   interfaces/action/Realign ` - performs a realignment of the connected _iNAT_  
  `/reboot:    interfaces/action/Reboot `  - performs a reboot of the connected _iNAT_

## Logs

Logs are stored in the directory `log/run`

## Unit Tests

Use the following commands to run unit tests:
Source your setup files:

```console
. install/setup.bash
```
or
```console
. install/setup.zsh
```
Run the tests:

```console
colcon test
```
or
```console
colcon test --packages-select ixcom_driver --event-handlers console_cohesion+
```
or
```console
colcon test --packages-select ixcom_driver --pytest-args -k <class or file> --event-handlers console_cohesion+
```
or
```console
colcon test --packages-select ixcom_driver --pytest-args -m <marker> --event-handlers console_cohesion+
```
Show results:
```console
colcon test-result --all --verbose
```
