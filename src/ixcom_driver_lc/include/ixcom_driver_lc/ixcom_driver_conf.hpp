#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <rclcpp/qos.hpp>
#include <iostream>

struct Config
{
    using UniquePtr = std::unique_ptr<Config>;

    static const uint8_t FRQ_GNSS = 1;
    static const uint8_t FRQ_EKF = 20;

    enum class TimestampMode : uint8_t {
        ROS,
        GPS
    };

    struct MLTP {
        bool enable;
        float lon;
        float lat;
        float alt;
    };

    // const std::string PAR_USE_ADAPTER = "use_adapter";
    // bool use_adapter_;

    const std::string PAR_IP_ADDRESS = "ip.address";
    std::string ip_address_;
    const std::string PAR_IP_PORT = "ip.port";
    int32_t ip_port_;

    const std::string PAR_SERIAL_IGNORE = "serial.ignore";
    bool serial_ignore_;
    const std::string PAR_SERIAL_PORT = "serial.port";
    uint8_t serial_port_;
    const std::string PAR_SERIAL_BAUD = "serial.baud";
    int32_t serial_baud_;
    const std::string PAR_SERIAL_ENABLE = "serial.enable";
    bool serial_enable_;

    const std::string PAR_TIMESTAMP_MODE = "timestamp_mode";
    TimestampMode timestamp_mode_;
    const std::string PAR_QOS = "qos";
    std::string qos_type_;
    const std::string PAR_LEAP_SECONDS = "leap_seconds";
    int32_t leap_seconds_;
    const std::string PAR_MLTP_ENABLE = "manual_local_tangential_plane.enable";
    bool mltp_enable_;
    const std::string PAR_MLTP_LAT = "manual_local_tangential_plane.lat";
    float mltp_lat_;
    const std::string PAR_MLTP_LON = "manual_local_tangential_plane.lon";
    float mltp_lon_;
    const std::string PAR_MLTP_ALT = "manual_local_tangential_plane.alt";
    float mltp_alt_;
    const std::string PAR_MAGNETOMETER_SCALE_FACTOR = "magnetometer_scale_factor";
    float magnetometer_scale_factor_;

    const std::string TOPICNAME_IMU = "Imu";
    const std::string PAR_IMU_FRQ = "topics." + TOPICNAME_IMU + ".frequency_hz";
    int32_t imu_frq_;
    const std::string PAR_IMU_REMAP_TO = "topics." + TOPICNAME_IMU + ".remap_to";
    std::string imu_remap_;

    const std::string TOPICNAME_NAVSATSTATUS = "NavSatStatus";
    const std::string PAR_NAVSATSTATUS_FRQ = "topics." + TOPICNAME_NAVSATSTATUS + ".frequency_hz";
    int32_t navsatstatus_frq_;
    const std::string PAR_NAVSATSTATUS_REMAP_TO = "topics." + TOPICNAME_NAVSATSTATUS + ".remap_to";
    std::string navsatstatus_remap_;

    const std::string TOPICNAME_NAVGNSS = "NavSatFix_GNSS";
    const std::string PAR_NAVGNSS_FRQ = "topics." + TOPICNAME_NAVGNSS + ".frequency_hz";
    int32_t navsatfix_gnss_frq_;
    const std::string PAR_NAVGNSS_REMAP_TO = "topics." + TOPICNAME_NAVGNSS + ".remap_to";
    std::string navsatfix_gnss_remap_;

    const std::string TOPICNAME_NAVINS = "NavSatFix_INS";
    const std::string PAR_NAVINS_FRQ = "topics." + TOPICNAME_NAVINS + ".frequency_hz";
    int32_t navsatfix_ins_frq_;
    const std::string PAR_NAVINS_REMAP_TO = "topics." + TOPICNAME_NAVINS + ".remap_to";
    std::string navsatfix_ins_remap_;

    const std::string TOPICNAME_TIMEREF = "TimeReference";
    const std::string PAR_TIMEREF_FRQ = "topics." + TOPICNAME_TIMEREF + ".frequency_hz";
    int32_t timereference_frq_;
    const std::string PAR_TIMEREF_REMAP_TO = "topics." + TOPICNAME_TIMEREF + ".remap_to";
    std::string timereference_remap_;

    const std::string TOPICNAME_MAGFIELD = "MagneticField";
    const std::string PAR_MAGFIELD_FRQ = "topics." + TOPICNAME_MAGFIELD + ".frequency_hz";
    int32_t magneticfield_frq_;
    const std::string PAR_MAGFIELD_REMAP_TO = "topics." + TOPICNAME_MAGFIELD + ".remap_to";
    std::string magneticfield_remap_;

    const std::string TOPICNAME_ODOMETRY = "Odometry";
    const std::string PAR_ODOMETRY_FRQ = "topics." + TOPICNAME_ODOMETRY + ".frequency_hz";
    int32_t odometry_frq_;
    const std::string PAR_ODOMETRY_REMAP_TO = "topics." + TOPICNAME_ODOMETRY + ".remap_to";
    std::string odometry_remap_;
    const std::string PAR_ODOMETRY_FRAME_ID = "topics." + TOPICNAME_ODOMETRY + ".frame_id";
    std::string odometry_frame_id_;

    const std::string TOPICNAME_POSECOVSTAMPED = "PoseWithCovarianceStamped";
    const std::string PAR_POSECOVSTAMPED_FRQ = "topics." + TOPICNAME_POSECOVSTAMPED + ".frequency_hz";
    int32_t posewithcovariancestamped_frq_;
    const std::string PAR_POSECOVSTAMPED_REMAP_TO = "topics." + TOPICNAME_POSECOVSTAMPED + ".remap_to";
    std::string posewithcovariancestamped_remap_;

    const std::string TOPICNAME_TWISTSTAMPED = "TwistStamped";
    const std::string PAR_TWISTSTAMPED_FRQ = "topics." + TOPICNAME_TWISTSTAMPED + ".frequency_hz";
    int32_t twiststamped_frq_;
    const std::string PAR_TWISTSTAMPED_REMAP_TO = "topics." + TOPICNAME_TWISTSTAMPED + ".remap_to";
    std::string twiststamped_remap_;

    static Config::TimestampMode TimestampModeFromString(const std::string& mode)
    {
        std::string transformed;
        transformed.resize(mode.size());
        std::transform(mode.begin(), mode.end(), transformed.begin(), ::tolower);

        if (transformed == "ros") {
            return Config::TimestampMode::ROS;
        } else if (transformed == "gps") {
            return Config::TimestampMode::GPS;
        }

        throw std::runtime_error("Invalid value \"" + mode + "\" for enum TimestampMode. Valid values: ROS, GPS.");
    }

    static rclcpp::QoS* qosFromString(const std::string &qos_s) {
        rclcpp::QoS *qos;
        if(qos_s == "SYSTEMDEFAULTS") {
            qos = new rclcpp::SystemDefaultsQoS();
            return qos;
        } else if(qos_s == "SENSORDATA") {
            qos = new rclcpp::SensorDataQoS();
            return qos;
        } else if(qos_s == "CLOCK") {
            qos = new rclcpp::ClockQoS();
            return qos;
        } else if(qos_s == "PARAMETEREVENTS") {
            qos = new rclcpp::ParameterEventsQoS();
            return qos;
        } else if(qos_s == "PARAMETERS") {
            qos = new rclcpp::ParametersQoS();
            return qos;
        } else if(qos_s == "SERVICES") {
            qos = new rclcpp::ServicesQoS();
            return qos;
        } else if(qos_s == "CUSTOM") {
            static const rmw_qos_profile_t my_qos_profile =
                {
                    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                    1000,
                    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                    RMW_QOS_POLICY_DURABILITY_VOLATILE,
                    RMW_QOS_DEADLINE_DEFAULT,
                    RMW_QOS_LIFESPAN_DEFAULT,
                    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                    false
                };
            // rclcpp::QoSInitialization::from_rmw(my_qos_profile);
            qos = new rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(my_qos_profile));
            return qos;
        }

        throw std::runtime_error("Invalid value \"" + qos_s + "\" for QOS. Valid values: SYSTEMDEFAULTS, SENSORDATA, CLOCK, PARAMETEREVENTS, PARAMETERS, SERVICES.");
    }

    static std::string qosTypeFromString(const std::string &qos_s) {
        std::string s;
        s.resize(qos_s.size());
        std::transform(qos_s.begin(), qos_s.end(), s.begin(), ::tolower);
        if(s == "systemdefaults") {
            return "SYSTEMDEFAULTS";
        } else if(s == "sensordata") {
            return "SENSORDATA";
        } else if(s == "clock") {
            return "CLOCK";
        } else if(s == "parameterevents") {
            return "PARAMETEREVENTS";
        } else if(s == "parameters") {
            return "PARAMETERS";
        } else if(s == "services") {
            return "SERVICES";
        } else if(s == "custom") {
            return "CUSTOM";
        }

        throw std::runtime_error("Invalid value \"" + qos_s + "\" for QOS. Valid values: SYSTEMDEFAULTS, SENSORDATA, CLOCK, PARAMETEREVENTS, PARAMETERS, SERVICES.");
    }
};
