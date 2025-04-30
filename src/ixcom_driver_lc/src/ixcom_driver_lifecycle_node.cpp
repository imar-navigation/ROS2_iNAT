#include "ixcom_driver_lc/ixcom_driver_lifecycle_node.hpp"
#include "ixcom_driver_lc/modules/utility.hpp"
#include <cassert>
#include <functional>
#include <stdexcept>
#include <cmath>

int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);
    DriverNode::SharedPtr node = std::make_shared<DriverNode>();
    rclcpp::spin(node->get_node_base_interface());
    return 0;
}

DriverNode::DriverNode() : rclcpp_lifecycle::LifecycleNode("ixcom_driver_lc", rclcpp::NodeOptions().allow_undeclared_parameters(true)) {
    declare_parameters();
}

DriverNode::~DriverNode() {
    delete qos_;
}

DriverNode::CallbackReturn DriverNode::on_configure(const rclcpp_lifecycle::State& state) {

    LifecycleNode::on_configure(state);

    get_parameters();

    // std::string s = conf_->use_adapter_ ? "true" : "false";
    // RCLCPP_INFO(get_logger(), "%s", ("* * *   USE ADAPTER: " + s).c_str());

    RCLCPP_INFO(get_logger(), "%s", ("configured QoS: " + conf_->qos_type_).c_str());

    xcom::XComState xcom;
    if(!xcom.initialize()) {
//        std::cerr << "unable to initialize xcom state object" << "\n";
        RCLCPP_ERROR(get_logger(), "%s", "unable to initialize xcom state object");
        return CallbackReturn::FAILURE;
    }
    // connect to device
    xcom::TcpClient tcp_client(conf_->ip_address_, conf_->ip_port_);

    // define input source
    xcom::XcomClientReader tcp_reader(tcp_client);
    if(!tcp_reader.initialize()) {
//        std::cerr << "/" << "\n";
        RCLCPP_ERROR(get_logger(), "%s", "unable to initialize tcp_reader for config");
        return CallbackReturn::FAILURE;
    }
    // define output source (optional)
    xcom::XcomClientWriter tcp_writer(tcp_client);
    if(!tcp_writer.initialize()) {
//        std::cerr << "/*Unable to initialize tcp_writer*/" << "\n";
        RCLCPP_ERROR(get_logger(), "%s", "unable to initialize tcp_writer for config");
        return CallbackReturn::FAILURE;
    }
//    XcomHandler msg_handler(xcom);
//    std::cout << "creating client\n";
    RCLCPP_INFO(get_logger(), "%s", "creating configuration client");
    client_ = std::make_unique<XcomHandler>(shared_from_this(),
                                            xcom,
                                            conf_->serial_ignore_,
                                            conf_->serial_port_,
                                            conf_->serial_baud_,
                                            conf_->serial_enable_,
                                            conf_->leap_seconds_);
    xcom.set_reader(&tcp_reader);
    xcom.set_writer(&tcp_writer);
    xcom.disable_forwarding();
    uint16_t channel = 0;
    bool exit = false;
    while(!exit) {
        RCLCPP_INFO(get_logger(), "%s", ("connecting to iNAT @ "
                                         + conf_->ip_address_ + ":"
                                         + std::to_string(conf_->ip_port_)
                                         + " channel " + std::to_string(channel)
                                         + "... for the config").c_str());
        const auto cmd_open = xcom.get_xcomcmd_open(channel);
        tcp_writer.write(reinterpret_cast<const uint8_t *>(&cmd_open), sizeof(cmd_open));
        const auto rc = xcom.process();
        if(rc == xcom::XComState::ReturnCode::InvalidResponse) {
            if(client_->invalidChannel()) {
                RCLCPP_INFO(get_logger(), "%s", ("channel " + std::to_string(channel) + " is in use, trying another one...").c_str());
            }
            channel++;
            if(channel > 31) {
                RCLCPP_ERROR(get_logger(), "%s", "no available channel found");
                return CallbackReturn::FAILURE;
            }
            if(!tcp_reader.initialize()) {
//                perror("timeout: ");
                RCLCPP_ERROR(get_logger(), "%s", "tcp reader initialization failed");
                return CallbackReturn::FAILURE;
            }
        } else {
            exit = true;
        }
    }

    if(client_->complete()) {

        conf_->leap_seconds_ = client_->getLeapSeconds();

        RCLCPP_INFO(get_logger(), "%s", "configuration completed");

        rclcpp::Rate sleeper(1.0);
        sleeper.sleep();
        build_broadcast_transformstamped();
        sleeper.sleep();
        build_srv_extaid();
        sleeper.sleep();
        if(build_topic_imu())
            sleeper.sleep();
        if(build_topic_navsatstatus())
            sleeper.sleep();
        if(build_topic_navsatfix_gnss())
            sleeper.sleep();
        if(build_topic_navsatfix_ins())
            sleeper.sleep();
        if(build_topic_timereference())
            sleeper.sleep();
        if(build_topic_magneticfield())
            sleeper.sleep();
        if(build_topic_odometry())
            sleeper.sleep();
        if(build_topic_posewithcovariancestamped())
            sleeper.sleep();
        if(build_topic_twiststamped())
            sleeper.sleep();

        return CallbackReturn::SUCCESS;
    } else {
        std::cerr << "client error";
        return CallbackReturn::FAILURE;
    }
}

void DriverNode::declare_parameters() {
    conf_ = std::make_unique<Config>();

    // declare_parameter<bool>(conf_->PAR_USE_ADAPTER, false);

    declare_parameter<std::string>(conf_->PAR_IP_ADDRESS, std::string("192.168.1.30"));
    declare_parameter<int32_t>(conf_->PAR_IP_PORT, 3000);
    declare_parameter<bool>(conf_->PAR_SERIAL_IGNORE, true);
    declare_parameter<uint8_t>(conf_->PAR_SERIAL_PORT, 1);
    declare_parameter<int32_t>(conf_->PAR_SERIAL_BAUD, 115200);
    declare_parameter<bool>(conf_->PAR_SERIAL_ENABLE, false);
    declare_parameter<std::string>(conf_->PAR_TIMESTAMP_MODE, std::string("GPS"));   // valid values: GPS, ROS
    declare_parameter<std::string>(conf_->PAR_QOS, "SYSTEMDEFAULTS");                // valid values: SYSTEMDEFAULTS, SENSORDATA, CLOCK, PARAMETEREVENTS, PARAMETERS, SERVICES
    declare_parameter<int32_t>(conf_->PAR_LEAP_SECONDS, 18);
    declare_parameter<bool>(conf_->PAR_MLTP_ENABLE, false);
    declare_parameter<float>(conf_->PAR_MLTP_LAT, 0.0);
    declare_parameter<float>(conf_->PAR_MLTP_LON, 0.0);
    declare_parameter<float>(conf_->PAR_MLTP_ALT, 0.0);
    declare_parameter<float>(conf_->PAR_MAGNETOMETER_SCALE_FACTOR, 0.0);
    declare_parameter<int32_t>(conf_->PAR_IMU_FRQ, 0);
    declare_parameter<std::string>(conf_->PAR_IMU_REMAP_TO, std::string(""));
    declare_parameter<int32_t>(conf_->PAR_NAVSATSTATUS_FRQ, 0);
    declare_parameter<std::string>(conf_->PAR_NAVSATSTATUS_REMAP_TO, std::string(""));
    declare_parameter<int32_t>(conf_->PAR_NAVGNSS_FRQ, 0);
    declare_parameter<std::string>(conf_->PAR_NAVGNSS_REMAP_TO, std::string(""));
    declare_parameter<int32_t>(conf_->PAR_NAVINS_FRQ, 0);
    declare_parameter<std::string>(conf_->PAR_NAVINS_REMAP_TO, std::string(""));
    declare_parameter<int32_t>(conf_->PAR_TIMEREF_FRQ, 0);
    declare_parameter<std::string>(conf_->PAR_TIMEREF_REMAP_TO, std::string(""));
    declare_parameter<int32_t>(conf_->PAR_MAGFIELD_FRQ, 0);
    declare_parameter<std::string>(conf_->PAR_MAGFIELD_REMAP_TO, std::string(""));
    declare_parameter<int32_t>(conf_->PAR_ODOMETRY_FRQ, 0);
    declare_parameter<std::string>(conf_->PAR_ODOMETRY_REMAP_TO, std::string(""));
    declare_parameter<int32_t>(conf_->PAR_POSECOVSTAMPED_FRQ, 0);
    declare_parameter<std::string>(conf_->PAR_POSECOVSTAMPED_REMAP_TO, std::string(""));
    declare_parameter<int32_t>(conf_->PAR_TWISTSTAMPED_FRQ, 0);
    declare_parameter<std::string>(conf_->PAR_TWISTSTAMPED_REMAP_TO, std::string(""));
}

void DriverNode::get_parameters() {

    // get_parameter(conf_->PAR_USE_ADAPTER, conf_->use_adapter_);

    get_parameter(conf_->PAR_IP_ADDRESS, conf_->ip_address_);
    get_parameter(conf_->PAR_IP_PORT, conf_->ip_port_);
    get_parameter(conf_->PAR_SERIAL_IGNORE, conf_->serial_ignore_);
    get_parameter(conf_->PAR_SERIAL_PORT, conf_->serial_port_);
    get_parameter(conf_->PAR_SERIAL_BAUD, conf_->serial_baud_);
    get_parameter(conf_->PAR_SERIAL_ENABLE, conf_->serial_enable_);
    conf_->timestamp_mode_ = Config::TimestampModeFromString(
        get_parameter(conf_->PAR_TIMESTAMP_MODE).get_value<std::string>());
    conf_->qos_type_ = Config::qosTypeFromString(get_parameter(conf_->PAR_QOS).get_value<std::string>());
    qos_ = Config::qosFromString(conf_->qos_type_);
    conf_->leap_seconds_ = abs(get_parameter(conf_->PAR_LEAP_SECONDS).get_value<int32_t>());
    get_parameter(conf_->PAR_MLTP_ENABLE, conf_->mltp_enable_);
    get_parameter(conf_->PAR_MLTP_LAT, conf_->mltp_lat_);
    get_parameter(conf_->PAR_MLTP_LON, conf_->mltp_lon_);
    get_parameter(conf_->PAR_MLTP_ALT, conf_->mltp_alt_);
    get_parameter(conf_->PAR_MAGNETOMETER_SCALE_FACTOR, conf_->magnetometer_scale_factor_);
    get_parameter(conf_->PAR_IMU_FRQ, conf_->imu_frq_);
    get_parameter(conf_->PAR_IMU_REMAP_TO, conf_->imu_remap_);
    get_parameter(conf_->PAR_NAVSATSTATUS_FRQ, conf_->navsatstatus_frq_);
    get_parameter(conf_->PAR_NAVSATSTATUS_REMAP_TO, conf_->navsatstatus_remap_);
    get_parameter(conf_->PAR_NAVGNSS_FRQ, conf_->navsatfix_gnss_frq_);
    get_parameter(conf_->PAR_NAVGNSS_REMAP_TO, conf_->navsatfix_gnss_remap_);
    get_parameter(conf_->PAR_NAVINS_FRQ, conf_->navsatfix_ins_frq_);
    get_parameter(conf_->PAR_NAVINS_REMAP_TO, conf_->navsatfix_ins_remap_);
    get_parameter(conf_->PAR_TIMEREF_FRQ, conf_->timereference_frq_);
    get_parameter(conf_->PAR_TIMEREF_REMAP_TO, conf_->timereference_remap_);
    get_parameter(conf_->PAR_MAGFIELD_FRQ, conf_->magneticfield_frq_);
    get_parameter(conf_->PAR_MAGFIELD_REMAP_TO, conf_->magneticfield_remap_);
    get_parameter(conf_->PAR_ODOMETRY_FRQ, conf_->odometry_frq_);
    get_parameter(conf_->PAR_ODOMETRY_REMAP_TO, conf_->odometry_remap_);
    get_parameter(conf_->PAR_POSECOVSTAMPED_FRQ, conf_->posewithcovariancestamped_frq_);
    get_parameter(conf_->PAR_POSECOVSTAMPED_REMAP_TO, conf_->posewithcovariancestamped_remap_);
    get_parameter(conf_->PAR_TWISTSTAMPED_FRQ, conf_->twiststamped_frq_);
    get_parameter(conf_->PAR_TWISTSTAMPED_REMAP_TO, conf_->twiststamped_remap_);
}

void DriverNode::build_broadcast_transformstamped() {

    if(!xcom_tf_.initialize()) {
        RCLCPP_ERROR(get_logger(), "%s", "unable to initialize xcom state object for tf2 broadcast");
    }
    RCLCPP_INFO(get_logger(), "%s", "creating transform broadcaster");
    transformstamped_ = std::make_shared<TransformStamped>(shared_from_this(),
                                                           xcom_tf_,
                                                           conf_->ip_address_,
                                                           conf_->ip_port_,
                                                           conf_->timestamp_mode_,
                                                           conf_->leap_seconds_,
                                                           client_->getMaintiming(),
                                                           client_->getPrescaler());
}

void DriverNode::build_srv_extaid() {
    if(!xcom_srv_extaid_.initialize()) {
        RCLCPP_ERROR(get_logger(), "%s", "unable to initialize xcom state object for extaid service");
    }
    RCLCPP_INFO(get_logger(), "%s", "creating extaid service");
    srvextaid_ = std::make_shared<SrvExtAid>(shared_from_this(),
                                             xcom_srv_extaid_,
                                             conf_->ip_address_,
                                             conf_->ip_port_,
                                             conf_->leap_seconds_);
}

bool DriverNode::build_topic_imu() {

    bool built = true;

    if(conf_->imu_remap_.length() == 0) {
        imu_topic_name_ = conf_->TOPICNAME_IMU;
    } else {
        imu_topic_name_ = conf_->imu_remap_;
        RCLCPP_INFO(get_logger(), "%s", ("remapping topic " + conf_->TOPICNAME_IMU + " to " + imu_topic_name_).c_str());
    }

    if(conf_->imu_frq_ > 0) {

        if(!xcom_imu_.initialize()) {
            RCLCPP_ERROR(get_logger(), "[%s] %s", imu_topic_name_.c_str(), "unable to initialize xcom state object");
        }

        imu_ = std::make_shared<IMU>(shared_from_this(),
                                     xcom_imu_,
                                     transformstamped_,
                                     conf_->imu_frq_,
                                     imu_topic_name_,
                                     conf_->ip_address_,
                                     conf_->ip_port_,
                                     conf_->timestamp_mode_,
                                     conf_->leap_seconds_,
                                     client_->getMaintiming(),
                                     client_->getPrescaler(),
                                     *qos_);
    } else {
        built = false;
        RCLCPP_INFO(get_logger(), "[%s] %s", imu_topic_name_.c_str(), "- topic skipped");
    }
    return built;
}

bool DriverNode::build_topic_navsatstatus() {

    bool built = true;

    if(conf_->navsatstatus_remap_.length() == 0) {
        navsatstatus_topic_name_ = conf_->TOPICNAME_NAVSATSTATUS;
    } else {
        navsatstatus_topic_name_ = conf_->navsatstatus_remap_;
        RCLCPP_INFO(get_logger(), "%s", ("remapping topic " + conf_->TOPICNAME_NAVSATSTATUS + " to " + navsatstatus_topic_name_).c_str());
    }

    if(conf_->navsatstatus_frq_ > 0) {

        if(!xcom_navsatstatus_.initialize()) {
            RCLCPP_ERROR(get_logger(), "[%s] %s", navsatstatus_topic_name_.c_str(), "unable to initialize xcom state object");
        }

        navsatstatus_ = std::make_shared<NavSatStatus>(shared_from_this(),
                                                       xcom_navsatstatus_,
                                                       transformstamped_,
                                                       conf_->navsatstatus_frq_,
                                                       navsatstatus_topic_name_,
                                                       conf_->ip_address_,
                                                       conf_->ip_port_,
                                                       conf_->timestamp_mode_,
                                                       conf_->leap_seconds_,
                                                       client_->getMaintiming(),
                                                       client_->getPrescaler(),
                                                       *qos_);
    } else {
        built = false;
        RCLCPP_INFO(get_logger(), "[%s] %s", navsatstatus_topic_name_.c_str(), "- topic skipped");
    }
    return built;
}

bool DriverNode::build_topic_navsatfix_gnss() {

    bool built = true;

    if(conf_->navsatfix_gnss_remap_.length() == 0) {
        navgnss_topic_name_ = conf_->TOPICNAME_NAVGNSS;
    } else {
        navgnss_topic_name_ = conf_->navsatfix_gnss_remap_;
        RCLCPP_INFO(get_logger(), "%s", ("remapping topic " + conf_->TOPICNAME_NAVGNSS + " to " + navgnss_topic_name_).c_str());
    }

    if(conf_->navsatfix_gnss_frq_ > 0) {

        if(!xcom_navgnss_.initialize()) {
            RCLCPP_ERROR(get_logger(), "[%s] %s", navgnss_topic_name_.c_str(), "unable to initialize xcom state object");
        }

        navgnss_ = std::make_shared<NavGNSS>(shared_from_this(),
                                                       xcom_navgnss_,
                                                       transformstamped_,
                                                       conf_->navsatfix_gnss_frq_,
                                                       navgnss_topic_name_,
                                                       conf_->ip_address_,
                                                       conf_->ip_port_,
                                                       conf_->timestamp_mode_,
                                                       conf_->leap_seconds_,
                                                       client_->getMaintiming(),
                                                       client_->getPrescaler(),
                                                       *qos_);
    } else {
        built = false;
        RCLCPP_INFO(get_logger(), "[%s] %s", navgnss_topic_name_.c_str(), "- topic skipped");
    }
    return built;
}

bool DriverNode::build_topic_navsatfix_ins() {

    bool built = true;

    if(conf_->navsatfix_ins_remap_.length() == 0) {
        navins_topic_name_ = conf_->TOPICNAME_NAVINS;
    } else {
        navins_topic_name_ = conf_->navsatfix_ins_remap_;
        RCLCPP_INFO(get_logger(), "%s", ("remapping topic " + conf_->TOPICNAME_NAVINS+ " to " + navins_topic_name_).c_str());
    }

    if(conf_->navsatfix_ins_frq_ > 0) {

        if(!xcom_navins_.initialize()) {
            RCLCPP_ERROR(get_logger(), "[%s] %s", navins_topic_name_.c_str(), "unable to initialize xcom state object");
        }

        navins_ = std::make_shared<NavINS>(shared_from_this(),
                                                       xcom_navins_,
                                                       transformstamped_,
                                                       conf_->navsatfix_ins_frq_,
                                                       navins_topic_name_,
                                                       conf_->ip_address_,
                                                       conf_->ip_port_,
                                                       conf_->timestamp_mode_,
                                                       conf_->leap_seconds_,
                                                       client_->getMaintiming(),
                                                       client_->getPrescaler(),
                                                       *qos_);
    } else {
        built = false;
        RCLCPP_INFO(get_logger(), "[%s] %s", navins_topic_name_.c_str(), "- topic skipped");
    }
    return built;
}

bool DriverNode::build_topic_timereference() {

    bool built = true;

    if(conf_->timereference_remap_.length() == 0) {
        timeref_topic_name_ = conf_->TOPICNAME_TIMEREF;
    } else {
        timeref_topic_name_ = conf_->timereference_remap_;
        RCLCPP_INFO(get_logger(), "%s", ("remapping topic " + conf_->TOPICNAME_TIMEREF + " to " + timeref_topic_name_).c_str());
    }

    if(conf_->timereference_frq_ > 0) {

        if(!xcom_timeref_.initialize()) {
            RCLCPP_ERROR(get_logger(), "[%s] %s", timeref_topic_name_.c_str(), "unable to initialize xcom state object");
        }

        timeref_ = std::make_shared<TimeReference>(shared_from_this(),
                                                   xcom_timeref_,
                                                   transformstamped_,
                                                   conf_->timereference_frq_,
                                                   timeref_topic_name_,
                                                   conf_->ip_address_,
                                                   conf_->ip_port_,
                                                   conf_->timestamp_mode_,
                                                   conf_->leap_seconds_,
                                                   client_->getMaintiming(),
                                                   client_->getPrescaler(),
                                                   *qos_);
    } else {
        built = false;
        RCLCPP_INFO(get_logger(), "[%s] %s", timeref_topic_name_.c_str(), "- topic skipped");
    }
    return built;
}

bool DriverNode::build_topic_magneticfield() {

    bool built = true;

    if(conf_->magneticfield_remap_.length() == 0) {
        magfield_topic_name_ = conf_->TOPICNAME_MAGFIELD;
    } else {
        magfield_topic_name_ = conf_->magneticfield_remap_;
        RCLCPP_INFO(get_logger(), "%s", ("remapping topic " + conf_->TOPICNAME_MAGFIELD + " to " + magfield_topic_name_).c_str());
    }

    if(conf_->magneticfield_frq_ > 0) {

//        RCLCPP_WARN(get_logger(), "[%s] %s", magfield_topic_name_.c_str(), "- topic not implemented");

        if(!xcom_magfield_.initialize()) {
            RCLCPP_ERROR(get_logger(), "[%s] %s", magfield_topic_name_.c_str(), "unable to initialize xcom state object");
        }

        magfield_ = std::make_shared<MagneticField>(shared_from_this(),
                                                    xcom_magfield_,
                                                    transformstamped_,
                                                    conf_->magneticfield_frq_,
                                                    magfield_topic_name_,
                                                    conf_->ip_address_,
                                                    conf_->ip_port_,
                                                    conf_->timestamp_mode_,
                                                    conf_->leap_seconds_,
                                                    client_->getMaintiming(),
                                                    client_->getPrescaler(),
                                                    conf_->magnetometer_scale_factor_,
                                                    *qos_);
    } else {
        built = false;
        RCLCPP_INFO(get_logger(), "[%s] %s", magfield_topic_name_.c_str(), "- topic skipped");
    }
    return built;
}

bool DriverNode::build_topic_odometry() {

    bool built = true;

    if(conf_->odometry_remap_.length() == 0) {
        odometry_topic_name_ = conf_->TOPICNAME_ODOMETRY;
    } else {
        odometry_topic_name_ = conf_->odometry_remap_;
        RCLCPP_INFO(get_logger(), "%s", ("remapping topic " + conf_->TOPICNAME_ODOMETRY + " to " + odometry_topic_name_).c_str());
    }

    if(conf_->odometry_frq_ > 0) {

        if(!xcom_odometry_.initialize()) {
            RCLCPP_ERROR(get_logger(), "[%s] %s", odometry_topic_name_.c_str(), "unable to initialize xcom state object");
        }

        Config::MLTP mltp = {conf_->mltp_enable_,
                             conf_->mltp_lon_,
                             conf_->mltp_lat_,
                             conf_->mltp_alt_};
        odometry_ = std::make_shared<Odometry>(shared_from_this(),
                                                   xcom_odometry_,
                                                   transformstamped_,
                                                   conf_->odometry_frq_,
                                                   odometry_topic_name_,
                                                   conf_->ip_address_,
                                                   conf_->ip_port_,
                                                   conf_->timestamp_mode_,
                                                   conf_->leap_seconds_,
                                                   client_->getMaintiming(),
                                                   client_->getPrescaler(),
                                                   mltp,
                                                   *qos_);
    } else {
        built = false;
        RCLCPP_INFO(get_logger(), "[%s] %s", odometry_topic_name_.c_str(), "- topic skipped");
    }
    return built;
}

bool DriverNode::build_topic_posewithcovariancestamped() {

    bool built = true;

    if(conf_->posewithcovariancestamped_remap_.length() == 0) {
        posewithcovstamped_topic_name_ = conf_->TOPICNAME_POSECOVSTAMPED;
    } else {
        posewithcovstamped_topic_name_ = conf_->posewithcovariancestamped_remap_;
        RCLCPP_INFO(get_logger(), "%s", ("remapping topic " + conf_->TOPICNAME_POSECOVSTAMPED + " to " + posewithcovstamped_topic_name_).c_str());
    }

    if(conf_->posewithcovariancestamped_frq_ > 0) {

        if(!xcom_posewithcovstamped_.initialize()) {
            RCLCPP_ERROR(get_logger(), "[%s] %s", posewithcovstamped_topic_name_.c_str(), "unable to initialize xcom state object");
        }

        posewithcovstamped_ = std::make_shared<PoseWithCovarianceStamped>(shared_from_this(),
                                                   xcom_posewithcovstamped_,
                                                   transformstamped_,
                                                   conf_->posewithcovariancestamped_frq_,
                                                   posewithcovstamped_topic_name_,
                                                   conf_->ip_address_,
                                                   conf_->ip_port_,
                                                   conf_->timestamp_mode_,
                                                   conf_->leap_seconds_,
                                                   client_->getMaintiming(),
                                                   client_->getPrescaler(),
                                                   *qos_);
    } else {
        built = false;
        RCLCPP_INFO(get_logger(), "[%s] %s", posewithcovstamped_topic_name_.c_str(), "- topic skipped");
    }
    return built;
}

bool DriverNode::build_topic_twiststamped() {

    bool built = true;

    if(conf_->twiststamped_remap_.length() == 0) {
        twiststamped_topic_name_ = conf_->TOPICNAME_TWISTSTAMPED;
    } else {
        twiststamped_topic_name_ = conf_->twiststamped_remap_;
        RCLCPP_INFO(get_logger(), "%s", ("remapping topic " + conf_->TOPICNAME_TWISTSTAMPED + " to " + twiststamped_topic_name_).c_str());
    }

    if(conf_->twiststamped_frq_ > 0) {

        if(!xcom_twiststamped_.initialize()) {
            RCLCPP_ERROR(get_logger(), "[%s] %s", twiststamped_topic_name_.c_str(), "unable to initialize xcom state object");
        }

        twiststamped_ = std::make_shared<TwistStamped>(shared_from_this(),
                                                   xcom_twiststamped_,
                                                   transformstamped_,
                                                   conf_->twiststamped_frq_,
                                                   twiststamped_topic_name_,
                                                   conf_->ip_address_,
                                                   conf_->ip_port_,
                                                   conf_->timestamp_mode_,
                                                   conf_->leap_seconds_,
                                                   client_->getMaintiming(),
                                                   client_->getPrescaler(),
                                                   *qos_);
    } else {
        built = false;
        RCLCPP_INFO(get_logger(), "[%s] %s", twiststamped_topic_name_.c_str(), "- topic skipped");
    }
    return built;
}

DriverNode::CallbackReturn DriverNode::on_activate(const rclcpp_lifecycle::State& state) {

    // Activate all manages lifecycle entities (namely the lifecycle publishers)
    LifecycleNode::on_activate(state);

    bool activeTopicsPresent = false;

    // this is not a topic but a broadcaster
    if(transformstamped_) {
        transformstamped_->activate();
    }

    bool imu_ok = false;
    if(imu_) {
        for(int c = 0; c < 10; c++) {
            if(imu_->success()) {
                imu_->activate();
                activeTopicsPresent = true;
                imu_ok = true;
                break;
            }
            rclcpp::Rate sleeper(1.0);
            sleeper.sleep();
        }
        if(!imu_ok) {
            RCLCPP_ERROR(get_logger(), "[%s] %s", imu_topic_name_.c_str(), "configuration failed");
        }
    }

    bool navsatstatus_ok = false;
    if(navsatstatus_) {
        for(int c = 0; c < 10; c++) {
            if(navsatstatus_->success()) {
                navsatstatus_->activate();
                activeTopicsPresent = true;
                navsatstatus_ok = true;
                break;
            }
            rclcpp::Rate sleeper(1.0);
            sleeper.sleep();
        }
        if(!navsatstatus_ok) {
            RCLCPP_ERROR(get_logger(), "[%s] %s", navsatstatus_topic_name_.c_str(), "configuration failed");
        }
    }

    bool navgnss_ok = false;
    if(navgnss_) {
        for(int c = 0; c < 10; c++) {
            if(navgnss_->success()) {
                navgnss_->activate();
                activeTopicsPresent = true;
                navgnss_ok = true;
                break;
            }
            rclcpp::Rate sleeper(1.0);
            sleeper.sleep();
        }
        if(!navgnss_ok) {
            RCLCPP_ERROR(get_logger(), "[%s] %s", navgnss_topic_name_.c_str(), "configuration failed");
        }
    }

    bool navins_ok = false;
    if(navins_) {
        for(int c = 0; c < 10; c++) {
            if(navins_->success()) {
                navins_->activate();
                activeTopicsPresent = true;
                navins_ok = true;
                break;
            }
            rclcpp::Rate sleeper(1.0);
            sleeper.sleep();
        }
        if(!navins_ok) {
            RCLCPP_ERROR(get_logger(), "[%s] %s", navins_topic_name_.c_str(), "configuration failed");
        }
    }

    bool timeref_ok = false;
    if(timeref_) {
        for(int c = 0; c < 10; c++) {
            if(timeref_->success()) {
                timeref_->activate();
                activeTopicsPresent = true;
                timeref_ok = true;
                break;
            }
            rclcpp::Rate sleeper(1.0);
            sleeper.sleep();
        }
        if(!timeref_ok) {
            RCLCPP_ERROR(get_logger(), "[%s] %s", timeref_topic_name_.c_str(), "configuration failed");
        }
    }

    bool magfield_ok = false;
    if(magfield_) {
        for(int c = 0; c < 10; c++) {
            if(magfield_->success()) {
                magfield_->activate();
                activeTopicsPresent = true;
                magfield_ok = true;
                break;
            }
            rclcpp::Rate sleeper(1.0);
            sleeper.sleep();
        }
        if(!magfield_ok) {
            RCLCPP_ERROR(get_logger(), "[%s] %s", magfield_topic_name_.c_str(), "configuration failed");
        }
    }

    bool odometry_ok = false;
    if(odometry_) {
        for(int c = 0; c < 10; c++) {
            if(odometry_->success()) {
                odometry_->activate();
                activeTopicsPresent = true;
                odometry_ok = true;
                break;
            }
            rclcpp::Rate sleeper(1.0);
            sleeper.sleep();
        }
        if(!odometry_ok) {
            RCLCPP_ERROR(get_logger(), "[%s] %s", odometry_topic_name_.c_str(), "configuration failed");
        }
    }

    bool posewithcovstamped_ok = false;
    if(posewithcovstamped_) {
        for(int c = 0; c < 10; c++) {
            if(posewithcovstamped_->success()) {
                posewithcovstamped_->activate();
                activeTopicsPresent = true;
                posewithcovstamped_ok = true;
                break;
            }
            rclcpp::Rate sleeper(1.0);
            sleeper.sleep();
        }
        if(!posewithcovstamped_ok) {
            RCLCPP_ERROR(get_logger(), "[%s] %s", posewithcovstamped_topic_name_.c_str(), "configuration failed");
        }
    }

    bool twiststamped_ok = false;
    if(twiststamped_) {
        for(int c = 0; c < 10; c++) {
            if(twiststamped_->success()) {
                twiststamped_->activate();
                activeTopicsPresent = true;
                twiststamped_ok = true;
                break;
            }
            rclcpp::Rate sleeper(1.0);
            sleeper.sleep();
        }
        if(!twiststamped_ok) {
            RCLCPP_ERROR(get_logger(), "[%s] %s", twiststamped_topic_name_.c_str(), "configuration failed");
        }
    }

    if(activeTopicsPresent) {
        rclcpp::Rate sleeper(2.0);
        sleeper.sleep();
        RCLCPP_INFO(get_logger(), "%s", "listeners can now subscribe to active topics");
    } else {
        RCLCPP_INFO(get_logger(), "%s", "no topics configured");
    }

    return CallbackReturn::SUCCESS;
}

DriverNode::CallbackReturn DriverNode::on_deactivate(const rclcpp_lifecycle::State& state) {

//    std::cout << "on_deactivate" << std::endl;
    LifecycleNode::on_deactivate(state);
    return CallbackReturn::SUCCESS;
}

DriverNode::CallbackReturn DriverNode::on_cleanup(const rclcpp_lifecycle::State& state) {

    LifecycleNode::on_cleanup(state);

    clearModules();

    return CallbackReturn::SUCCESS;
}

DriverNode::CallbackReturn DriverNode::on_shutdown(const rclcpp_lifecycle::State& state) {

    LifecycleNode::on_shutdown(state);

    clearModules();

    return CallbackReturn::SUCCESS;
}

void DriverNode::clearModules() {

    imu_->cleanup();
    imu_.reset();

    navsatstatus_->cleanup();
    navsatstatus_.reset();

    navgnss_->cleanup();
    navgnss_.reset();

    navins_->cleanup();
    navins_.reset();

    timeref_->cleanup();
    timeref_.reset();

    magfield_->cleanup();
    magfield_.reset();

    odometry_->cleanup();
    odometry_.reset();

    posewithcovstamped_->cleanup();
    posewithcovstamped_.reset();

    twiststamped_->cleanup();
    twiststamped_.reset();

    transformstamped_->cleanup();
    transformstamped_.reset();

    srvextaid_->cleanup();
    srvextaid_.reset();
}




