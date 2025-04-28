#include "ixcom_driver_lc/modules/odometry.hpp"
#include "ixcom_driver_lc/ixcom_driver_conf.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <ixcom_driver_lc/modules/utility.hpp>
#include <thread>


Odometry::Odometry(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                   xcom::XComState &state,
                   TransformStamped::SharedPtr tf2,
                   int32_t topic_freq,
                   const std::string &topic_name,
                   const std::string &ip_address,
                   int32_t ip_port,
                   Config::TimestampMode timestamp_mode,
                   int32_t leap_seconds,
                   uint16_t maintiming,
                   uint16_t prescaler,
                   const Config::MLTP &mltp,
                   const rclcpp::QoS &qos) :
    MessageHandler(&state),
    //XComParameters_Imu(&state),
    //xcom::CommandHandler(&state),
    xcom::ResponseHandler(&state),
    node_(node),
    xcom_(state),
    ip_address_(ip_address),
    ip_port_(ip_port),
    topic_name_(topic_name),
    topic_freq_(topic_freq),
    timestamp_mode_(timestamp_mode),
    leap_seconds_(leap_seconds),
    maintiming_(maintiming),
    prescaler_(prescaler),
    mltp_(mltp),
    qos_(qos),
    tf2_(tf2) {
    setLocalTangentialPlane();
    std::thread th(&Odometry::init, this);
    th.detach();
}

Odometry::~Odometry() {
    cleanup();
}

void Odometry::cleanup() {
    run_frq_mon_ = false;
    xcom_.send_message(xcom_.get_xcomcmd_clearall());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    xcom_.set_rc(xcom::XComState::ReturnCode::Ok);
    xcom_.forced_exit();
}

void Odometry::init() {

    if(prescaler_ == 0) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", topic_name_.c_str(), "prescaler is 0");
    }
    if(maintiming_ == 0) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", topic_name_.c_str(), "maintiming is 0");
    }

    if(prescaler_ > 0) {
        if(topic_freq_ > maintiming_ / prescaler_) {
            topic_freq_ = maintiming_ / prescaler_;
        }
    }

    // connect to device
    xcom::TcpClient tcp_client(ip_address_, ip_port_);
    // define input source
    xcom::XcomClientReader tcp_reader(tcp_client);
    if(!tcp_reader.initialize()) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", topic_name_.c_str(), "unable to initialize tcp_reader");
    }
    // define output source (optional)
    xcom::XcomClientWriter tcp_writer(tcp_client);
    if(!tcp_writer.initialize()) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", topic_name_.c_str(), "unable to initialize tcp_writer");
    }

    xcom_.set_reader(&tcp_reader);
    xcom_.set_writer(&tcp_writer);
    xcom_.disable_forwarding();

    bool exit = false;
    RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(), ("connecting to iNAT @ "
                                     + ip_address_ + ":"
                                     + std::to_string(ip_port_)
                                     + "...").c_str());
    while(!exit) {
//        RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(), ("connecting to iNAT @: "
//                                         + ip_address_ + ":"
//                                         + std::to_string(ip_port_)
//                                         + " channel " + std::to_string(channel_)
//                                         + "...").c_str());
        const auto cmd_open = xcom_.get_xcomcmd_open(channel_);
        tcp_writer.write(reinterpret_cast<const uint8_t *>(&cmd_open), sizeof(cmd_open));
        const auto rc = xcom_.process();
        if(rc == xcom::XComState::ReturnCode::InvalidResponse) {
            if(invalid_channel_) {
//                RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(),
//                            ("channel " + std::to_string(channel_) + " is in use, trying another one...").c_str());
            }
            channel_++;
            if(channel_ > 31) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] %s", topic_name_.c_str(), "no available channel found");
            }
            if(!tcp_reader.initialize()) {
//                perror("timeout: ");
                RCLCPP_ERROR(node_->get_logger(), "[%s] %s", topic_name_.c_str(), "tcp reader initialization failed");
            }
        } else {
            exit = true;
        }
    }
}

void Odometry::handle_response(XCOMResp response) noexcept {
    if(response == XCOMResp::OK) {
        invalid_channel_ = false;

        if(!init_done_) {

            RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(),
                        ("connected to iNAT on channel " + std::to_string(channel_)).c_str());

            odometry_msg_.header.frame_id = "enu";
            odometry_msg_.child_frame_id = "inat_enclosure";

            auto cmd_clearall = xcom_.get_xcomcmd_clearall();
            xcom_.send_message(cmd_clearall);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            uint16_t div1 = calculateDividerForRate(topic_freq_, maintiming_, prescaler_);
            msg_INSSOL_frq_ = calculateRateForDivider(div1, maintiming_, prescaler_);
            auto cmd_add_inssol = xcom_.get_xcomcmd_enablelog(XCOM_MSGID_INSSOL, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC, div1);
            xcom_.send_message(cmd_add_inssol);

            uint16_t div2 = calculateDividerForRate(topic_freq_, maintiming_, prescaler_);
            msg_IMUCORR_frq_ = calculateRateForDivider(div2, maintiming_, prescaler_);
            auto cmd_add_imucorr = xcom_.get_xcomcmd_enablelog(XCOM_MSGID_IMUCORR, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC, div2);
            xcom_.send_message(cmd_add_imucorr);

            uint16_t div3 = calculateDividerForRate(((topic_freq_ > Config::FRQ_EKF) ? (Config::FRQ_EKF) : (topic_freq_)), maintiming_, prescaler_);
            msg_EKFSTDDEV_frq_ = calculateRateForDivider(div3, maintiming_, prescaler_);
            auto cmd_add_ekfstddev = xcom_.get_xcomcmd_enablelog(XCOM_MSGID_EKFSTDDEV, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC, div3);
            xcom_.send_message(cmd_add_ekfstddev);

            setup_freq_ = calculateRateForDivider(std::min({div1, div2, div3}), maintiming_, prescaler_);
            time_delta_ = static_cast<uint64_t>(1.0 / double(setup_freq_) * 1e6);
            if(setup_freq_ < topic_freq_) {
                RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(),
                            ("configured frequency ("
                             + std::to_string(topic_freq_)
                             + " Hz) is reduced to the next possible: "
                             + std::to_string(setup_freq_)
                             + " Hz").c_str());
            } else {
                RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(),
                            ("configured @ "
                             + std::to_string(setup_freq_)
                             + " Hz").c_str());
            }

            pub_ = node_->create_publisher<OdometryMsg>(topic_name_, qos_);
            success_ = true;
        }
        init_done_ = true;
    } else {
        if(response == XCOMResp::INVALIDCHANNEL) {
            invalid_channel_ = true;
        } else if(response == XCOMResp::LOGEXISTS) {
            RCLCPP_WARN(node_->get_logger(), "[%s] %s", topic_name_.c_str(), "requested log already exists");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "[%s] %s: %d", topic_name_.c_str(), "invalid response received", static_cast<int>(response));
        }
        xcom_.set_rc(xcom::XComState::ReturnCode::InvalidResponse);
        xcom_.forced_exit();
    }
}

void Odometry::handle_xcom_msg(const XCOMmsg_INSSOL &msg) noexcept {
    msg_INSSOL_age_ = 0;
    updateINSSOL(msg);
}

void Odometry::handle_xcom_msg(const XCOMmsg_IMUCORR &msg) noexcept {
    msg_IMUCORR_age_ = 0;
    updateIMUCORR(msg);
}

void Odometry::handle_xcom_msg(const XCOMmsg_EKFSTDDEV &msg) noexcept {
    msg_EKFSTDDEV_age_ = 0;
    updateEKFSTDDEV(msg);
}

void Odometry::activate() {
    active_ = true;
    RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(), "+ configured and activated");

    t_pub_ = std::chrono::high_resolution_clock::now();
    t_pub_upd_ = std::chrono::high_resolution_clock::now();
    std::thread th_mon(&Odometry::frq_mon, this);
    th_mon.detach();
}

uint16_t Odometry::getSetupFreq() {
    return setup_freq_;
}

bool Odometry::success() {
    return success_;
}

void Odometry::updateINSSOL(const XCOMmsg_INSSOL &msg) {

    if(!reference_is_set_) {
        ref_ = lla2ecef(msg.pos[0], msg.pos[1], msg.altitude);
        if(mltp_.enable) {
            ref_.x -= ltp_reference_.x;
            ref_.y -= ltp_reference_.y;
            ref_.z -= ltp_reference_.z; 
        }
        reference_is_set_ = true;
    }

    if(reference_is_set_) {
        odometry_msg_.pose.pose.position = ecef2enu(ref_, msg.pos[0], msg.pos[1]);

        tf2::Quaternion q;
        q.setRPY(msg.rpy[0], msg.rpy[1], msg.rpy[2]);
        odometry_msg_.pose.pose.orientation = tf2::toMsg(q);

        insSolDataIsSet_ = true;
    }
}

void Odometry::updateEKFSTDDEV(const XCOMmsg_EKFSTDDEV &msg) {

    odometry_msg_.pose.covariance[0] = msg.pos[1] * msg.pos[1];
    odometry_msg_.pose.covariance[7] = msg.pos[0] * msg.pos[0];
    odometry_msg_.pose.covariance[14] = msg.pos[2] * msg.pos[2];
    odometry_msg_.pose.covariance[21] = msg.tilt[0] * msg.tilt[0];
    odometry_msg_.pose.covariance[28] = msg.tilt[1] * msg.tilt[1];
    odometry_msg_.pose.covariance[35] = msg.tilt[2] * msg.tilt[2];

    ekfDataIsSet_ = true;
}

void Odometry::updateIMUCORR(const XCOMmsg_IMUCORR &msg) {

    odometry_msg_.twist.twist.angular.x = msg.omg[0];
    odometry_msg_.twist.twist.angular.y = msg.omg[1];
    odometry_msg_.twist.twist.angular.z = msg.omg[2];

    imuCorrDataIsSet_ = true;

    gps_time_ = UpdateGPSTime(msg.header, leap_seconds_);
    publish();
}

void Odometry::publish() {

    if(!(insSolDataIsSet_ && ekfDataIsSet_ && imuCorrDataIsSet_)) {
        return;
    }

    if(active_) {

        // auto t1 = std::chrono::high_resolution_clock::now();

        odometry_msg_.header.stamp = (timestamp_mode_ == Config::TimestampMode::GPS) ? gps_time_ : node_->now();

        pub_->publish(odometry_msg_);

        t_pub_ = t_pub_upd_;
        t_pub_upd_ = std::chrono::high_resolution_clock::now();
        duration_ = std::chrono::duration_cast<std::chrono::microseconds>(t_pub_upd_ - t_pub_).count();

        size_t n = pub_->get_subscription_count();
        if(n > num_of_subscribers_) {
            RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(), "* new listener joined");
            tf2_->subscriberAdded();
            num_of_subscribers_ = n;
            RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(), ("current number of subscriptions: " + std::to_string(num_of_subscribers_)).c_str());
        } else if(n < num_of_subscribers_) {
            RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(), "+ a listener leaved");
            num_of_subscribers_ = n;
            RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(), ("current number of subscriptions: " + std::to_string(num_of_subscribers_)).c_str());
        }

        // auto t2 = std::chrono::high_resolution_clock::now();
        // auto d = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

        // std::cout << "odo publish: " << std::to_string(d) << std::endl;
    }
}

void Odometry::setLocalTangentialPlane() {
    if(mltp_.enable) {
        ltp_reference_.x = mltp_.lon;
        ltp_reference_.y = mltp_.lat;
        ltp_reference_.z = mltp_.alt;
    } else {
        ltp_reference_.x = 0.0;
        ltp_reference_.y = 0.0;
        ltp_reference_.z = 0.0;
    }
}

Odometry::Point Odometry::lla2ecef(float longitutde, float latitude, float altitude) {
    // Implementation based on https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
    constexpr static double SEMIMAJOR_AXIS = 6378137.0;
    constexpr static double SEMIMINOR_AXIS = 6356752.31424518;

    // Compute the prime vertical radius of curvature
    double cos_term = cos(latitude);
    double sin_term = sin(latitude);

    double n = (SEMIMAJOR_AXIS * SEMIMAJOR_AXIS) /
               std::sqrt(
                   SEMIMAJOR_AXIS * SEMIMAJOR_AXIS * cos_term * cos_term +
                   SEMIMINOR_AXIS * SEMIMINOR_AXIS * sin_term * sin_term);

    // Calculate x, y and z in geoecentric coordinates from the longitude, latitude and altitude, using
    // the prime vertical radius of curvature n
    Point p;
    p.x = (n + altitude) * cos(latitude) * cos(longitutde);
    p.y = (n + altitude) * cos(latitude) * sin(longitutde);
    p.z = (n * ((SEMIMINOR_AXIS * SEMIMINOR_AXIS) / (SEMIMAJOR_AXIS * SEMIMAJOR_AXIS)) + altitude) * sin(latitude);

    return p;
}

Odometry::Point Odometry::ecef2enu(Point p, double longitude, double latitude) {
    // Implementation based on https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ECEF_to_ENU
    Point enu;
    enu.x = -sin(longitude) * p.x + cos(longitude) * p.y;
    enu.y = -sin(latitude) * cos(longitude) * p.x - sin(latitude) * sin(longitude) * p.y + cos(latitude) * p.z;
    enu.z = cos(latitude) * cos(longitude) * p.x + cos(latitude) * sin(longitude) * p.y + sin(latitude) * p.z;

    return enu;
}

void Odometry::frq_mon() {

    bool unknown_state_printed = false;
    bool msg_INSSOL_ok = true;
    bool msg_IMUCORR_ok = true;
    bool msg_EKFSTDDEV_ok = true;

    while(run_frq_mon_) {

        bool pub_state_changed = false;

        if(duration_ > (time_delta_ * 2.0)) {
            if(pub_state_ != PubState::ERR) {
                pub_state_changed = true;
                pub_state_ = PubState::ERR;
            }
        } else if(duration_ > (time_delta_ * 1.3)) {
            if(pub_state_ != PubState::WARN) {
                pub_state_changed = true;
                pub_state_ = PubState::WARN;
            }
            // RCLCPP_ERROR(node_->get_logger(), "[%s] %s", topic_name_.c_str(), "frequency delayed by more than 100 %");
        } else if(duration_ == 0) {
            if(pub_state_ != PubState::UNKNOWN) {
                pub_state_changed = true;
                pub_state_ = PubState::UNKNOWN;
            }
        } else {
            if(pub_state_ != PubState::OK) {
                pub_state_changed = true;
                pub_state_ = PubState::OK;
            }
            // RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(), ("time delta / duration: " + std::to_string(time_delta_) + " / " + std::to_string(duration)).c_str());
        }

        if(pub_state_changed) {
            switch(pub_state_) {
            case PubState::UNKNOWN: {
                if(!unknown_state_printed) {
                    RCLCPP_WARN(node_->get_logger(), "[%s] %s", topic_name_.c_str(), "topic frequency unknown");
                    unknown_state_printed = true;
                }
                break;
            }
            case PubState::OK: {
                RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(), "topic frequency ok");
                break;
            }
            case PubState::WARN: {
                RCLCPP_WARN(node_->get_logger(), "[%s] %s", topic_name_.c_str(), "topic frequency delayed by more than 30 %");
                break;
            }
            case PubState::ERR: {
                RCLCPP_ERROR(node_->get_logger(), "[%s] %s", topic_name_.c_str(), "topic frequency delayed by more than 100 %");
                break;
            }
            }
        }

        if(msg_INSSOL_ok && (msg_INSSOL_age_ > xcom_age_max_)) {
            msg_INSSOL_ok = false;
            RCLCPP_ERROR(node_->get_logger(), "[%s] %s (%s)", topic_name_.c_str(), "missing iNAT data", "XCOMmsg_INSSOL");
        } else if(!msg_INSSOL_ok && !(msg_INSSOL_age_ > xcom_age_max_)) {
            msg_INSSOL_ok = true;
            RCLCPP_INFO(node_->get_logger(), "[%s] %s (%s)", topic_name_.c_str(), "receiving iNAT data", "XCOMmsg_INSSOL");
        }
        if(msg_IMUCORR_ok && (msg_IMUCORR_age_ > xcom_age_max_)) {
            msg_IMUCORR_ok = false;
            RCLCPP_ERROR(node_->get_logger(), "[%s] %s (%s)", topic_name_.c_str(), "missing iNAT data", "XCOMmsg_IMUCORR");
        } else if(!msg_IMUCORR_ok && !(msg_IMUCORR_age_ > xcom_age_max_)) {
            msg_IMUCORR_ok = true;
            RCLCPP_INFO(node_->get_logger(), "[%s] %s (%s)", topic_name_.c_str(), "receiving iNAT data", "XCOMmsg_IMUCORR");
        }
        if(msg_EKFSTDDEV_ok && (msg_EKFSTDDEV_age_ > xcom_age_max_)) {
            msg_EKFSTDDEV_ok = false;
            RCLCPP_ERROR(node_->get_logger(), "[%s] %s (%s)", topic_name_.c_str(), "missing iNAT data", "XCOMmsg_EKFSTDDEV");
        } else if(!msg_EKFSTDDEV_ok && !(msg_EKFSTDDEV_age_ > xcom_age_max_)) {
            msg_EKFSTDDEV_ok = true;
            RCLCPP_INFO(node_->get_logger(), "[%s] %s (%s)", topic_name_.c_str(), "receiving iNAT data", "XCOMmsg_EKFSTDDEV");
        }

        if(msg_INSSOL_c_ > (mon_frq_ / msg_INSSOL_frq_)) {
            if(msg_INSSOL_age_ == age_max_val_) {
                msg_INSSOL_age_ = xcom_age_max_ + 1;
            } else {
                msg_INSSOL_age_++;
            }
            msg_INSSOL_c_ = 0;
        }
        if(msg_IMUCORR_c_ > (mon_frq_ / msg_IMUCORR_frq_)) {
            if(msg_IMUCORR_age_ == age_max_val_) {
                msg_IMUCORR_age_ = xcom_age_max_ + 1;
            } else {
                msg_IMUCORR_age_++;
            }
            msg_IMUCORR_c_ = 0;
        }
        if(msg_EKFSTDDEV_c_ > (mon_frq_ / msg_EKFSTDDEV_frq_)) {
            if(msg_EKFSTDDEV_age_ == age_max_val_) {
                msg_EKFSTDDEV_age_ = xcom_age_max_ + 1;
            } else {
                msg_EKFSTDDEV_age_++;
            }
            msg_EKFSTDDEV_c_ = 0;
        }

        msg_INSSOL_c_++;
        msg_IMUCORR_c_++;
        msg_EKFSTDDEV_c_++;

        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::this_thread::sleep_for(std::chrono::milliseconds(mon_t_gap_));
    }
}
