#include "ixcom_driver_lc/modules/magnetic_field.hpp"
#include "ixcom_driver_lc/ixcom_driver_conf.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <ixcom_driver_lc/modules/utility.hpp>
#include <thread>


MagneticField::MagneticField(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
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
                             float magfield_scalefactor,
                             const rclcpp::QoS &qos) :
    MessageHandler(&state),
    XComParameters_MagField(&state),
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
    magfield_scalefactor_(magfield_scalefactor),
    qos_(qos),
    tf2_(tf2) {
    std::thread th(&MagneticField::init, this);
    th.detach();
}

MagneticField::~MagneticField() {
    cleanup();
}

void MagneticField::cleanup() {
    run_frq_mon_ = false;
    xcom_.send_message(xcom_.get_xcomcmd_clearall());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    xcom_.set_rc(xcom::XComState::ReturnCode::Ok);
    xcom_.forced_exit();
}

void MagneticField::init() {

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

void MagneticField::handle_response(XCOMResp response) {
    if(response == XCOMResp::OK) {
        invalid_channel_ = false;

        if(!init_done_) {

            RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(),
                        ("connected to iNAT on channel " + std::to_string(channel_)).c_str());

            magfield_msg_.header.frame_id = "magnetic_field";

            auto cmd_clearall = xcom_.get_xcomcmd_clearall();
            xcom_.send_message(cmd_clearall);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            xcom_.send_message(xcom_.get_generic_param<XCOMParEKF_MAGATTAID>());

            uint16_t div1 = calculateDividerForRate(topic_freq_, maintiming_, prescaler_);
            msg_MAGDATA_frq_ = calculateRateForDivider(div1, maintiming_, prescaler_);
            auto cmd_add_magdata = xcom_.get_xcomcmd_enablelog(XCOM_MSGID_MAGDATA, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC, div1);
            xcom_.send_message(cmd_add_magdata);

            setup_freq_ = calculateRateForDivider(div1, maintiming_, prescaler_);
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

            pub_ = node_->create_publisher<MagneticFieldMsg>(topic_name_, qos_);
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

void MagneticField::handle_xcom_msg(const XCOMmsg_MAGDATA &msg) {
    msg_MAGDATA_age_ = 0;
    updateMAGDATA(msg);
}

void MagneticField::handle_xcom_param(const XCOMParEKF_MAGATTAID& param) {
    par_MAGATTAID_age_ = 0;
    setParData(param);
}

void MagneticField::activate() {
    active_ = true;
    RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(), "+ configured and activated");

    t_pub_ = std::chrono::high_resolution_clock::now();
    t_pub_upd_ = std::chrono::high_resolution_clock::now();
    std::thread th_mon(&MagneticField::frq_mon, this);
    th_mon.detach();
}

uint16_t MagneticField::getSetupFreq() {
    return setup_freq_;
}

bool MagneticField::success() {
    return success_;
}

void MagneticField::setParData(const XCOMParEKF_MAGATTAID& param) {

    magfield_msg_.magnetic_field_covariance[0] = param.mag_field_stddev[0] * 1e-7;
    magfield_msg_.magnetic_field_covariance[4] = param.mag_field_stddev[1] * 1e-7;
    magfield_msg_.magnetic_field_covariance[8] = param.mag_field_stddev[2] * 1e-7;

    parDataIsSet_ = true;
}

void MagneticField::updateMAGDATA(const XCOMmsg_MAGDATA& msg) {

    if(!parDataIsSet_) {
        return;
    }

    magfield_msg_.magnetic_field.x = msg.field[0] * magfield_scalefactor_;
    magfield_msg_.magnetic_field.y = msg.field[1] * magfield_scalefactor_;
    magfield_msg_.magnetic_field.z = msg.field[2] * magfield_scalefactor_;

    gps_time_ = UpdateGPSTime(msg.header, leap_seconds_);
    publish();
}

void MagneticField::publish() {

    magfield_msg_.header.stamp = (timestamp_mode_ == Config::TimestampMode::GPS) ? gps_time_ : node_->now();

    if(active_) {
        pub_->publish(magfield_msg_);

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
    }
}

void MagneticField::frq_mon() {

    bool unknown_state_printed = false;
    bool par_MAGATTAID_ok = true;
    bool msg_MAGDATA_ok = true;

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

        if(msg_MAGDATA_ok && (msg_MAGDATA_age_ > xcom_age_max_)) {
            msg_MAGDATA_ok = false;
            RCLCPP_ERROR(node_->get_logger(), "[%s] %s (%s)", topic_name_.c_str(), "missing iNAT data", "XCOMmsg_MAGDATA");
        } else if(!msg_MAGDATA_ok && !(msg_MAGDATA_age_ > xcom_age_max_)) {
            msg_MAGDATA_ok = true;
            RCLCPP_INFO(node_->get_logger(), "[%s] %s (%s)", topic_name_.c_str(), "missing iNAT data", "XCOMmsg_MAGDATA");
        }
        if(par_MAGATTAID_ok && (par_MAGATTAID_age_ > xcom_age_max_)) {
            par_MAGATTAID_ok = false;
            RCLCPP_ERROR(node_->get_logger(), "[%s] %s (%s)", topic_name_.c_str(), "missing iNAT data", "XCOMParEKF_MAGATTAID");
        } else if(!par_MAGATTAID_ok && !(par_MAGATTAID_age_ > xcom_age_max_)) {
            par_MAGATTAID_ok = true;
            RCLCPP_INFO(node_->get_logger(), "[%s] %s (%s)", topic_name_.c_str(), "receiving iNAT data", "XCOMParEKF_MAGATTAID");
        }

        if(msg_MAGDATA_c_ > (mon_frq_ / msg_MAGDATA_frq_)) {
            if(msg_MAGDATA_age_ == age_max_val_) {
                msg_MAGDATA_age_ = xcom_age_max_ + 1;
            } else {
                msg_MAGDATA_age_++;
            }
            msg_MAGDATA_c_ = 0;
        }
        if(!parDataIsSet_) {
            if(par_MAGATTAID_c_ > (mon_frq_ / par_frq_)) {
                if(par_MAGATTAID_age_ == age_max_val_) {
                    par_MAGATTAID_age_ = xcom_age_max_ + 1;
                } else {
                    par_MAGATTAID_age_++;
                }
                par_MAGATTAID_c_ = 0;
            }
        }

        par_MAGATTAID_c_++;
        msg_MAGDATA_c_++;

        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::this_thread::sleep_for(std::chrono::milliseconds(mon_t_gap_));
    }
}
