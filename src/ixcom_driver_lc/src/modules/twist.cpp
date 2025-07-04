#include "ixcom_driver_lc/modules/twist.hpp"
#include "ixcom_driver_lc/ixcom_driver_conf.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <ixcom_driver_lc/modules/utility.hpp>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ixcom_driver_lc/modules/utility.hpp>

TwistStamped::TwistStamped(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
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
    qos_(qos),
    tf2_(tf2) {
    std::thread th(&TwistStamped::init, this);
    th.detach();
}

TwistStamped::~TwistStamped() {
    cleanup();
}

void TwistStamped::cleanup() {
    run_frq_mon_ = false;
    xcom_.send_message(xcom_.get_xcomcmd_clearall());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    xcom_.set_rc(xcom::XComState::ReturnCode::Ok);
    xcom_.forced_exit();
}

void TwistStamped::init() {

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
//        RCLCPP_ERROR(node_->get_logger(), "%s", "unable to initialize tcp_writer for imu");
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

void TwistStamped::handle_response(XCOMResp response) noexcept {
    if(response == XCOMResp::OK) {
        invalid_channel_ = false;

        if(!init_done_) {

            RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(),
                        ("connected to iNAT on channel " + std::to_string(channel_)).c_str());

            twiststamped_msg_.header.frame_id = "inat_output_frame";

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

            uint16_t div3 = calculateDividerForRate(topic_freq_, maintiming_, prescaler_);
            msg_INSDCM_frq_ = calculateRateForDivider(div3, maintiming_, prescaler_);
            auto cmd_add_insdcm = xcom_.get_xcomcmd_enablelog(XCOM_MSGID_INSDCM, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC, div3);
            xcom_.send_message(cmd_add_insdcm);

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

            pub_ = node_->create_publisher<TwistStampedMsg>(topic_name_, qos_);
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

void TwistStamped::handle_xcom_msg(const XCOMmsg_INSSOL &msg) noexcept {
    msg_INSSOL_age_ = 0;
    updateINSSOL(msg);
}

void TwistStamped::handle_xcom_msg(const XCOMmsg_IMUCORR &msg) noexcept {
    msg_IMUCORR_age_ = 0;
    updateIMUCORR(msg);
}

void TwistStamped::handle_xcom_msg(const XCOMmsg_INSDCM &msg) noexcept {
    msg_INSDCM_age_ = 0;
    updateINSDCM(msg);
}

void TwistStamped::activate() {
    active_ = true;
    RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(), "+ configured and activated");

    t_pub_ = std::chrono::high_resolution_clock::now();
    t_pub_upd_ = std::chrono::high_resolution_clock::now();
    std::thread th_mon(&TwistStamped::frq_mon, this);
    th_mon.detach();
}

uint16_t TwistStamped::getSetupFreq() {
    return setup_freq_;
}

bool TwistStamped::success() {
    return success_;
}

void TwistStamped::updateINSSOL(const XCOMmsg_INSSOL& msg)
{
    vel_x_ = msg.vel[0];
    vel_y_ = msg.vel[1];
    vel_z_ = msg.vel[2];

    insSolDataIsSet_ = true;
}

void TwistStamped::updateINSDCM(const XCOMmsg_INSDCM& msg)
{
    std::memcpy(coefficients_, msg.c_nb, sizeof(float) * 9);

    insDcmDataIsSet_ = true;
}

void TwistStamped::updateIMUCORR(const XCOMmsg_IMUCORR& msg)
{
    twiststamped_msg_.twist.angular.x = msg.omg[0];
    twiststamped_msg_.twist.angular.y = msg.omg[1];
    twiststamped_msg_.twist.angular.z = msg.omg[2];

    imuCorrDataIsSet_ = true;

    gps_time_ = UpdateGPSTime(msg.header, leap_seconds_);
    publish();
}

void TwistStamped::publish()
{
    // std::cout << "publishing TwistStamped...   " << std::endl;

    if(!(insSolDataIsSet_ && insDcmDataIsSet_ && imuCorrDataIsSet_)) {
        return;
    }

    // Do some inline matrix-vector multiplication
    // Remember that the coefficient matrix is put into memory column-by-column
    // which is why the first row is addressed by indices 0, 3, and 6
    twiststamped_msg_.twist.linear.x =
        coefficients_[0] * vel_x_ + coefficients_[3] * vel_y_ + coefficients_[6] * vel_z_;

    twiststamped_msg_.twist.linear.y =
        coefficients_[1] * vel_x_ + coefficients_[4] * vel_y_ + coefficients_[7] * vel_z_;

    twiststamped_msg_.twist.linear.x =
        coefficients_[2] * vel_x_ + coefficients_[5] * vel_y_ + coefficients_[8] * vel_z_;

    twiststamped_msg_.header.stamp = (timestamp_mode_ == Config::TimestampMode::GPS) ? gps_time_ : node_->now();

    if(active_) {
        pub_->publish(twiststamped_msg_);

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
            RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(), "† a listener leaved");
            num_of_subscribers_ = n;
            RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(), ("current number of subscriptions: " + std::to_string(num_of_subscribers_)).c_str());
        }
    }
}

void TwistStamped::frq_mon() {

    bool unknown_state_printed = false;
    bool msg_INSSOL_ok = true;
    bool msg_IMUCORR_ok = true;
    bool msg_INSDCM_ok = true;

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
        if(msg_INSDCM_ok && (msg_INSDCM_age_ > xcom_age_max_)) {
            msg_INSDCM_ok = false;
            RCLCPP_ERROR(node_->get_logger(), "[%s] %s (%s)", topic_name_.c_str(), "missing iNAT data", "XCOMmsg_INSDCM");
        } else if(!msg_INSDCM_ok && !(msg_INSDCM_age_ > xcom_age_max_)) {
            msg_INSDCM_ok = true;
            RCLCPP_INFO(node_->get_logger(), "[%s] %s (%s)", topic_name_.c_str(), "receiving iNAT data", "XCOMmsg_INSDCM");
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
        if(msg_INSDCM_c_ > (mon_frq_ / msg_INSDCM_frq_)) {
            if(msg_INSDCM_age_ == age_max_val_) {
                msg_INSDCM_age_ = xcom_age_max_ + 1;
            } else {
                msg_INSDCM_age_++;
            }
            msg_INSDCM_c_ = 0;
        }

        msg_INSSOL_c_++;
        msg_IMUCORR_c_++;
        msg_INSDCM_c_++;

        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::this_thread::sleep_for(std::chrono::milliseconds(mon_t_gap_));
    }
}
