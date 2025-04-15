#include "ixcom_driver_lc/modules/imu.hpp"
#include "ixcom_driver_lc/ixcom_driver_conf.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <ixcom_driver_lc/modules/utility.hpp>
#include <thread>

IMU::IMU(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
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
    XComParameters_Imu(&state),
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
    std::thread th_run(&IMU::init, this);
    th_run.detach();
}

IMU::~IMU() {
    cleanup();
}

void IMU::cleanup() {
    run_frq_mon_ = false;
    xcom_.send_message(xcom_.get_xcomcmd_clearall());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    xcom_.set_rc(xcom::XComState::ReturnCode::Ok);
    xcom_.forced_exit();
}

void IMU::init() {
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

void IMU::handle_response(XCOMResp response) noexcept {
    if(response == XCOMResp::OK) {
        invalid_channel_ = false;

        if(!init_done_) {

            RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(),
                        ("connected to iNAT on channel " + std::to_string(channel_)).c_str());

            imu_msg_.header.frame_id = "inat_output_frame";

            auto cmd_clearall = xcom_.get_xcomcmd_clearall();
            xcom_.send_message(cmd_clearall);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            xcom_.send_message(xcom_.get_generic_param<XCOMParEKF_IMUCONFIG2>());

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
            // std::cout << "set time delta: " << std::to_string(time_delta_) << std::endl;

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

            pub_ = node_->create_publisher<ImuMsg>(topic_name_, qos_);
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

void IMU::handle_xcom_msg(const XCOMmsg_INSSOL &msg) noexcept {

    // t_inssol_ = t_inssol_upd_;
    // t_inssol_upd_ = std::chrono::high_resolution_clock::now();
    // auto d = std::chrono::duration_cast<std::chrono::microseconds>(t_inssol_upd_ - t_inssol_).count();
    // std::cout << "duration inssol: " << std::to_string(d) << std::endl;

    msg_INSSOL_age_ = 0;
    updateINSSOL(msg);
}

void IMU::handle_xcom_msg(const XCOMmsg_IMUCORR &msg) noexcept {

    // t_imucorr_ = t_imucorr_upd_;
    // t_imucorr_upd_ = std::chrono::high_resolution_clock::now();
    // auto d = std::chrono::duration_cast<std::chrono::microseconds>(t_imucorr_upd_ - t_imucorr_).count();
    // std::cout << "duration imucorr: " << std::to_string(d) << std::endl;

    msg_IMUCORR_age_ = 0;
    updateIMUCORR(msg);
}

void IMU::handle_xcom_msg(const XCOMmsg_EKFSTDDEV &msg) noexcept {

    // t_ekfstddev_ = t_ekfstddev_upd_;
    // t_ekfstddev_upd_ = std::chrono::high_resolution_clock::now();
    // auto d = std::chrono::duration_cast<std::chrono::microseconds>(t_ekfstddev_upd_ - t_ekfstddev_).count();
    // std::cout << "duration ekfstddev: " << std::to_string(d) << std::endl;

    msg_EKFSTDDEV_age_ = 0;
    updateEKFSTDDEV(msg);
}

void IMU::handle_xcom_param(const XCOMParEKF_IMUCONFIG2& param) {
    par_IMUCONFIG2_age_ = 0;
    setParData(param);
}

void IMU::activate() {
    active_ = true;
    RCLCPP_INFO(node_->get_logger(), "[%s] %s", topic_name_.c_str(), "+ configured and activated");

    // t_inssol_ = std::chrono::high_resolution_clock::now();
    // t_imucorr_ = std::chrono::high_resolution_clock::now();
    // t_ekfstddev_ = std::chrono::high_resolution_clock::now();

    t_pub_ = std::chrono::high_resolution_clock::now();
    t_pub_upd_ = std::chrono::high_resolution_clock::now();
    std::thread th_mon(&IMU::frq_mon, this);
    th_mon.detach();
}

uint16_t IMU::getSetupFreq() {
    return setup_freq_;
}

bool IMU::success() {
    return success_;
}

void IMU::setParData(const XCOMParEKF_IMUCONFIG2& param) {

    imu_msg_.angular_velocity_covariance[0] = param.acc.root_noise_psd[0] * param.acc.root_noise_psd[0] * maintiming_;
    imu_msg_.angular_velocity_covariance[4] = param.acc.root_noise_psd[1] * param.acc.root_noise_psd[1] * maintiming_;
    imu_msg_.angular_velocity_covariance[8] = param.acc.root_noise_psd[2] * param.acc.root_noise_psd[2] * maintiming_;

    imu_msg_.linear_acceleration_covariance[0] = param.gyro.root_noise_psd[0] * param.gyro.root_noise_psd[0] * maintiming_;
    imu_msg_.linear_acceleration_covariance[4] = param.gyro.root_noise_psd[1] * param.gyro.root_noise_psd[1] * maintiming_;
    imu_msg_.linear_acceleration_covariance[8] = param.gyro.root_noise_psd[2] * param.gyro.root_noise_psd[2] * maintiming_;

    parDataIsSet_ = true;
}

void IMU::updateINSSOL(const XCOMmsg_INSSOL &msg) {

    tf2::Quaternion q;
    q.setRPY(msg.rpy[0], msg.rpy[1], msg.rpy[2]);
    imu_msg_.orientation = tf2::toMsg(q);

    imu_msg_.linear_acceleration.x = msg.accel[0];
    imu_msg_.linear_acceleration.y = msg.accel[1];
    imu_msg_.linear_acceleration.z = msg.accel[2];

    // std::cout << "data selection: " << std::to_string(msg.data_selection) << std::endl;

    insSolDataIsSet_ = true;
}

void IMU::updateEKFSTDDEV(const XCOMmsg_EKFSTDDEV &msg) {

    if(msg.footer.global_status.bits.ALIGNMODE == XCOMGlobalStatusAlignmode::AlignmentComplete) {
        imu_msg_.orientation_covariance[0] = msg.tilt[0] * msg.tilt[0];
    } else {
        imu_msg_.orientation_covariance[0] = -1.0;
        imu_msg_.orientation_covariance[4] = msg.tilt[1] * msg.tilt[1];
        imu_msg_.orientation_covariance[8] = msg.tilt[2] * msg.tilt[2];
    }

    ekfDataIsSet_ = true;
}

void IMU::updateIMUCORR(const XCOMmsg_IMUCORR &msg) {

    imu_msg_.angular_velocity.x = msg.omg[0];
    imu_msg_.angular_velocity.y = msg.omg[1];
    imu_msg_.angular_velocity.z = msg.omg[2];

    imuCorrDataIsSet_ = true;

    gps_time_ = UpdateGPSTime(msg.header, leap_seconds_);
    publish();
}


void IMU::publish() {

    // pub_age_ = 0;
    
    if(!(parDataIsSet_ && insSolDataIsSet_ && ekfDataIsSet_ && imuCorrDataIsSet_)) {
        return;
    }

    if(active_) {

        // auto t1 = std::chrono::high_resolution_clock::now();

        imu_msg_.header.stamp = (timestamp_mode_ == Config::TimestampMode::GPS) ? gps_time_ : node_->now();

        pub_->publish(imu_msg_);

        t_pub_ = t_pub_upd_;
        t_pub_upd_ = std::chrono::high_resolution_clock::now();
        duration_ = std::chrono::duration_cast<std::chrono::microseconds>(t_pub_upd_ - t_pub_).count();
        // std::cout << "duration: " << std::to_string(duration_) << std::endl;

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

        // std::cout << "imu publish: " << std::to_string(d) << std::endl;
    }
}

void IMU::frq_mon() {

    bool unknown_state_printed = false;
    bool par_IMUCONFIG2_ok = true;
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

        if(par_IMUCONFIG2_ok && (par_IMUCONFIG2_age_ > xcom_age_max_)) {
            par_IMUCONFIG2_ok = false;
            RCLCPP_ERROR(node_->get_logger(), "[%s] %s (%s)", topic_name_.c_str(), "missing iNAT data", "XCOMParEKF_IMUCONFIG2");
        } else if(!par_IMUCONFIG2_ok && !(par_IMUCONFIG2_age_ > xcom_age_max_)) {
            par_IMUCONFIG2_ok = true;
            RCLCPP_INFO(node_->get_logger(), "[%s] %s (%s)", topic_name_.c_str(), "receiving iNAT data", "XCOMParEKF_IMUCONFIG2");
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

        if(!parDataIsSet_) {
            if(par_IMUCONFIG2_c_ > (mon_frq_ / par_frq_)) {
                if(par_IMUCONFIG2_age_ == age_max_val_) {
                    par_IMUCONFIG2_age_ = xcom_age_max_ + 1;
                } else {
                    par_IMUCONFIG2_age_++;
                }
                par_IMUCONFIG2_c_ = 0;
            }
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

        par_IMUCONFIG2_c_++;
        msg_INSSOL_c_++;
        msg_IMUCORR_c_++;
        msg_EKFSTDDEV_c_++;

        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::this_thread::sleep_for(std::chrono::milliseconds(mon_t_gap_));
    }
}
