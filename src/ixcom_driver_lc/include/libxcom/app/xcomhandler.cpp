#include "xcomhandler.hpp"
#include <string>
#include <chrono>
#include <thread>


XcomHandler::XcomHandler(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                         xcom::XComState &state,
                         bool serial_ignore,
                         uint8_t serial_port,
                         uint32_t serial_baud,
                         bool serial_enable,
                         int32_t leap_seconds)
    : MessageHandler(&state),
      XComParameters_XcomHandler(&state),
      xcom::CommandHandler(&state),
      xcom::ResponseHandler(&state),
      node_(node),
      _xcom(state),
      leap_seconds_(leap_seconds),
      serial_ignore_(serial_ignore),
      serial_port_(serial_port),
      serial_baud_(serial_baud),
      serial_enable_(serial_enable){}

void XcomHandler::handle_command(uint16_t cmd_id, std::size_t frame_len, uint8_t *frame) noexcept {
//    std::cout << "Command received" << "\n";
}

void XcomHandler::handle_response(XCOMResp response) noexcept {

    if(response == XCOMResp::OK) {
        invalid_channel_ = false;
        complete_status_ = static_cast<CompleteStatus>(static_cast<uint16_t>(complete_status_)
                                                       | (static_cast<uint16_t>(CompleteStatus::RESPONSES)));

        if(!_init_done) {

            // set_sysstat_mode();

            auto cmd_clearall = _xcom.get_xcomcmd_clearall();
            _xcom.send_message(cmd_clearall);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            _xcom.send_message(_xcom.get_generic_param<XCOMParDAT_SYSSTAT>());
            _xcom.send_message(_xcom.get_generic_param<XCOMParSYS_FWVERSION>());
            _xcom.send_message(_xcom.get_generic_param<XCOMParSYS_MAINTIMING>());
            _xcom.send_message(_xcom.get_generic_param<XCOMParSYS_PRESCALER>());
            _xcom.send_message(_xcom.get_generic_param<XCOMParDAT_VEL>());
            _xcom.send_message(_xcom.get_generic_param<XCOMParDAT_IMU>());
            auto cmd_add_gnsssol = _xcom.get_xcomcmd_enablelog(XCOM_MSGID_GNSSSOL, XComLogTrigger::XCOM_CMDLOG_TRIG_EVENT, 1);
            _xcom.send_message(cmd_add_gnsssol);
        }
        _init_done = true;
    } else {
        complete_status_ =
                static_cast<CompleteStatus>(static_cast<uint16_t>(complete_status_)
                                                       &~ (static_cast<uint16_t>(CompleteStatus::RESPONSES)));
        if(response == XCOMResp::INVALIDCHANNEL) {
            invalid_channel_ = true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "%s: %d", "invalid response received", static_cast<int>(response));
        }
        _xcom.set_rc(xcom::XComState::ReturnCode::InvalidResponse);
        _xcom.forced_exit();
    }

    if(complete_status_ == CompleteStatus::FULL) {
        _xcom.send_message(_xcom.get_xcomcmd_clearall());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        _xcom.set_rc(xcom::XComState::ReturnCode::Ok);
        _xcom.forced_exit();
    }
}

void XcomHandler::handle_xcom_param(const XCOMParSYS_MAINTIMING& param) {
    maintiming_ = param.main_timing;
    complete_status_ =
            static_cast<CompleteStatus>(static_cast<uint16_t>(complete_status_)
                                          | static_cast<uint16_t>(CompleteStatus::MAINTIMING));
    RCLCPP_INFO(node_->get_logger(), "%s", ("got maintiming: " + std::to_string(maintiming_)).c_str());

    add_sysstat();
}

void XcomHandler::handle_xcom_param(const XCOMParSYS_PRESCALER& param) {
    prescaler_ = param.prescaler;
    complete_status_
            = static_cast<CompleteStatus>(static_cast<uint16_t>(complete_status_)
                                          | static_cast<uint16_t>(CompleteStatus::PRESCALER));
    RCLCPP_INFO(node_->get_logger(), "%s", ("got prescaler: " + std::to_string(prescaler_)).c_str());

    add_sysstat();
}

void XcomHandler::handle_xcom_param(const XCOMParSYS_FWVERSION& param) {

    int l = sizeof(param.payload) / sizeof(param.payload[0]);

    std::string s[3];

    bool p1found = false;
    bool p2found = false;

    for(int i = 0; i < l; i++) {
        if(param.payload[i] == '.') {
            if(p1found == false) {
                p1found = true;
            } else {
                p2found = true;
            }
            continue;
        }
        if(!p1found) {
            s[0].push_back(param.payload[i]);
        } else if(p1found && ! p2found) {
            s[1].push_back(param.payload[i]);
        } else {
            s[2].push_back(param.payload[i]);
        }
    }

    bool err = false;
    bool warn = false;
    try {fwv_[0] = std::stoi(s[0]);} catch(std::exception){err = true;}
    try {fwv_[1] = std::stoi(s[1]);} catch(std::exception){warn = true;}
    try {fwv_[2] = std::stoi(s[2]);} catch(std::exception){warn = true;}

    RCLCPP_INFO(node_->get_logger(), "%s", ("iNAT version: "
                + std::to_string(fwv_[0]) + "."
                + std::to_string(fwv_[1]) + "."
                + std::to_string(fwv_[2])).c_str());

    if(err) {
        RCLCPP_ERROR(node_->get_logger(), "%s", "could not get major firmware version");
    }
    if(warn) {
        RCLCPP_WARN(node_->get_logger(), "%s", "could not get minor or patch firmware version");
    }

    if(!err) {
        complete_status_ =
                static_cast<CompleteStatus>(static_cast<uint16_t>(complete_status_)
                                            | static_cast<uint16_t>(CompleteStatus::FWV));
    }

    if(!err) {
        send_serial_config();
    }
}

void XcomHandler::handle_xcom_param(const XCOMParDAT_SYSSTAT& param) {
    // auto t = xcom::XComState::get_timestamp(param);
    // std::cout << "XCOMParDAT_SYSSTAT received: tow:" << t << ", sysstatmode: " << param.mode << std::endl;
    if(sysstat_mode_is_set_) {
        if(param.mode == (current_sysstat_mode_ |
                          PARDAT_SYSSTAT_MASK_IMU |
                          PARDAT_SYSSTAT_MASK_GNSS |
                          PARDAT_SYSSTAT_MASK_REMALIGNTIME)) {
            complete_status_ = static_cast<CompleteStatus>(static_cast<uint16_t>(complete_status_)
                                                           | static_cast<uint16_t>(CompleteStatus::SYSSTAT));
        } else {
            RCLCPP_ERROR(node_->get_logger(), "%s", "could not set SysStat mode");
        }
    } else {
        sysstat_mode_is_set_ = true;
        current_sysstat_mode_ = param.mode;
        set_sysstat_mode();
    }
}

// SERIAL interface Linux Firmware
void XcomHandler::handle_xcom_param(const XCOMParXCOM_SERIALPORT& param) {
    if(param.port == serial_port_
            && param.baud_rate == serial_baud_
            && param.options == serial_enable_) {
        complete_status_ =
                static_cast<CompleteStatus>(static_cast<uint16_t>(complete_status_)
                                            | static_cast<uint16_t>(CompleteStatus::SERIAL));
        RCLCPP_INFO(node_->get_logger(), "%s", "serial configuration done");
    }
}

// SERIAL interface QNX Firmware
void XcomHandler::handle_xcom_param(const XCOMParXCOM_INTERFACE& param) {
    if(param.port == serial_port_
            && param.baud == serial_baud_
            && param.port_mode == serial_enable_) {
        complete_status_ =
                static_cast<CompleteStatus>(static_cast<uint16_t>(complete_status_)
                                            | static_cast<uint16_t>(CompleteStatus::SERIAL));
        RCLCPP_INFO(node_->get_logger(), "%s", "serial configuration done");
    }
}

void XcomHandler::handle_xcom_param(const XCOMParDAT_VEL& param) {

    vel_mode_ = static_cast<XCOM_PARDAT_VEL_Mode>(param.mode);

    if(vel_mode_ != XCOM_PARDAT_VEL_Mode::XCOM_PARDAT_VEL_BODY) {
        RCLCPP_INFO(node_->get_logger(), "%s", "setting INS velocity frame type to BODY");
        auto p_set = _xcom.get_generic_param<XCOMParDAT_VEL>();
        p_set.param_header.param_id = XCOMPAR_PARDAT_VEL;
        p_set.param_header.is_request = 0;
        p_set.mode = XCOM_PARDAT_VEL_Mode::XCOM_PARDAT_VEL_BODY;
        _xcom.send_message(p_set);

        // doublecheck
        auto p_get = _xcom.get_generic_param<XCOMParDAT_VEL>();
        p_get.param_header.param_id = XCOMPAR_PARDAT_VEL;
        _xcom.send_message(p_get);
    } else {
        RCLCPP_INFO(node_->get_logger(), "%s", "INS velocity frame type is set to BODY");
    }

    complete_status_ =
            static_cast<CompleteStatus>(static_cast<uint16_t>(complete_status_)
                                        | static_cast<uint16_t>(CompleteStatus::VELMODE));
}

void XcomHandler::handle_xcom_param(const XCOMParDAT_IMU& param) {
    imu_mode_ = static_cast<XCOM_PARDAT_IMU_Mode>(param.mode);

    if(imu_mode_ != XCOM_PARDAT_IMU_Mode::XCOM_PARDAT_IMU_IMURAW) {
        RCLCPP_INFO(node_->get_logger(), "%s", "setting IMU output mode to IMURAW");
        auto p_set = _xcom.get_generic_param<XCOMParDAT_IMU>();
        p_set.param_header.param_id = XCOMPAR_PARDAT_IMU;
        p_set.param_header.is_request = 0;
        p_set.mode = XCOM_PARDAT_IMU_Mode::XCOM_PARDAT_IMU_IMURAW;
        _xcom.send_message(p_set);

        // doublecheck
        auto p_get = _xcom.get_generic_param<XCOMParDAT_IMU>();
        p_get.param_header.param_id = XCOMPAR_PARDAT_IMU;
        _xcom.send_message(p_get);
    } else {
        RCLCPP_INFO(node_->get_logger(), "%s", "IMU output mode is set to IMURAW");
    }

    complete_status_ =
            static_cast<CompleteStatus>(static_cast<uint16_t>(complete_status_)
                                        | static_cast<uint16_t>(CompleteStatus::IMUMODE));
}

void XcomHandler::handle_xcom_msg(const XCOMmsg_GNSSSOL &msg) noexcept {

    // needed for leap seconds if not configured in config file
    if(msg.posvel_type != GnssPosVelType::NONE) {
        if(leap_seconds_ == 0) {
            auto gnsstime = _xcom.get_xcomcmd_enablelog(XCOM_MSGID_GNSSTIME, XComLogTrigger::XCOM_CMDLOG_TRIG_POLLED, 1);
            _xcom.send_message(gnsstime);
        } else {
            if((static_cast<uint16_t>(complete_status_) & static_cast<uint16_t>(CompleteStatus::LEAPSECONDS)) != static_cast<uint16_t>(CompleteStatus::LEAPSECONDS)) {
                RCLCPP_INFO(node_->get_logger(), "%s", ("configured leap seconds: " + std::to_string(leap_seconds_)).c_str());
                complete_status_
                        = static_cast<CompleteStatus>(static_cast<uint16_t>(complete_status_)
                                                      | static_cast<uint16_t>(CompleteStatus::LEAPSECONDS));
            }
        }
    } else {
        RCLCPP_INFO(node_->get_logger(), "%s", "waiting for GNSS solution...");
    }
}

void XcomHandler::handle_xcom_msg(const XCOMmsg_GNSSTIME &msg) noexcept {
    leap_seconds_ = std::abs(round(msg.utc_offset));
    RCLCPP_INFO(node_->get_logger(), "%s", ("got leap seconds from iNAT: " + std::to_string(leap_seconds_)).c_str());
    complete_status_
            = static_cast<CompleteStatus>(static_cast<uint16_t>(complete_status_)
                                          | static_cast<uint16_t>(CompleteStatus::LEAPSECONDS));
}

void XcomHandler::handle_xcom_msg(const XCOMmsg_SYSSTAT& msg) noexcept {
    auto data = xcom::XComState::process_msg_sysstat(_xcom.get_payload(), _xcom.get_payload_length());
    if(data.has_value()) {
        if(data.value().status_remaining_aligntime.has_value()) {
            int val = data.value().status_remaining_aligntime.value();
            if(val > 0) {
                RCLCPP_INFO(node_->get_logger(), "%s %d", "waiting for alignment ... ", val);
            } else {
                RCLCPP_INFO(node_->get_logger(), "%s", "alignment completed");
                complete_status_
                        = static_cast<CompleteStatus>(static_cast<uint16_t>(complete_status_)
                                                      | static_cast<uint16_t>(CompleteStatus::ALIGNMENT));
            }
        }
    }

    if(complete_status_ == CompleteStatus::FULL) {
        _xcom.send_message(_xcom.get_xcomcmd_clearall());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        _xcom.set_rc(xcom::XComState::ReturnCode::Ok);
        _xcom.forced_exit();
    }
}

void XcomHandler::add_sysstat() {
    if(sysstat_added_) {
        return;
    }
    bool got_maintiming = (static_cast<uint16_t>(complete_status_) & static_cast<uint16_t>(CompleteStatus::MAINTIMING))
            == static_cast<uint16_t>(CompleteStatus::MAINTIMING);
    bool got_prescaler = (static_cast<uint16_t>(complete_status_) & static_cast<uint16_t>(CompleteStatus::PRESCALER))
            == static_cast<uint16_t>(CompleteStatus::PRESCALER);
    if(got_maintiming && got_prescaler) {
        if((maintiming_ > 0) && (prescaler_ > 0)) {
            auto cmd_add_sysstat = _xcom.get_xcomcmd_enablelog(XCOM_MSGID_SYSSTAT, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC, (maintiming_ / prescaler_));  // 1 Hz
            _xcom.send_message(cmd_add_sysstat);
            sysstat_added_ = true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "%s", ("maintiming (" + std::to_string(maintiming_)
                     + ") and prescaler (" + std::to_string(prescaler_)
                     + ") both must not be zero").c_str());
        }
    }
}

void XcomHandler::send_serial_config() {
    if(!serial_ignore_) {
        RCLCPP_INFO(node_->get_logger(), "%s", "setting serial configuration");
        if(fwv_[0] < 3) {
            // linux firmware
            auto p = _xcom.get_generic_param<XCOMParXCOM_SERIALPORT>();
            p.param_header.param_id = XCOMPAR_PARXCOM_SERIALPORT;
            p.param_header.is_request = 0;
            p.baud_rate = serial_baud_;
            p.port = serial_port_;
            p.options = serial_enable_;  // 1: enable
            _xcom.send_message(p);
            p.param_header.is_request = 1;  // request the parameter for doublecheck
            _xcom.send_message(p);
        } else {
            // qnx firmware
            auto p = _xcom.get_generic_param<XCOMParXCOM_INTERFACE>();
            p.param_header.param_id = XCOMPAR_PARXCOM_INTERFACE;
            p.param_header.is_request = 0;
            p.baud = serial_baud_;
            p.port = serial_port_;
            p.port_mode = serial_enable_;  // 1: enable
            _xcom.send_message(p);
            p.param_header.is_request = 1;   // request the parameter for doublecheck
            _xcom.send_message(p);
        }
    } else {
        RCLCPP_INFO(node_->get_logger(), "%s", "serial configuration ignored");
        complete_status_ =
                static_cast<CompleteStatus>(static_cast<uint16_t>(complete_status_)
                                            | static_cast<uint16_t>(CompleteStatus::SERIAL));
    }
}

bool XcomHandler::invalidChannel() {
    return invalid_channel_;
}

// bool XcomHandler::connected() {
//     return connected_;
// }

void XcomHandler::set_sysstat_mode() {
    auto p_set = _xcom.get_generic_param<XCOMParDAT_SYSSTAT>();
    p_set.param_header.param_id = XCOMPAR_PARDAT_SYSSTAT;
    p_set.param_header.is_request = 0;
    p_set.mode = current_sysstat_mode_ |
             PARDAT_SYSSTAT_MASK_IMU |
             PARDAT_SYSSTAT_MASK_GNSS |
             PARDAT_SYSSTAT_MASK_REMALIGNTIME;

    _xcom.send_message(p_set);

    // doublecheck
    auto p_get = _xcom.get_generic_param<XCOMParDAT_SYSSTAT>();
    p_get.param_header.param_id = XCOMPAR_PARDAT_SYSSTAT;
    _xcom.send_message(p_get);

    // complete_status_ = static_cast<CompleteStatus>(static_cast<uint16_t>(complete_status_)
    //                                                | static_cast<uint16_t>(CompleteStatus::SYSSTAT));
}

uint16_t XcomHandler::getMaintiming() {
    return maintiming_;
}

uint16_t XcomHandler::getPrescaler() {
    return prescaler_;
}

int32_t XcomHandler::getLeapSeconds() {
    return leap_seconds_;
}

bool XcomHandler::complete() {
    if(complete_status_ == CompleteStatus::FULL) {
        return true;
    }
    return false;
}
