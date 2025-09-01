#include "ixcom_driver_lc/modules/srv_extaid.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ixcom_driver_lc/modules/utility.hpp>
#include <thread>
#include <chrono>
#include <string>

using std::placeholders::_1;
using std::placeholders::_2;

SrvExtAid::SrvExtAid(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                     xcom::XComState &state,
                     const std::string &ip_address,
                     int32_t ip_port,
                     int32_t leap_seconds) :
    xcom::ResponseHandler(&state),
    xcom::CommandHandler(&state),
    xcom::ParameterHandler<XCOMParIMU_MISALIGN>(&state),
    node_(node),
    xcom_(state),
    ip_address_(ip_address),
    ip_port_(ip_port),
    leap_seconds_(leap_seconds) {
    extaidItems_.pos_llh = {false, false, initFrameCounter()};
    extaidItems_.pos_ecef = {false, false, initFrameCounter()};
    extaidItems_.pos_utm = {false, false, initFrameCounter()};
    extaidItems_.pos_mgrs = {false, false, initFrameCounter()};
    extaidItems_.hdg = {false, false, initFrameCounter()};
    extaidItems_.vel = {false, false, initFrameCounter()};
    extaidItems_.vel_body = {false, false, initFrameCounter()};
    extaidItems_.height = {false, false, initFrameCounter()};
    std::thread th(&SrvExtAid::init, this);
    th.detach();
}

SrvExtAid::~SrvExtAid() {
    cleanup();
}

void SrvExtAid::cleanup() {
    // xcom_.send_message(xcom_.get_xcomcmd_clearall());
    // std::this_thread::sleep_for(std::chrono::milliseconds(200));
    xcom_.set_rc(xcom::XComState::ReturnCode::Ok);
    xcom_.forced_exit();
}

void SrvExtAid::init() {

    // required for the service to be available
    // srv_ = node_->create_service<ExtAidMsg>("ext_aid_xcom", &SrvExtAid::getExtAidMsg);
    srv_posllh_ = node_->create_service<extaid_posllh_msg>(SRV_EXTPOSLLH, std::bind(&SrvExtAid::get_extaid_posllh_msg, this, _1, _2));
    srv_posecef_ = node_->create_service<extaid_posecef_msg>(SRV_EXTPOSECEF, std::bind(&SrvExtAid::get_extaid_posecef_msg, this, _1, _2));
    srv_posutm_ = node_->create_service<extaid_posutm_msg>(SRV_EXTPOSUTM, std::bind(&SrvExtAid::get_extaid_posutm_msg, this, _1, _2));
    srv_posmgrs_ = node_->create_service<extaid_posmgrs_msg>(SRV_EXTPOSMGRS, std::bind(&SrvExtAid::get_extaid_posmgrs_msg, this, _1, _2));
    srv_hdg_ = node_->create_service<extaid_hdg_msg>(SRV_EXTHDG, std::bind(&SrvExtAid::get_extaid_hdg_msg, this, _1, _2));
    srv_vel_ = node_->create_service<extaid_vel_msg>(SRV_EXTVEL, std::bind(&SrvExtAid::get_extaid_vel_msg, this, _1, _2));
    srv_velbody_ = node_->create_service<extaid_velbody_msg>(SRV_EXTVELBODY, std::bind(&SrvExtAid::get_extaid_velbody_msg, this, _1, _2));
    srv_height_ = node_->create_service<extaid_height_msg>(SRV_EXTHEIGHT, std::bind(&SrvExtAid::get_extaid_height_msg, this, _1, _2));

    // connect to device
    xcom::TcpClient tcp_client(ip_address_, ip_port_);
    // define input source
    xcom::XcomClientReader tcp_reader(tcp_client);
    if(!tcp_reader.initialize()) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", SRV_EXTAID.c_str(), "unable to initialize tcp_reader");
    }
    // define output source (optional)
    xcom::XcomClientWriter tcp_writer(tcp_client);
    if(!tcp_writer.initialize()) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", SRV_EXTAID.c_str(), "unable to initialize tcp_writer");
    }

    // set_writer(&tcp_writer);
    xcom_.set_reader(&tcp_reader);
    xcom_.set_writer(&tcp_writer);
    xcom_.disable_forwarding();

    bool exit = false;
    RCLCPP_INFO(node_->get_logger(), "[%s] %s", SRV_EXTAID.c_str(), ("connecting to iNAT @ "
                                     + ip_address_ + ":"
                                     + std::to_string(ip_port_)
                                     + "...").c_str());
    while(!exit) {
       // RCLCPP_INFO(node_->get_logger(), "[%s] %s", "srv_extaid", ("connecting to iNAT @: "
       //                                  + ip_address_ + ":"
       //                                  + std::to_string(ip_port_)
       //                                  + " channel " + std::to_string(channel_)
       //                                  + "...").c_str());
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
                RCLCPP_ERROR(node_->get_logger(), "[%s] %s", SRV_EXTAID.c_str(), "no available channel found");
            }
            if(!tcp_reader.initialize()) {
//                perror("timeout: ");
                RCLCPP_ERROR(node_->get_logger(), "[%s] %s", SRV_EXTAID.c_str(), "tcp reader initialization failed");
            }
        } else {
            // exit = true;
        }
    }
    // while(true);
}

bool SrvExtAid::success() {
    return success_;
}

void SrvExtAid::get_extaid_posllh_msg(const std::shared_ptr<extaid_posllh_msg::Request> request,
                                      std::shared_ptr<extaid_posllh_msg::Response> response) {

    RCLCPP_INFO(node_->get_logger(), "[%s] %s", SRV_EXTPOSLLH.c_str(), "request received");

    auto cmd = xcom_.get_xcomcmd_extaid_posllh(gpsTimeStamp(request->time_stamp, leap_seconds_), request->time_mode,
                                               request->position, request->position_stddev,
                                               request->lever_arm, request->lever_arm_stddev,
                                               request->enable_msl_altitude);

    // posllh_response_ = std::make_shared<extaid_posllh_msg::Response>(*response);

    // tcp_writer_->write(reinterpret_cast<const uint8_t *>(&cmd), sizeof(cmd));
    // const auto rc = xcom_.process();
    // RCLCPP_INFO(node_->get_logger(), "[%s] %s" "%d", "srv_extaid_posllh", "inat response: ", rc);

    cmd.header.frame_counter = updateFrameCounter(extaidItems_.pos_llh.frame_counter);
    xcom_.send_message(cmd);

    extaidItems_.pos_llh.success = false;
    extaidItems_.pos_llh.requested = true;
    extaidItems_.pos_llh.frame_counter = cmd.header.frame_counter;
    // auto endTime = std::chrono::system_clock::now() + std::chrono::seconds(1);
    std::unique_lock<std::mutex> lck(m_);
    // cv_.wait(lck);
    cv_.wait_for(lck, std::chrono::seconds(1));
    // while (!ready) cv.wait(lck);
    // while(!ready_) {
    //     cv_.wait_until(lck, endTime);
    //     response->success = false;
    // }
    // while(!ready_) {
    //     cv_.wait_for(lck, std::chrono::seconds(1));
    //     // cv_.wait_for(lck, std::chrono::nanoseconds(1));
    //     response->success = false;
    //     extaidItems.pos_llh.response = false;
    //     RCLCPP_ERROR(node_->get_logger(), "[%s] %s", "srv_extaid_llh", "timeout while waiting for response from iNAT");
    //     ready_ = false;
    //     return;
    // }

    // RCLCPP_INFO(node_->get_logger(), "[%s] %s %d", SRV_EXTPOSLLH.c_str(), "setting success", extaidItems_.pos_llh.success);
    // assert(0==1);
    response->success = extaidItems_.pos_llh.success;
}

void SrvExtAid::get_extaid_posecef_msg(const std::shared_ptr<extaid_posecef_msg::Request> request,
                                       std::shared_ptr<extaid_posecef_msg::Response> response) {

    RCLCPP_INFO(node_->get_logger(), "[%s] %s", SRV_EXTPOSECEF.c_str(), "request received");

    auto cmd = xcom_.get_xcomcmd_extaid_posecef(gpsTimeStamp(request->time_stamp, leap_seconds_), request->time_mode,
                                                request->position, request->position_stddev,
                                                request->lever_arm, request->lever_arm_stddev);
    cmd.header.frame_counter = updateFrameCounter(extaidItems_.pos_ecef.frame_counter);
    xcom_.send_message(cmd);
    extaidItems_.pos_ecef.success = false;
    extaidItems_.pos_ecef.requested = true;
    extaidItems_.pos_ecef.frame_counter = cmd.header.frame_counter;
    std::unique_lock<std::mutex> lck(m_);
    cv_.wait_for(lck, std::chrono::seconds(1));
    response->success = extaidItems_.pos_ecef.success;
}

void SrvExtAid::get_extaid_posutm_msg(const std::shared_ptr<extaid_posutm_msg::Request> request,
                                      std::shared_ptr<extaid_posutm_msg::Response> response) {

    RCLCPP_INFO(node_->get_logger(), "[%s] %s", SRV_EXTPOSUTM.c_str(), "request received");

    auto cmd = xcom_.get_xcomcmd_extaid_posutm(gpsTimeStamp(request->time_stamp, leap_seconds_), request->time_mode,
                                               request->zone, request->north_hp,
                                               request->easting, request->northing, request->altitude,
                                               request->position_stddev,
                                               request->lever_arm, request->lever_arm_stddev);
    cmd.header.frame_counter = updateFrameCounter(extaidItems_.pos_utm.frame_counter);
    xcom_.send_message(cmd);
    extaidItems_.pos_utm.success = false;
    extaidItems_.pos_utm.requested = true;
    extaidItems_.pos_utm.frame_counter = cmd.header.frame_counter;
    std::unique_lock<std::mutex> lck(m_);
    cv_.wait_for(lck, std::chrono::seconds(1));
    response->success = extaidItems_.pos_utm.success;
}

void SrvExtAid::get_extaid_posmgrs_msg(const std::shared_ptr<extaid_posmgrs_msg::Request> request,
                                       std::shared_ptr<extaid_posmgrs_msg::Response> response) {

    RCLCPP_INFO(node_->get_logger(), "[%s] %s", SRV_EXTPOSMGRS.c_str(), "request received");

    int8_t mgrs[XCOM_EXTAID_POSMGRS_MAX_LENGTH];
    std::string s = request->mgrs;
    int len = std::min(static_cast<int>(s.length()), XCOM_EXTAID_POSMGRS_MAX_LENGTH);
    for(int i = 0; i < len; i++) {
        mgrs[i] = s.at(i);
    }
    auto cmd = xcom_.get_xcomcmd_extaid_posmgrs(gpsTimeStamp(request->time_stamp, leap_seconds_), request->time_mode,
                                                mgrs,
                                                request->altitude,
                                                request->position_stddev,
                                                request->lever_arm, request->lever_arm_stddev);
    cmd.header.frame_counter = updateFrameCounter(extaidItems_.pos_mgrs.frame_counter);
    xcom_.send_message(cmd);
    extaidItems_.pos_mgrs.success = false;
    extaidItems_.pos_mgrs.requested = true;
    extaidItems_.pos_mgrs.frame_counter = cmd.header.frame_counter;
    std::unique_lock<std::mutex> lck(m_);
    cv_.wait_for(lck, std::chrono::seconds(1));
    response->success = extaidItems_.pos_mgrs.success;
}

void SrvExtAid::get_extaid_hdg_msg(const std::shared_ptr<extaid_hdg_msg::Request> request,
                                   std::shared_ptr<extaid_hdg_msg::Response> response) {

    RCLCPP_INFO(node_->get_logger(), "[%s] %s", SRV_EXTHDG.c_str(), "request received");

    auto cmd = xcom_.get_xcomcmd_extaid_hdg(gpsTimeStamp(request->time_stamp, leap_seconds_), request->time_mode,
                                            request->heading, request->heading_stddev);
    cmd.header.frame_counter = updateFrameCounter(extaidItems_.hdg.frame_counter);
    xcom_.send_message(cmd);
    extaidItems_.hdg.success = false;
    extaidItems_.hdg.requested = true;
    extaidItems_.hdg.frame_counter = cmd.header.frame_counter;
    std::unique_lock<std::mutex> lck(m_);
    cv_.wait_for(lck, std::chrono::seconds(1));
    response->success = extaidItems_.hdg.success;
}

void SrvExtAid::get_extaid_vel_msg(const std::shared_ptr<extaid_vel_msg::Request> request,
                                   std::shared_ptr<extaid_vel_msg::Response> response) {

    RCLCPP_INFO(node_->get_logger(), "[%s] %s", SRV_EXTVEL.c_str(), "request received");

    auto cmd = xcom_.get_xcomcmd_extaid_vel_ned(gpsTimeStamp(request->time_stamp, leap_seconds_), request->time_mode,
                                            request->velocity, request->velocity_stddev);
    cmd.header.frame_counter = updateFrameCounter(extaidItems_.vel.frame_counter);
    xcom_.send_message(cmd);
    extaidItems_.vel.success = false;
    extaidItems_.vel.requested = true;
    extaidItems_.vel.frame_counter = cmd.header.frame_counter;
    std::unique_lock<std::mutex> lck(m_);
    cv_.wait_for(lck, std::chrono::seconds(1));
    response->success = extaidItems_.vel.success;
}

void SrvExtAid::get_extaid_velbody_msg(const std::shared_ptr<extaid_velbody_msg::Request> request,
                                       std::shared_ptr<extaid_velbody_msg::Response> response) {

    RCLCPP_INFO(node_->get_logger(), "[%s] %s", SRV_EXTVELBODY.c_str(), "request received");
    if(!rotVehicleToEnclosure.isKnown) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", SRV_EXTVELBODY.c_str(),
                     "Rotation between vehicle and enclosure frame is not known, "
                     "please restart the node");
        response->success = false;
        return;
    }
    // Transform the velocity from vehicle frame to enclosure frame
    tf2::Vector3 vel_vehicle(request->velocity[0], request->velocity[1], request->velocity[2]);
    tf2::Vector3 vel_enclosure =
        tf2::quatRotate(rotVehicleToEnclosure.q_vehicle_to_enclosure, vel_vehicle);
    tf2::Vector3 vel_stddev_enclosure =
        tf2::quatRotate(rotVehicleToEnclosure.q_vehicle_to_enclosure,
                        tf2::Vector3(request->velocity_stddev[0], request->velocity_stddev[1],
                                     request->velocity_stddev[2]));

    auto cmd = xcom_.get_xcomcmd_extaid_vel_body(gpsTimeStamp(request->time_stamp, leap_seconds_), request->time_mode,
                                                request->velocity, request->velocity_stddev,
                                                request->lever_arm, request->lever_arm_stddev);
    cmd.header.frame_counter = updateFrameCounter(extaidItems_.vel_body.frame_counter);
    xcom_.send_message(cmd);
    extaidItems_.vel_body.success = false;
    extaidItems_.vel_body.requested = true;
    extaidItems_.vel_body.frame_counter = cmd.header.frame_counter;
    std::unique_lock<std::mutex> lck(m_);
    cv_.wait_for(lck, std::chrono::seconds(1));
    response->success = extaidItems_.vel_body.success;
}

void SrvExtAid::get_extaid_height_msg(const std::shared_ptr<extaid_height_msg::Request> request,
                                       std::shared_ptr<extaid_height_msg::Response> response) {

    RCLCPP_INFO(node_->get_logger(), "[%s] %s", SRV_EXTHEIGHT.c_str(), "request received");

    auto cmd = xcom_.get_xcomcmd_extaid_height(gpsTimeStamp(request->time_stamp, leap_seconds_), request->time_mode,
                                               request->height, request->height_stddev);
    cmd.header.frame_counter = updateFrameCounter(extaidItems_.height.frame_counter);
    xcom_.send_message(cmd);
    extaidItems_.height.success = false;
    extaidItems_.height.requested = true;
    extaidItems_.height.frame_counter = cmd.header.frame_counter;
    std::unique_lock<std::mutex> lck(m_);
    cv_.wait_for(lck, std::chrono::seconds(1));
    response->success = extaidItems_.height.success;
}

void SrvExtAid::handle_command(uint16_t cmd_id, std::size_t frame_len, uint8_t *frame) noexcept {
       std::cout << "Command received" << cmd_id << "\n";
}

void SrvExtAid::handle_xcom_param(const XCOMParIMU_MISALIGN& msg) noexcept {
    float rot_xyz[3] = {msg.xyz[0], msg.xyz[1], msg.xyz[2]};

    RCLCPP_INFO(node_->get_logger(),
                "[%s] %s [%0.3f, %0.3f, %0.3f]",
                SRV_EXTAID.c_str(), "Rotation between INS enclosure and vehicle frame:", rot_xyz[0], rot_xyz[1], rot_xyz[2]);
    
                rotVehicleToEnclosure.q_vehicle_to_enclosure.setRPY(rot_xyz[0], rot_xyz[1], rot_xyz[2]);
    rotVehicleToEnclosure.q_vehicle_to_enclosure =
        rotVehicleToEnclosure.q_vehicle_to_enclosure
            .inverse();  // Invert the rotation to get the final transformation from vehicle frame into
                         // the INS enclosure frame
    rotVehicleToEnclosure.isKnown = true;
}

bool SrvExtAid::connected() {
    return connected_;
}

void SrvExtAid::handle_response(XCOMResp response) noexcept {

    // RCLCPP_INFO(node_->get_logger(), "[%s] %s %d", "srv_extaid", "response: ", response);

    if(!init_done_) {
        if(response == XCOMResp::OK) {
            connected_ = true;
            invalid_channel_ = false;
            RCLCPP_INFO(node_->get_logger(), "[%s] %s", SRV_EXTAID.c_str(),
                        ("connected to iNAT on channel " + std::to_string(channel_)).c_str());

            auto cmd_clearall = xcom_.get_xcomcmd_clearall();
            xcom_.send_message(cmd_clearall);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            // Get rotation between INS enclosure and vehicle frame
            // RCLCPP_INFO(node_->get_logger(),
            //             "requesting rotation between INS enclosure and vehicle frame");
            auto cmd_add_ins_enclosure = xcom_.get_generic_param<XCOMParIMU_MISALIGN>();
            xcom_.send_message(cmd_add_ins_enclosure);

            // uint16_t div1 = calculateDividerForRate(((topic_freq_ > Config::FRQ_GNSS) ? (Config::FRQ_GNSS) : (topic_freq_)), maintiming_, prescaler_);
            // msg_GNSSSOL_frq_ = calculateRateForDivider(div1, maintiming_, prescaler_);
            // auto cmd_add_gnsssol = xcom_.get_xcomcmd_enablelog(XCOM_MSGID_GNSSSOL, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC, div1);
            // xcom_.send_message(cmd_add_gnsssol);

            success_ = true;
            init_done_ = true;
            // cv_build_.notify_one();
        } else {
            if(response == XCOMResp::INVALIDCHANNEL) {
                invalid_channel_ = true;
            } else if(response == XCOMResp::INVALIDCMD) {
                RCLCPP_WARN(node_->get_logger(), "[%s] %s", SRV_EXTAID.c_str(), "invalid command");
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[%s] %s: %d", SRV_EXTAID.c_str(), "invalid response received", static_cast<int>(response));
            }
            xcom_.set_rc(xcom::XComState::ReturnCode::InvalidResponse);
            xcom_.forced_exit();
        }
    } else {
        // RCLCPP_INFO(node_->get_logger(), "[%s] %s %d", "srv_extaid", "inat response: ", !static_cast<int>(response));
        // if(response == XCOMResp::OK) {
        //     extaidItems_.pos_llh.success = true;
        // } else {
        //     extaidItems_.pos_llh.success = false;
        // }

        // if(response == XCOMResp::OK) {
        // if(response == XCOMResp::PAROUTOFRANGE) {
        //     if(extaidItems_.pos_llh.requested) {
        //         extaidItems_.pos_llh.requested = false;
        //         extaidItems_.pos_llh.success = true;
        //     } else if(extaidItems_.pos_ecef.requested) {
        //         extaidItems_.pos_ecef.requested = false;
        //         extaidItems_.pos_ecef.success = true;
        //     }
        // }
        uint8_t frame_counter = reinterpret_cast<const XCOMHeader*>(xcom_.get_payload())->frame_counter;
        int inv_resp = !static_cast<int>(response);
        if(extaidItems_.pos_llh.requested) {
            extaidItems_.pos_llh.requested = false;
            if(frame_counter == extaidItems_.pos_llh.frame_counter) {
                // RCLCPP_INFO(node_->get_logger(), "[%s] %s %d", "srv_extaid_llh", "frame counter match: ", frame_counter);
                extaidItems_.pos_llh.success = inv_resp;
                if(extaidItems_.pos_llh.success) {
                    RCLCPP_INFO(node_->get_logger(), "[%s] %s", SRV_EXTPOSLLH.c_str(), "iNAT accepted the data");
                } else {
                    RCLCPP_WARN(node_->get_logger(), "[%s] %s (XCOMResp: %d)", SRV_EXTPOSLLH.c_str(), "iNAT did not accept the data", static_cast<int>(response));
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[%s] %s %d %d", SRV_EXTPOSLLH.c_str(), "frame counter mismatch: ", frame_counter, extaidItems_.pos_llh.frame_counter);
            }
        } else if(extaidItems_.pos_ecef.requested) {
            extaidItems_.pos_ecef.requested = false;
            if(frame_counter == extaidItems_.pos_ecef.frame_counter) {
                // RCLCPP_INFO(node_->get_logger(), "[%s] %s %d", "srv_extaid_ecef", "frame counter match: ", frame_counter);
                extaidItems_.pos_ecef.success = inv_resp;
                if(extaidItems_.pos_ecef.success) {
                    RCLCPP_INFO(node_->get_logger(), "[%s] %s", SRV_EXTPOSECEF.c_str(), "iNAT accepted the data");
                } else {
                    RCLCPP_WARN(node_->get_logger(), "[%s] %s (XCOMResp: %d)", SRV_EXTPOSECEF.c_str(), "iNAT did not accept the data", static_cast<int>(response));
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[%s] %s %d %d", SRV_EXTPOSECEF.c_str(), "frame counter mismatch: ", frame_counter, extaidItems_.pos_llh.frame_counter);
            }
        } else if(extaidItems_.pos_utm.requested) {
            extaidItems_.pos_utm.requested = false;
            if(frame_counter == extaidItems_.pos_utm.frame_counter) {
                // RCLCPP_INFO(node_->get_logger(), "[%s] %s %d", "srv_extaid_utm", "frame counter match: ", frame_counter);
                extaidItems_.pos_utm.success = inv_resp;
                if(extaidItems_.pos_utm.success) {
                    RCLCPP_INFO(node_->get_logger(), "[%s] %s", SRV_EXTPOSUTM.c_str(), "iNAT accepted the data");
                } else {
                    RCLCPP_WARN(node_->get_logger(), "[%s] %s (XCOMResp: %d)", SRV_EXTPOSUTM.c_str(), "iNAT did not accept the data", static_cast<int>(response));
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[%s] %s %d %d", SRV_EXTPOSUTM.c_str(), "frame counter mismatch: ", frame_counter, extaidItems_.pos_llh.frame_counter);
            }
        } else if(extaidItems_.hdg.requested) {
            extaidItems_.hdg.requested = false;
            if(frame_counter == extaidItems_.hdg.frame_counter) {
                // RCLCPP_INFO(node_->get_logger(), "[%s] %s %d", "srv_extaid_hdg", "frame counter match: ", frame_counter);
                extaidItems_.hdg.success = inv_resp;
                if(extaidItems_.hdg.success) {
                    RCLCPP_INFO(node_->get_logger(), "[%s] %s", SRV_EXTHDG.c_str(), "iNAT accepted the data");
                } else {
                    RCLCPP_WARN(node_->get_logger(), "[%s] %s (XCOMResp: %d)", SRV_EXTHDG.c_str(), "iNAT did not accept the data", static_cast<int>(response));
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[%s] %s %d %d", SRV_EXTHDG.c_str(), "frame counter mismatch: ", frame_counter, extaidItems_.pos_llh.frame_counter);
            }
        } else if(extaidItems_.vel.requested) {
            extaidItems_.vel.requested = false;
            if(frame_counter == extaidItems_.vel.frame_counter) {
                // RCLCPP_INFO(node_->get_logger(), "[%s] %s %d", "srv_extaid_vel", "frame counter match: ", frame_counter);
                extaidItems_.vel.success = inv_resp;
                if(extaidItems_.vel.success) {
                    RCLCPP_INFO(node_->get_logger(), "[%s] %s", SRV_EXTVEL.c_str(), "iNAT accepted the data");
                } else {
                    RCLCPP_WARN(node_->get_logger(), "[%s] %s (XCOMResp: %d)", SRV_EXTVEL.c_str(), "iNAT did not accept the data", static_cast<int>(response));
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[%s] %s %d %d", SRV_EXTVEL.c_str(), "frame counter mismatch: ", frame_counter, extaidItems_.pos_llh.frame_counter);
            }
        } else if(extaidItems_.vel_body.requested) {
            extaidItems_.vel_body.requested = false;
            if(frame_counter == extaidItems_.vel_body.frame_counter) {
                // RCLCPP_INFO(node_->get_logger(), "[%s] %s %d", "srv_extaid_velbody", "frame counter match: ", frame_counter);
                extaidItems_.vel_body.success = inv_resp;
                if(extaidItems_.vel_body.success) {
                    RCLCPP_INFO(node_->get_logger(), "[%s] %s", SRV_EXTVELBODY.c_str(), "iNAT accepted the data");
                } else {
                    RCLCPP_WARN(node_->get_logger(), "[%s] %s (XCOMResp: %d)", SRV_EXTVELBODY.c_str(), "iNAT did not accept the data", static_cast<int>(response));
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[%s] %s %d %d", SRV_EXTVELBODY.c_str(), "frame counter mismatch: ", frame_counter, extaidItems_.pos_llh.frame_counter);
            }
        } else if(extaidItems_.height.requested) {
            extaidItems_.height.requested = false;
            if(frame_counter == extaidItems_.height.frame_counter) {
                // RCLCPP_INFO(node_->get_logger(), "[%s] %s %d", "srv_extaid_height", "frame counter match: ", frame_counter);
                extaidItems_.height.success = inv_resp;
                if(extaidItems_.height.success) {
                    RCLCPP_INFO(node_->get_logger(), "[%s] %s", SRV_EXTHEIGHT.c_str(), "iNAT accepted the data");
                } else {
                    RCLCPP_WARN(node_->get_logger(), "[%s] %s (XCOMResp: %d)", SRV_EXTHEIGHT.c_str(), "iNAT did not accept the data", static_cast<int>(response));
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[%s] %s %d %d", SRV_EXTHEIGHT.c_str(), "frame counter mismatch: ", frame_counter, extaidItems_.pos_llh.frame_counter);
            }
        }
        // RCLCPP_INFO(node_->get_logger(), "[%s] %s", "srv_extaid", "notifying");
        cv_.notify_all();
    }

    // if(response == XCOMResp::OK) {
    //     invalid_channel_ = false;

    //     if(!init_done_) {

    //         RCLCPP_INFO(node_->get_logger(), "[%s] %s", "srv_extaid",
    //                     ("connected to iNAT on channel " + std::to_string(channel_)).c_str());

    //         auto cmd_clearall = xcom_.get_xcomcmd_clearall();
    //         xcom_.send_message(cmd_clearall);
    //         std::this_thread::sleep_for(std::chrono::milliseconds(200));

    //         // uint16_t div1 = calculateDividerForRate(((topic_freq_ > Config::FRQ_GNSS) ? (Config::FRQ_GNSS) : (topic_freq_)), maintiming_, prescaler_);
    //         // msg_GNSSSOL_frq_ = calculateRateForDivider(div1, maintiming_, prescaler_);
    //         // auto cmd_add_gnsssol = xcom_.get_xcomcmd_enablelog(XCOM_MSGID_GNSSSOL, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC, div1);
    //         // xcom_.send_message(cmd_add_gnsssol);

    //         success_ = true;
    //     }
    //     init_done_ = true;
    // } else {
    //     if(!init_done_) {
    //         if(response == XCOMResp::INVALIDCHANNEL) {
    //             invalid_channel_ = true;
    //         } else if(response == XCOMResp::INVALIDCMD) {
    //             RCLCPP_WARN(node_->get_logger(), "[%s] %s", "srv_extaid", "invalid command");
    //         } else {
    //             RCLCPP_ERROR(node_->get_logger(), "[%s] %s: %d", "srv_extaid", "invalid response received", static_cast<int>(response));
    //         }
    //         xcom_.set_rc(xcom::XComState::ReturnCode::InvalidResponse);
    //         xcom_.forced_exit();
    //     } else {
    //         RCLCPP_INFO(node_->get_logger(), "[%s] %s %d", "srv_extaid", "inat response: ", response);
    //         // if(posecef_response_)
    //         //     posecef_response_->success = true;
    //         // if(posllh_response_)
    //         //     posllh_response_->success = true;
    //         // ready_ = true;
    //         extaidItems_.pos_llh.success = false;
    //         cv_.notify_all();
    //     }
    // }
}


