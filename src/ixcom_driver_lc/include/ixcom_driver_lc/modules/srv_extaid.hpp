#pragma once

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <ixcom/ixcom.h>
#include <ixcom/message_handler.h>
#include <ixcom/tcp_client.h>
#include <ixcom/command_handler.h>
// #include <ixcom/message_handler.h>
#include <ixcom/response_handler.h>
#include "ixcom_interfaces/srv/ext_aid_pos_llh.hpp"
#include "ixcom_interfaces/srv/ext_aid_pos_ecef.hpp"
#include "ixcom_interfaces/srv/ext_aid_pos_utm.hpp"
#include "ixcom_interfaces/srv/ext_aid_pos_mgrs.hpp"
#include "ixcom_interfaces/srv/ext_aid_hdg.hpp"
#include "ixcom_interfaces/srv/ext_aid_vel.hpp"
#include "ixcom_interfaces/srv/ext_aid_vel_body.hpp"
#include "ixcom_interfaces/srv/ext_aid_height.hpp"
#include <condition_variable>
#include <mutex>

// using XComMessages_SrvExtAid = xcom::MessageHandler<XCOMmsg_GNSSSOL>;

class SrvExtAid : public xcom::ResponseHandler, xcom::CommandHandler
{
public:

    using SharedPtr = std::shared_ptr<SrvExtAid>;

    SrvExtAid(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
              xcom::XComState &state,
              const std::string &ip_address,
              int32_t ip_port,
              int32_t leap_seconds);
    ~SrvExtAid();

    // void activate();
    // bool success();
    void cleanup();

private:

    const std::string SRV_EXTAID {"srv_extaid"};
    const std::string SRV_EXTPOSLLH {"ext_position_llh"};
    const std::string SRV_EXTPOSECEF {"ext_position_ecef"};
    const std::string SRV_EXTPOSUTM {"ext_position_utm"};
    const std::string SRV_EXTPOSMGRS {"ext_position_mgrs"};
    const std::string SRV_EXTHDG {"ext_heading"};
    const std::string SRV_EXTVEL {"ext_velocity"};
    const std::string SRV_EXTVELBODY {"ext_velocity_body"};
    const std::string SRV_EXTHEIGHT {"ext_height"};

    using extaid_posllh_msg = ixcom_interfaces::srv::ExtAidPosLlh;
    using extaid_posecef_msg = ixcom_interfaces::srv::ExtAidPosEcef;
    using extaid_posutm_msg = ixcom_interfaces::srv::ExtAidPosUtm;
    using extaid_posmgrs_msg = ixcom_interfaces::srv::ExtAidPosMgrs;
    using extaid_hdg_msg = ixcom_interfaces::srv::ExtAidHdg;
    using extaid_vel_msg = ixcom_interfaces::srv::ExtAidVel;
    using extaid_velbody_msg = ixcom_interfaces::srv::ExtAidVelBody;
    using extaid_height_msg = ixcom_interfaces::srv::ExtAidHeight;

    void handle_command(uint16_t cmd_id, std::size_t frame_len, uint8_t *frame) noexcept override;
    void handle_response(XCOMResp response) noexcept override;
    // void handle_xcom_msg(const XCOMmsg_GNSSSOL &msg) override;

    //    std::string log_msg(const std::string &s);
    void init();

    // void set_writer(xcom::IWriter* writer) noexcept { tcp_writer_ = writer; }

    void get_extaid_posllh_msg(const std::shared_ptr<extaid_posllh_msg::Request> request,
                               std::shared_ptr<extaid_posllh_msg::Response> response);
    void get_extaid_posecef_msg(const std::shared_ptr<extaid_posecef_msg::Request> request,
                                std::shared_ptr<extaid_posecef_msg::Response> response);
    void get_extaid_posutm_msg(const std::shared_ptr<extaid_posutm_msg::Request> request,
                               std::shared_ptr<extaid_posutm_msg::Response> response);
    void get_extaid_posmgrs_msg(const std::shared_ptr<extaid_posmgrs_msg::Request> request,
                                std::shared_ptr<extaid_posmgrs_msg::Response> response);
    void get_extaid_hdg_msg(const std::shared_ptr<extaid_hdg_msg::Request> request,
                            std::shared_ptr<extaid_hdg_msg::Response> response);
    void get_extaid_vel_msg(const std::shared_ptr<extaid_vel_msg::Request> request,
                            std::shared_ptr<extaid_vel_msg::Response> response);
    void get_extaid_velbody_msg(const std::shared_ptr<extaid_velbody_msg::Request> request,
                                std::shared_ptr<extaid_velbody_msg::Response> response);
    void get_extaid_height_msg(const std::shared_ptr<extaid_height_msg::Request> request,
                               std::shared_ptr<extaid_height_msg::Response> response);

    // void publish();

    uint16_t channel_ = 0;
    bool invalid_channel_ = false;
    bool init_done_ = false;
    // bool active_ = false;
    // std::atomic_bool success_ = ATOMIC_VAR_INIT(false);

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    rclcpp::Service<extaid_posllh_msg>::SharedPtr srv_posllh_;
    rclcpp::Service<extaid_posecef_msg>::SharedPtr srv_posecef_;
    rclcpp::Service<extaid_posutm_msg>::SharedPtr srv_posutm_;
    rclcpp::Service<extaid_posmgrs_msg>::SharedPtr srv_posmgrs_;
    rclcpp::Service<extaid_hdg_msg>::SharedPtr srv_hdg_;
    rclcpp::Service<extaid_vel_msg>::SharedPtr srv_vel_;
    rclcpp::Service<extaid_velbody_msg>::SharedPtr srv_velbody_;
    rclcpp::Service<extaid_height_msg>::SharedPtr srv_height_;
    xcom::XComState &xcom_;
    const std::string &ip_address_;
    int32_t ip_port_;
    int32_t leap_seconds_;
    // xcom::IWriter* tcp_writer_ = nullptr;
    std::mutex m_;
    std::condition_variable cv_;
    struct ExtaidItem {
        bool requested;
        bool success;
        uint8_t frame_counter;
    };
    struct ExtaidItems {
        ExtaidItem pos_llh;
        ExtaidItem pos_ecef;
        ExtaidItem pos_utm;
        ExtaidItem pos_mgrs;
        ExtaidItem hdg;
        ExtaidItem vel;
        ExtaidItem vel_body;
        ExtaidItem height;
    } extaidItems_;
    // bool ready_ = false;
    // std::shared_ptr<extaid_posecef_msg::Response> posecef_response_;
    // std::shared_ptr<extaid_posllh_msg::Response> posllh_response_;
};
