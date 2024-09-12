#ifndef XCOMHANDLER_HPP
#define XCOMHANDLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <ixcom/ixcom.h>
#include <ixcom/message_handler.h>
#include <ixcom/tcp_client.h>
#include <ixcom/command_handler.h>
#include <ixcom/message_handler.h>
#include <ixcom/response_handler.h>
#include <ixcom/parameter_handler.h>
//#include <gflags/gflags.h>
#include <iostream>

using XComMessages_XcomHandler = xcom::MessageHandler<
                                    XCOMmsg_GNSSSOL,
                                    XCOMmsg_GNSSTIME,
                                    XCOMmsg_SYSSTAT>;
using XComParameters_XcomHandler = xcom::ParameterHandler<
                                    XCOMParSYS_MAINTIMING,
                                    XCOMParSYS_PRESCALER,
                                    XCOMParSYS_FWVERSION,
                                    XCOMParDAT_SYSSTAT,
                                    XCOMParXCOM_SERIALPORT,     // linux firmware
                                    XCOMParXCOM_INTERFACE,      // qnx firmware
                                    XCOMParDAT_VEL>;
class XcomHandler : public XComMessages_XcomHandler, XComParameters_XcomHandler, xcom::CommandHandler, xcom::ResponseHandler
{
public:
    explicit XcomHandler(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                         xcom::XComState &state,
                         bool serial_ignore,
                         uint8_t serial_port,
                         uint32_t serial_baud,
                         bool serial_enable,
                         int32_t leap_seconds);
    ~XcomHandler() override = default;

    bool invalidChannel();
    uint16_t getMaintiming();
    uint16_t getPrescaler();
    int32_t getLeapSeconds();
    bool complete();

private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    xcom::XComState &_xcom;
    bool invalid_channel_ = false;
    bool _init_done = false;

    void handle_command(uint16_t cmd_id, std::size_t frame_len, uint8_t *frame) override;
    void handle_response(XCOMResp response) override;
    void handle_xcom_msg(const XCOMmsg_GNSSSOL &msg) override;
    void handle_xcom_msg(const XCOMmsg_GNSSTIME &msg) override;
    void handle_xcom_msg(const XCOMmsg_SYSSTAT &msg) override;
    void handle_xcom_param(const XCOMParSYS_MAINTIMING& param) override;
    void handle_xcom_param(const XCOMParSYS_PRESCALER& param) override;
    void handle_xcom_param(const XCOMParSYS_FWVERSION& param) override;
    void handle_xcom_param(const XCOMParDAT_SYSSTAT& param) override;
    void handle_xcom_param(const XCOMParXCOM_SERIALPORT& param) override;
    void handle_xcom_param(const XCOMParXCOM_INTERFACE& param) override;
    void handle_xcom_param(const XCOMParDAT_VEL& param) override;

    void send_serial_config();
    void set_sysstat_mode();
    void add_sysstat();

    std::atomic<uint16_t> maintiming_ = 0;
    std::atomic<uint16_t> prescaler_ = 0;
    std::atomic<bool> sysstat_added_ = false;
    const uint32_t sysstat_mode_ = PARDAT_SYSSTAT_MASK_IMU |
                                   PARDAT_SYSSTAT_MASK_GNSS |
                                   PARDAT_SYSSTAT_MASK_REMALIGNTIME;
    uint32_t current_sysstat_mode_;
    bool sysstat_mode_is_set_ = false;
    int fwv_[3] = {0, 0, 0};
    int32_t leap_seconds_;
    XCOM_PARDAT_VEL_Mode vel_mode_;
    bool serial_ignore_;
    uint8_t serial_port_;
    uint32_t serial_baud_;
    bool serial_enable_;

    enum class CompleteStatus : uint16_t
    {
        NONE = 0,
        MAINTIMING = 1,
        PRESCALER = 2,
        FWV = 4,
        LEAPSECONDS = 8,
        VELMODE = 16,
        SERIAL = 32,
        ALIGNMENT = 64,
        RESPONSES = 128,
        SYSSTAT = 256,
        FULL = 511
    };
    CompleteStatus complete_status_ = CompleteStatus::NONE;
};

#endif // XCOMHANDLER_HPP
