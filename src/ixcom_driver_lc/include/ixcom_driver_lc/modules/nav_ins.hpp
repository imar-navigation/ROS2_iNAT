#pragma once

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <ixcom/ixcom.h>
#include <ixcom/message_handler.h>
#include <ixcom/tcp_client.h>
//#include <ixcom/command_handler.h>
#include <ixcom/message_handler.h>
#include <ixcom/response_handler.h>
#include <ixcom/parameter_handler.h>
#include <ixcom_driver_lc/ixcom_driver_conf.hpp>
#include <ixcom_driver_lc/modules/transform.hpp>

using XComMessages_NavIns = xcom::MessageHandler<XCOMmsg_GNSSSOL,
                                                 XCOMmsg_INSSOL,
                                                 XCOMmsg_EKFSTDDEV>;
using XComParameters_NavIns = xcom::ParameterHandler<XCOMParGNSS_LOCKOUTSYSTEM,
                                                     XCOMParDAT_POS>;

class NavINS : public XComMessages_NavIns, XComParameters_NavIns, xcom::ResponseHandler  // , xcom::CommandHandler
{
public:
    NavINS(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
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
           const rclcpp::QoS &qos);
    ~NavINS();

    void activate();
    uint16_t getSetupFreq();
    bool success();
    void cleanup();

private:
    using NavSatFixMsg = sensor_msgs::msg::NavSatFix;
    using NavSatStatusMsg = sensor_msgs::msg::NavSatStatus;

    //    void handle_command(uint16_t cmd_id, std::size_t frame_len, uint8_t *frame) override;
    void handle_response(XCOMResp response) noexcept override;
    void handle_xcom_msg(const XCOMmsg_GNSSSOL &msg) noexcept override;
    void handle_xcom_msg(const XCOMmsg_INSSOL &msg) noexcept override;
    void handle_xcom_msg(const XCOMmsg_EKFSTDDEV &msg) noexcept override;
    void handle_xcom_param(const XCOMParGNSS_LOCKOUTSYSTEM& param) override;
    void handle_xcom_param(const XCOMParDAT_POS& param) override;

    void init();

    void setParData_LOCKOUTSYSTEM(const XCOMParGNSS_LOCKOUTSYSTEM& param);
    void setParData_POS(const XCOMParDAT_POS& param);
    void updateGNSSSOL(const XCOMmsg_GNSSSOL &msg);
    void updateINSSOL(const XCOMmsg_INSSOL &msg);
    void updateEKFSTDDEV(const XCOMmsg_EKFSTDDEV &msg);

    bool parDataLockoutIsSet_ = false;
    bool parDataPosIsSet_ = false;
    bool gnssSolDataIsSet_ = false;
    bool insSolDataIsSet_ = false;
    bool ekfDataIsSet_ = false;

    void publish();

    void frq_mon();

    uint16_t channel_ = 0;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    xcom::XComState &xcom_;
    bool invalid_channel_ = false;
    bool init_done_ = false;
    bool active_ = false;
    int32_t setup_freq_ = 0;
    size_t num_of_subscribers_ = 0;
    std::atomic_bool success_ = ATOMIC_VAR_INIT(false);
    rclcpp::Publisher<NavSatFixMsg>::SharedPtr pub_;
    NavSatFixMsg navsatfix_msg_;
    NavSatStatusMsg navsatstatus_msg_;
    rclcpp::Time gps_time_;
    XCOM_PARDAT_POS_AltMode alt_mode_ = XCOM_PARDAT_POS_AltMode::XCOM_PARDAT_POS_Alt_WGS84;
    float undulation_ = 0.f;

    const std::string &ip_address_;
    int32_t ip_port_;
    const std::string &topic_name_;
    int32_t topic_freq_;
    Config::TimestampMode timestamp_mode_;
    int32_t leap_seconds_;
    uint16_t maintiming_;
    uint16_t prescaler_;
    const rclcpp::QoS &qos_;
    TransformStamped::SharedPtr tf2_;

    uint64_t time_delta_ = 0;
    bool run_frq_mon_ = true;
    const uint16_t mon_frq_ = 1000;
    const uint16_t mon_t_gap_ = 1;
    std::chrono::time_point<std::chrono::high_resolution_clock> t_pub_;
    std::chrono::time_point<std::chrono::high_resolution_clock> t_pub_upd_;
    const uint16_t par_frq_ = 1;
    std::atomic_uint32_t par_LOCKOUTSYSTEM_age_ = ATOMIC_VAR_INIT(0);
    uint32_t par_LOCKOUTSYSTEM_c_ = 0;
    std::atomic_uint32_t par_POS_age_ = ATOMIC_VAR_INIT(0);
    uint32_t par_POS_c_ = 0;
    uint16_t msg_INSSOL_frq_ = 0;
    std::atomic_uint32_t msg_INSSOL_age_ = ATOMIC_VAR_INIT(0);
    uint32_t msg_INSSOL_c_ = 0;
    uint16_t msg_GNSSSOL_frq_ = 0;
    std::atomic_uint32_t msg_GNSSSOL_age_ = ATOMIC_VAR_INIT(0);
    uint32_t msg_GNSSSOL_c_ = 0;
    uint16_t msg_EKFSTDDEV_frq_ = 0;
    std::atomic_uint32_t msg_EKFSTDDEV_age_ = ATOMIC_VAR_INIT(0);
    uint32_t msg_EKFSTDDEV_c_ = 0;
    const uint32_t age_max_val_ = 0xffffffff;
    const uint32_t xcom_age_max_ = 3;
    std::atomic_uint64_t duration_ = ATOMIC_VAR_INIT(0);
    enum class PubState {
        UNKNOWN,
        OK,
        WARN,
        ERR
    };
    PubState pub_state_ = PubState::UNKNOWN;
};
