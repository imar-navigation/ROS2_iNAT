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

using XComMessages_NavGnss = xcom::MessageHandler<XCOMmsg_GNSSSOL>;
using XComParameters_NavGnss = xcom::ParameterHandler<XCOMParGNSS_LOCKOUTSYSTEM>;

class NavGNSS : public XComMessages_NavGnss, XComParameters_NavGnss, xcom::ResponseHandler  // , xcom::CommandHandler
{
public:
    // NavGNSS(rclcpp_lifecycle::LifecycleNode::SharedPtr node, iXCOM::Client& client, Config::TimestampMode timestamp_mode);
    NavGNSS(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
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
    ~NavGNSS();

    void activate();
    uint16_t getSetupFreq();
    bool success();
    void cleanup();

private:
    using NavSatStatusMsg = sensor_msgs::msg::NavSatStatus;
    using NavSatFixMsg = sensor_msgs::msg::NavSatFix;

    //    void handle_command(uint16_t cmd_id, std::size_t frame_len, uint8_t *frame) override;
    void handle_response(XCOMResp response) override;
    void handle_xcom_msg(const XCOMmsg_GNSSSOL &msg) override;
    void handle_xcom_param(const XCOMParGNSS_LOCKOUTSYSTEM& param) override;

    //    std::string log_msg(const std::string &s);
    void init();

    void setParData(const XCOMParGNSS_LOCKOUTSYSTEM& param);
    void updateGNSSSOL(const XCOMmsg_GNSSSOL &msg);

    void publish();

    void frq_mon();

    uint16_t channel_ = 0;
    bool invalid_channel_ = false;
    bool init_done_ = false;
    bool parDataIsSet_ = false;
    bool gnssSolDataIsSet_ = false;
    bool active_ = false;
    int32_t setup_freq_ = 0;
    size_t num_of_subscribers_ = 0;
    std::atomic_bool success_ = false;
    rclcpp::Publisher<NavSatFixMsg>::SharedPtr pub_;
    NavSatFixMsg navsatfix_msg_;
    NavSatStatusMsg navsatstatus_msg_;
    rclcpp::Time gps_time_;

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    xcom::XComState &xcom_;
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
    std::atomic_uint32_t par_GNSS_LOCKOUTSYSTEM_age_ = 0;
    uint32_t par_GNSS_LOCKOUTSYSTEM_c_ = 0;
    uint16_t msg_GNSSSOL_frq_ = 0;
    std::atomic_uint32_t msg_GNSSSOL_age_ = 0;
    uint32_t msg_GNSSSOL_c_ = 0;
    const uint32_t age_max_val_ = 0xffffffff;
    const uint32_t xcom_age_max_ = 3;
    std::atomic_uint64_t duration_ = 0;
    enum class PubState {
        UNKNOWN,
        OK,
        WARN,
        ERR
    };
    PubState pub_state_ = PubState::UNKNOWN;
};
