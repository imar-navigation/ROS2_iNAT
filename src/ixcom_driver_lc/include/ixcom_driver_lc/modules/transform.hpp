#pragma once

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <ixcom/ixcom.h>
#include <ixcom/message_handler.h>
#include <ixcom/tcp_client.h>
//#include <ixcom/command_handler.h>
#include <ixcom/message_handler.h>
#include <ixcom/response_handler.h>
#include <ixcom/parameter_handler.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <ixcom_driver_lc/ixcom_driver_conf.hpp>

using XComMessages_Transform   = xcom::MessageHandler<XCOMmsg_GNSSLEVERARM>;
using XComParameters_Transform = xcom::ParameterHandler<XCOMParIMU_MISALIGN>;

class TransformStamped : public XComMessages_Transform,
                         xcom::ResponseHandler,
                         XComParameters_Transform  //, xcom::CommandHandler
{
public:
    using SharedPtr = std::shared_ptr<TransformStamped>;

    TransformStamped(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
        xcom::XComState &state,
        const std::string &ip_address,
        int32_t ip_port,
        Config::TimestampMode timestamp_mode,
        int32_t leap_seconds,
        uint16_t maintiming,
        uint16_t prescaler);
    ~TransformStamped();

    void activate();
    void subscriberAdded();
    bool connected();
    bool success();
    void cleanup();

private:
    using TransformStampedMsg = geometry_msgs::msg::TransformStamped;
    using TFMsg = tf2_msgs::msg::TFMessage;

//    void handle_command(uint16_t cmd_id, std::size_t frame_len, uint8_t *frame) override;
    void handle_response(XCOMResp response) noexcept override;
    void handle_xcom_msg(const XCOMmsg_GNSSLEVERARM& msg) noexcept override;
    void handle_xcom_param(const XCOMParIMU_MISALIGN& par) noexcept override;

    void init();

    void updateGNSSLEVERARM(const XCOMmsg_GNSSLEVERARM& msg);
    void updateIMUMISALIGN(const XCOMParIMU_MISALIGN& par);

    void broadcast();

    uint16_t channel_ = 0;
    bool invalid_channel_ = false;
    bool init_done_ = false;
    bool active_ = false;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcast_;
    std::array<float, 3> offset_1_ = {0.f, 0.f, 0.f};
    std::array<float, 3> offset_2_ = {0.f, 0.f, 0.f};
    TransformStampedMsg tfs_msg_1_;
    TransformStampedMsg tfs_msg_2_;
    TransformStampedMsg tfs_msg_enc_vehicle_;
    TFMsg tf_msg_;
    rclcpp::Time gps_time_;
    bool data_updated_ = false;
    bool pardata_is_set_ = false;
    bool subscriber_added_ = false;
    std::atomic_bool connected_ = ATOMIC_VAR_INIT(false);
    std::atomic_bool success_ = ATOMIC_VAR_INIT(false);

    const std::string &ip_address_;
    int32_t ip_port_;
    Config::TimestampMode timestamp_mode_;
    int32_t leap_seconds_;
    uint16_t maintiming_;
    uint16_t prescaler_;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    xcom::XComState &xcom_;
};
