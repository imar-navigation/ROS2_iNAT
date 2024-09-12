#include "ixcom_driver_lc/modules/transform.hpp"
#include "ixcom_driver_lc/ixcom_driver_conf.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <ixcom_driver_lc/modules/utility.hpp>
#include <thread>

TransformStamped::TransformStamped(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                                   xcom::XComState &state,
                                   const std::string &ip_address,
                                   int32_t ip_port,
                                   Config::TimestampMode timestamp_mode,
                                   int32_t leap_seconds,
                                   uint16_t maintiming,
                                   uint16_t prescaler) :
    MessageHandler(&state),
//    xcom::CommandHandler(&state),
    xcom::ResponseHandler(&state),
    ip_address_(ip_address),
    ip_port_(ip_port),
    timestamp_mode_(timestamp_mode),
    leap_seconds_(leap_seconds),
    maintiming_(maintiming),
    prescaler_(prescaler),
    node_(node),
    xcom_(state) {
    std::thread th(&TransformStamped::init, this);
    th.detach();
}

TransformStamped::~TransformStamped() {
    cleanup();
}

void TransformStamped::cleanup() {
    xcom_.send_message(xcom_.get_xcomcmd_clearall());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    xcom_.set_rc(xcom::XComState::ReturnCode::Ok);
    xcom_.forced_exit();
}

void TransformStamped::init() {

    if(prescaler_ == 0) {
        RCLCPP_ERROR(node_->get_logger(), "%s", "transform: prescaler is 0");
    }
    if(maintiming_ == 0) {
        RCLCPP_ERROR(node_->get_logger(), "%s", "transform: maintiming is 0");
    }

    // connect to device
    xcom::TcpClient tcp_client(ip_address_, ip_port_);
    // define input source
    xcom::XcomClientReader tcp_reader(tcp_client);
    if(!tcp_reader.initialize()) {
        RCLCPP_ERROR(node_->get_logger(), "%s", "unable to initialize tcp_reader for tf2");
    }
    // define output source (optional)
    xcom::XcomClientWriter tcp_writer(tcp_client);
    if(!tcp_writer.initialize()) {
        RCLCPP_ERROR(node_->get_logger(), "%s", "unable to initialize tcp_writer for tf2");
    }

    xcom_.set_reader(&tcp_reader);
    xcom_.set_writer(&tcp_writer);
    xcom_.disable_forwarding();

    bool exit = false;
    RCLCPP_INFO(node_->get_logger(), "[%s] %s", "Transform", ("connecting to iNAT @ "
                                     + ip_address_ + ":"
                                     + std::to_string(ip_port_)
                                     + "...").c_str());
    while(!exit) {
//        RCLCPP_INFO(node_->get_logger(), "%s", ("connecting to iNAT @: "
//                                         + ip_address_ + ":"
//                                         + std::to_string(ip_port_)
//                                         + " channel " + std::to_string(channel_)
//                                         + "... for the tf2 broadcast").c_str());
        const auto cmd_open = xcom_.get_xcomcmd_open(channel_);
        tcp_writer.write(reinterpret_cast<const uint8_t *>(&cmd_open), sizeof(cmd_open));
        const auto rc = xcom_.process();
        if(rc == xcom::XComState::ReturnCode::InvalidResponse) {
            if(invalid_channel_) {
//                RCLCPP_INFO(node_->get_logger(), "%s", ("channel " + std::to_string(channel_) + " is in use, trying another one...").c_str());
            }
            channel_++;
            if(channel_ > 31) {
                RCLCPP_ERROR(node_->get_logger(), "%s", "no available channel found");
            }
            if(!tcp_reader.initialize()) {
                RCLCPP_ERROR(node_->get_logger(), "%s", "tcp reader initialization failed");
            }
        } else {
            exit = true;
        }
    }
}

//void TransformStamped::handle_command(uint16_t cmd_id, std::size_t frame_len, uint8_t *frame) {}

void TransformStamped::handle_response(XCOMResp response) {
    if(response == XCOMResp::OK) {
        invalid_channel_ = false;

        if(!init_done_) {

            RCLCPP_INFO(node_->get_logger(), "[%s] %s", "Transform",
                        ("connected to iNAT on channel " + std::to_string(channel_)).c_str());

            tfs_msg_1_.header.frame_id = tfs_msg_2_.header.frame_id = "inat_enclosure";
            tfs_msg_1_.child_frame_id = "primary_gnss_ant";
            tfs_msg_2_.child_frame_id = "secondary_gnss_ant";

            auto cmd_clearall = xcom_.get_xcomcmd_clearall();
            xcom_.send_message(cmd_clearall);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            int divider = (prescaler_ > 0) ? (maintiming_ / prescaler_) : 250;  // 1 Hz
            auto cmd_add_gnsssol = xcom_.get_xcomcmd_enablelog(XCOM_MSGID_GNSSLEVERARM, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC, divider);
            xcom_.send_message(cmd_add_gnsssol);

            broadcast_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
        }
        init_done_ = true;
    } else {
        if(response == XCOMResp::INVALIDCHANNEL) {
            invalid_channel_ = true;
        } else if(response == XCOMResp::LOGEXISTS) {
            RCLCPP_WARN(node_->get_logger(), "[%s] %s", "Transform", "requested log already exists");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "%s: %d", "invalid response received", static_cast<int>(response));
        }
        xcom_.set_rc(xcom::XComState::ReturnCode::InvalidResponse);
        xcom_.forced_exit();
    }
}

void TransformStamped::handle_xcom_msg(const XCOMmsg_GNSSLEVERARM &msg) {
    updateGNSSLEVERARM(msg);
}

void TransformStamped::activate() {
    active_ = true;
}

bool TransformStamped::success() {
    return success_;
}

void TransformStamped::updateGNSSLEVERARM(const XCOMmsg_GNSSLEVERARM& msg) {

    if((abs(offset_1_[0] - msg.primary_ant_offset[0]) > 0.01)
        || (abs(offset_1_[1] - msg.primary_ant_offset[1]) > 0.01)
        || (abs(offset_1_[2] - msg.primary_ant_offset[2]) > 0.01)) {

        tfs_msg_1_.transform.translation.x = msg.primary_ant_offset[0];
        tfs_msg_1_.transform.translation.y = msg.primary_ant_offset[1];
        tfs_msg_1_.transform.translation.z = msg.primary_ant_offset[2];

        offset_1_[0] = msg.primary_ant_offset[0];
        offset_1_[1] = msg.primary_ant_offset[1];
        offset_1_[2] = msg.primary_ant_offset[2];

        data_updated_ = true;
    }

    if((abs(offset_2_[0] - msg.secondary_ant_offset[0]) > 0.01)
        || (abs(offset_2_[1] - msg.secondary_ant_offset[1]) > 0.01)
        || (abs(offset_2_[2] - msg.secondary_ant_offset[2]) > 0.01)) {

        tfs_msg_2_.transform.translation.x = msg.secondary_ant_offset[0];
        tfs_msg_2_.transform.translation.y = msg.secondary_ant_offset[1];
        tfs_msg_2_.transform.translation.z = msg.secondary_ant_offset[2];

        offset_2_[0] = msg.secondary_ant_offset[0];
        offset_2_[1] = msg.secondary_ant_offset[1];
        offset_2_[2] = msg.secondary_ant_offset[2];

        data_updated_ = true;
    }

    if(data_updated_ || subscriber_added_) {
        gps_time_ = UpdateGPSTime(msg.header, leap_seconds_);
        broadcast();
    }
}

void TransformStamped::subscriberAdded() {
    subscriber_added_ = true;
}

void TransformStamped::broadcast() {

    tfs_msg_1_.header.stamp = (timestamp_mode_ == Config::TimestampMode::GPS) ? gps_time_ : node_->now();
    tfs_msg_2_.header.stamp = (timestamp_mode_ == Config::TimestampMode::GPS) ? gps_time_ : node_->now();
    tf_msg_.transforms.push_back(tfs_msg_1_);
    tf_msg_.transforms.push_back(tfs_msg_2_);

    if(active_) {
        data_updated_ = false;
        subscriber_added_ = false;
        broadcast_->sendTransform(tf_msg_.transforms);
        tf_msg_.transforms.clear();
    }
}
