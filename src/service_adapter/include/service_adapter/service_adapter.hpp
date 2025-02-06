#pragma once

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include <mutex>
#include <cmath>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "interfaces/srv/ext_aid_pos_llh.hpp"
#include "interfaces/srv/ext_aid_pos_ecef.hpp"
#include "interfaces/srv/ext_aid_pos_utm.hpp"
#include "interfaces/srv/ext_aid_pos_mgrs.hpp"
#include "interfaces/srv/ext_aid_hdg.hpp"
#include "interfaces/srv/ext_aid_vel.hpp"
#include "interfaces/srv/ext_aid_vel_body.hpp"
#include "interfaces/srv/ext_aid_height.hpp"

class ServiceAdapter {
    public:
        ServiceAdapter();
        ~ServiceAdapter();
    private:
        using ImuMsg = sensor_msgs::msg::Imu;
        using MagneticFieldMsg = sensor_msgs::msg::MagneticField;
        using NavSatStatusMsg = sensor_msgs::msg::NavSatStatus;
        using NavSatFixMsg = sensor_msgs::msg::NavSatFix;
        using OdometryMsg = nav_msgs::msg::Odometry;
        using PoseWithCovarianceStampedMsg = geometry_msgs::msg::PoseWithCovarianceStamped;
        using TimeReferenceMsg = sensor_msgs::msg::TimeReference;
        using TwistStampedMsg = geometry_msgs::msg::TwistStamped;

        using if_extaid_posllh = interfaces::srv::ExtAidPosLlh;
        using if_extaid_posecef = interfaces::srv::ExtAidPosEcef;
        using if_extaid_posutm = interfaces::srv::ExtAidPosUtm;
        using if_extaid_posmgrs = interfaces::srv::ExtAidPosMgrs;
        using if_extaid_hdg = interfaces::srv::ExtAidHdg;
        using if_extaid_vel = interfaces::srv::ExtAidVel;
        using if_extaid_velbody = interfaces::srv::ExtAidVelBody;
        using if_extaid_height = interfaces::srv::ExtAidHeight;

        const std::string TOPIC_IMU {"Imu"};
        const std::string TOPIC_MAGNETICFIELD {"MagneticField"};
        const std::string TOPIC_NAVSATSTATUS {"NavSatStatus"};
        const std::string TOPIC_NAVSATFIXGNSS {"NavSatFix_GNSS"};
        const std::string TOPIC_NAVSATFIXINS {"NavSatFix_INS"};
        const std::string TOPIC_ODOMETRY {"Odometry"};
        const std::string TOPIC_POSEWITHCOVARIANCESTAMPED {"PoseWithCovarianceStamped"};
        const std::string TOPIC_TIMEREFERENCE {"TimeReference"};
        const std::string TOPIC_TWISTSTAMPED {"TwistStamped"};

        const std::string CL_EXTPOSLLH {"ext_position_llh"};
        const std::string CL_EXTPOSECEF {"ext_position_ecef"};
        const std::string CL_EXTPOSUTM {"ext_position_utm"};
        const std::string CL_EXTPOSMGRS {"ext_position_mgrs"};
        const std::string CL_EXTHDG {"ext_heading"};
        const std::string CL_EXTVEL {"ext_velocity"};
        const std::string CL_EXTVELBODY {"ext_velocity_body"};
        const std::string CL_EXTHEIGHT {"ext_height"};

        // ImuMsg imu_msg_;
        // MagneticFieldMsg magneticfield_msg_;
        // NavSatStatusMsg navsatstatus_msg_;
        // NavSatFixMsg navsatfix_msg_;
        // OdometryMsg odometry_msg_;
        // PoseWithCovarianceStampedMsg posewithcovariancestamped_msg_;
        // TimeReferenceMsg timereference_msg_;
        // TwistStampedMsg twiststamped_msg_;

        rclcpp::Subscription<ImuMsg>::SharedPtr sub_imu_;
        rclcpp::Subscription<MagneticFieldMsg>::SharedPtr sub_magneticfield_;
        rclcpp::Subscription<NavSatStatusMsg>::SharedPtr sub_navsatstatus_;
        rclcpp::Subscription<NavSatFixMsg>::SharedPtr sub_navsatfixgnss_;
        rclcpp::Subscription<NavSatFixMsg>::SharedPtr sub_navsatfixins_;
        rclcpp::Subscription<OdometryMsg>::SharedPtr sub_odometry_;
        rclcpp::Subscription<PoseWithCovarianceStampedMsg>::SharedPtr sub_posewithcovariancestamped_;
        rclcpp::Subscription<TimeReferenceMsg>::SharedPtr sub_timereference_;
        rclcpp::Subscription<TwistStampedMsg>::SharedPtr sub_twiststamped_;
        
        void run();
        void cb_imu(const ImuMsg& msg);
        void cb_magneticfield(const MagneticFieldMsg& msg);
        void cb_navsatstatus(const NavSatStatusMsg& msg);
        void cb_navsatfixgnss(const NavSatFixMsg& msg);
        void cb_navsatfixins(const NavSatFixMsg& msg);
        void cb_odometry(const OdometryMsg& msg);
        void cb_posewithcovariancestamped(const PoseWithCovarianceStampedMsg& msg);
        void cb_timereference(const TimeReferenceMsg& msg);
        void cb_twiststamped(const TwistStampedMsg& msg);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<if_extaid_posllh>::SharedPtr cl_extposllh_;
        rclcpp::Client<if_extaid_posecef>::SharedPtr cl_extposecef_;
        rclcpp::Client<if_extaid_posutm>::SharedPtr cl_extposutm_;
        rclcpp::Client<if_extaid_posmgrs>::SharedPtr cl_extposmgrs_;
        rclcpp::Client<if_extaid_hdg>::SharedPtr cl_exthdg_;
        rclcpp::Client<if_extaid_vel>::SharedPtr cl_extvel_;
        rclcpp::Client<if_extaid_velbody>::SharedPtr cl_extvelbody_;
        rclcpp::Client<if_extaid_height>::SharedPtr cl_extheight_;

        std::shared_ptr<if_extaid_posllh::Request> rq_send_extposllh_;
        std::shared_ptr<if_extaid_posecef::Request> rq_send_extposecef_;
        std::shared_ptr<if_extaid_posutm::Request> rq_send_extposutm_;
        std::shared_ptr<if_extaid_posmgrs::Request> rq_send_extposmgrs_;
        std::shared_ptr<if_extaid_hdg::Request> rq_send_exthdg_;
        std::shared_ptr<if_extaid_vel::Request> rq_send_extvel_;
        std::shared_ptr<if_extaid_velbody::Request> rq_send_extvelbody_;
        std::shared_ptr<if_extaid_height::Request> rq_send_extheight_;

        void res_cb_extposllh(rclcpp::Client<if_extaid_posllh>::SharedFuture future);
        void res_cb_extposecef(rclcpp::Client<if_extaid_posecef>::SharedFuture future);
        void res_cb_extposutm(rclcpp::Client<if_extaid_posutm>::SharedFuture future);
        void res_cb_extposmgrs(rclcpp::Client<if_extaid_posmgrs>::SharedFuture future);
        void res_cb_exthdg(rclcpp::Client<if_extaid_hdg>::SharedFuture future);
        void res_cb_extvel(rclcpp::Client<if_extaid_vel>::SharedFuture future);
        void res_cb_extvelbody(rclcpp::Client<if_extaid_velbody>::SharedFuture future);
        void res_cb_extheight(rclcpp::Client<if_extaid_height>::SharedFuture future);

        // template<typename T, typename R>
        // void res_cb(T type, R *result) {
        //     RCLCPP_INFO(node_->get_logger(),
        //         "%s success: %s", type.c_str(),
        //         result.get()->success ? "OK" : "FAILURE");
        // }

        std::mutex mx_extposllh_;
        std::mutex mx_extposecef_;
        std::mutex mx_extposutm_;
        std::mutex mx_extposmgrs_;
        std::mutex mx_exthdg_;
        std::mutex mx_extvel_;
        std::mutex mx_extvelbody_;
        std::mutex mx_extheight_;

        rclcpp::QoS *qos_;

        inline double get_timestamp(int32_t s, uint32_t ns) {return s + ns * 1e-9;}
        inline double get_rad(double deg) {return deg * M_PI / 180;}
};
