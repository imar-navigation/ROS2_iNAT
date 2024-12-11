#pragma once

#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "libxcom/app/xcomhandler.hpp"

#include "ixcom_driver_lc/ixcom_driver_conf.hpp"
#include "ixcom_driver_lc/modules/transform.hpp"
#include "ixcom_driver_lc/modules/imu.hpp"
#include "ixcom_driver_lc/modules/navsatstatus.hpp"
#include "ixcom_driver_lc/modules/nav_gnss.hpp"
#include "ixcom_driver_lc/modules/nav_ins.hpp"
#include "ixcom_driver_lc/modules/time_reference.hpp"
#include "ixcom_driver_lc/modules/magnetic_field.hpp"
#include "ixcom_driver_lc/modules/odometry.hpp"
#include "ixcom_driver_lc/modules/pose_ecef.hpp"
#include "ixcom_driver_lc/modules/twist.hpp"
#include "ixcom_driver_lc/modules/srv_extaid.hpp"

class DriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
	DriverNode();
    ~DriverNode();

	CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
	CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
	CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
	CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
	CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;


protected:
	using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

	void clearModules();

	Config::UniquePtr conf_;
    std::unique_ptr<XcomHandler> client_;
    xcom::XComState xcom_tf_;
    TransformStamped::SharedPtr transformstamped_;
    xcom::XComState xcom_srv_extaid_;
    SrvExtAid::SharedPtr srvextaid_;

    rclcpp::QoS *qos_;

    std::string imu_topic_name_;
    std::string navsatstatus_topic_name_;
    std::string navgnss_topic_name_;
    std::string navins_topic_name_;
    std::string timeref_topic_name_;
    std::string magfield_topic_name_;
    std::string odometry_topic_name_;
    std::string posewithcovstamped_topic_name_;
    std::string twiststamped_topic_name_;

    xcom::XComState xcom_imu_;
    xcom::XComState xcom_navsatstatus_;
    xcom::XComState xcom_navgnss_;
    xcom::XComState xcom_navins_;
    xcom::XComState xcom_timeref_;
    xcom::XComState xcom_magfield_;
    xcom::XComState xcom_odometry_;
    xcom::XComState xcom_posewithcovstamped_;
    xcom::XComState xcom_twiststamped_;

    std::shared_ptr<IMU> imu_;
    std::shared_ptr<NavSatStatus> navsatstatus_;
    std::shared_ptr<NavGNSS> navgnss_;
    std::shared_ptr<NavINS> navins_;
    std::shared_ptr<TimeReference> timeref_;
    std::shared_ptr<MagneticField> magfield_;
    std::shared_ptr<Odometry> odometry_;
    std::shared_ptr<PoseWithCovarianceStamped> posewithcovstamped_;
    std::shared_ptr<TwistStamped> twiststamped_;

private:
    void declare_parameters();
    void get_parameters();
    void build_broadcast_transformstamped();
    void build_srv_extaid();
    bool build_topic_imu();
    bool build_topic_navsatstatus();
    bool build_topic_navsatfix_gnss();
    bool build_topic_navsatfix_ins();
    bool build_topic_timereference();
    bool build_topic_magneticfield();
    bool build_topic_odometry();
    bool build_topic_posewithcovariancestamped();
    bool build_topic_twiststamped();
};
