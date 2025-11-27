#include "service_adapter/service_adapter.hpp"

#include <memory>
#include <chrono>
using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<ServiceAdapter>());
    ServiceAdapter serviceAdapter;
    rclcpp::shutdown();
    return 0;
}

ServiceAdapter::ServiceAdapter() {
    run();
}
ServiceAdapter::~ServiceAdapter(){
    delete qos_;
}

void ServiceAdapter::run() {
  
    node_ = rclcpp::Node::make_shared("ixcom_service_adapter");

    qos_ = new rclcpp::SystemDefaultsQoS();
    sub_twistwithcovariancestamped_ = node_->create_subscription<TwistWithCovarianceStampedMsg>(TOPIC_TWISTWITHCOVARIANCESTAMPED, *qos_,
        std::bind(&ServiceAdapter::cb_twistwithcovariancestamped, this, std::placeholders::_1));
    sub_imu_ = node_->create_subscription<ImuMsg>(TOPIC_IMU, *qos_,
        std::bind(&ServiceAdapter::cb_imu, this, std::placeholders::_1));
    sub_magneticfield_ = node_->create_subscription<MagneticFieldMsg>(TOPIC_MAGNETICFIELD, *qos_,
       std::bind(&ServiceAdapter::cb_magneticfield, this, std::placeholders::_1));
    sub_navsatstatus_ = node_->create_subscription<NavSatStatusMsg>(TOPIC_NAVSATSTATUS, *qos_,
        std::bind(&ServiceAdapter::cb_navsatstatus, this, std::placeholders::_1));
    sub_navsatfixgnss_ = node_->create_subscription<NavSatFixMsg>(TOPIC_NAVSATFIXGNSS, *qos_,
        std::bind(&ServiceAdapter::cb_navsatfixgnss, this, std::placeholders::_1));
    sub_navsatfixins_ = node_->create_subscription<NavSatFixMsg>(TOPIC_NAVSATFIXINS, *qos_,
        std::bind(&ServiceAdapter::cb_navsatfixins, this, std::placeholders::_1));
    sub_odometry_ = node_->create_subscription<OdometryMsg>(TOPIC_ODOMETRY, *qos_,
        std::bind(&ServiceAdapter::cb_odometry, this, std::placeholders::_1));
    sub_posewithcovariancestamped_ = node_->create_subscription<PoseWithCovarianceStampedMsg>(TOPIC_POSEWITHCOVARIANCESTAMPED, *qos_,
        std::bind(&ServiceAdapter::cb_posewithcovariancestamped, this, std::placeholders::_1));
    sub_timereference_ = node_->create_subscription<TimeReferenceMsg>(TOPIC_TIMEREFERENCE, *qos_,
        std::bind(&ServiceAdapter::cb_timereference, this, std::placeholders::_1));
    sub_twiststamped_ = node_->create_subscription<TwistStampedMsg>(TOPIC_TWISTSTAMPED, *qos_,
        std::bind(&ServiceAdapter::cb_twiststamped, this, std::placeholders::_1));
    
    cl_extposllh_ = node_->create_client<if_extaid_posllh>(CL_EXTPOSLLH);
    rq_send_extposllh_ = std::make_shared<if_extaid_posllh::Request>();
    cl_extposecef_ = node_->create_client<if_extaid_posecef>(CL_EXTPOSECEF);
    rq_send_extposecef_ = std::make_shared<if_extaid_posecef::Request>();
    cl_extposutm_ = node_->create_client<if_extaid_posutm>(CL_EXTPOSUTM);
    rq_send_extposutm_ = std::make_shared<if_extaid_posutm::Request>();
    cl_extposmgrs_ = node_->create_client<if_extaid_posmgrs>(CL_EXTPOSMGRS);
    rq_send_extposmgrs_ = std::make_shared<if_extaid_posmgrs::Request>();
    cl_exthdg_ = node_->create_client<if_extaid_hdg>(CL_EXTHDG);
    rq_send_exthdg_ = std::make_shared<if_extaid_hdg::Request>();
    cl_extvel_ = node_->create_client<if_extaid_vel>(CL_EXTVEL);
    rq_send_extvel_ = std::make_shared<if_extaid_vel::Request>();
    cl_extvelbody_ = node_->create_client<if_extaid_velbody>(CL_EXTVELBODY);
    rq_send_extvelbody_ = std::make_shared<if_extaid_velbody::Request>();
    cl_extheight_ = node_->create_client<if_extaid_height>(CL_EXTHEIGHT);
    rq_send_extheight_ = std::make_shared<if_extaid_height::Request>();

    rclcpp::spin(node_);
}

void ServiceAdapter::cb_twistwithcovariancestamped(const TwistWithCovarianceStampedMsg& msg) {
    RCLCPP_INFO(node_->get_logger(), "Received %s, calling %s service.",
    std::string("TwistWithCovarianceStampedMsg").c_str(), CL_EXTVELBODY.c_str());

    std::unique_lock<std::mutex> lk(mx_extvelbody_);
    rq_send_extvelbody_->time_stamp = get_timestamp(msg.header.stamp.sec, msg.header.stamp.nanosec);
    rq_send_extvelbody_->time_mode = 0;
    rq_send_extvelbody_->velocity[0] = msg.twist.twist.linear.x;
    rq_send_extvelbody_->velocity[1] = msg.twist.twist.linear.y;
    rq_send_extvelbody_->velocity[2] = msg.twist.twist.linear.z;
    rq_send_extvelbody_->velocity_stddev[0] = sqrt(msg.twist.covariance.at(0));
    rq_send_extvelbody_->velocity_stddev[1] = sqrt(msg.twist.covariance.at(1));
    rq_send_extvelbody_->velocity_stddev[2] = sqrt(msg.twist.covariance.at(2));
    rq_send_extvelbody_->lever_arm[0] = 0.0;
    rq_send_extvelbody_->lever_arm[1] = 0.0;
    rq_send_extvelbody_->lever_arm[2] = 0.0;
    rq_send_extvelbody_->lever_arm_stddev[0] = 0.1;
    rq_send_extvelbody_->lever_arm_stddev[1] = 0.1;
    rq_send_extvelbody_->lever_arm_stddev[2] = 0.1;
    lk.unlock();
}

void ServiceAdapter::cb_imu(const ImuMsg& msg) {
    RCLCPP_INFO(node_->get_logger(), "Received %s", std::string("ImuMsg").c_str());
}

void ServiceAdapter::cb_magneticfield(const MagneticFieldMsg& msg) {
    RCLCPP_INFO(node_->get_logger(), "Received %s", std::string("MagneticFieldMsg").c_str());
}

void ServiceAdapter::cb_navsatstatus(const NavSatStatusMsg& msg) {
    RCLCPP_INFO(node_->get_logger(), "Received %s", std::string("NavSatStatusMsg").c_str());
}

void ServiceAdapter::cb_navsatfixgnss(const NavSatFixMsg& msg) {
    RCLCPP_INFO(node_->get_logger(), "Received %s, calling %s service.",
        std::string("NavSatFixMsg").c_str(), CL_EXTPOSLLH.c_str());

    std::unique_lock<std::mutex> lk(mx_extposllh_);
    rq_send_extposllh_->time_stamp = get_timestamp(msg.header.stamp.sec, msg.header.stamp.nanosec);
    rq_send_extposllh_->time_mode = 0;
    rq_send_extposllh_->position[0] = get_rad(msg.longitude);
    rq_send_extposllh_->position[1] = get_rad(msg.latitude);
    rq_send_extposllh_->position[2] = get_rad(msg.altitude);
    rq_send_extposllh_->position_stddev[0] = 0.1;
    rq_send_extposllh_->position_stddev[1] = 0.1;
    rq_send_extposllh_->position_stddev[2] = 0.1;
    rq_send_extposllh_->lever_arm[0] = 0.0;
    rq_send_extposllh_->lever_arm[1] = 0.0;
    rq_send_extposllh_->lever_arm[2] = 0.0;
    rq_send_extposllh_->lever_arm_stddev[0] = 0.1;
    rq_send_extposllh_->lever_arm_stddev[1] = 0.1;
    rq_send_extposllh_->lever_arm_stddev[2] = 0.1;
    rq_send_extposllh_->enable_msl_altitude = 0;
    lk.unlock();

    cl_extposllh_->async_send_request(rq_send_extposllh_,
        std::bind(&ServiceAdapter::res_cb_extposllh, this, std::placeholders::_1));
}

void ServiceAdapter::cb_navsatfixins(const NavSatFixMsg& msg) {
    RCLCPP_INFO(node_->get_logger(), "Received %s, calling %s service.",
        std::string("NavSatFixMsg").c_str(), CL_EXTPOSLLH.c_str());

    std::unique_lock<std::mutex> lk(mx_extposllh_);
    rq_send_extposllh_->time_stamp = get_timestamp(msg.header.stamp.sec, msg.header.stamp.nanosec);
    rq_send_extposllh_->time_mode = 0;
    rq_send_extposllh_->position[0] = get_rad(msg.longitude);
    rq_send_extposllh_->position[1] = get_rad(msg.latitude);
    rq_send_extposllh_->position[2] = get_rad(msg.altitude);
    rq_send_extposllh_->position_stddev[0] = 0.1;
    rq_send_extposllh_->position_stddev[1] = 0.1;
    rq_send_extposllh_->position_stddev[2] = 0.1;
    rq_send_extposllh_->lever_arm[0] = 0.0;
    rq_send_extposllh_->lever_arm[1] = 0.0;
    rq_send_extposllh_->lever_arm[2] = 0.0;
    rq_send_extposllh_->lever_arm_stddev[0] = 0.1;
    rq_send_extposllh_->lever_arm_stddev[1] = 0.1;
    rq_send_extposllh_->lever_arm_stddev[2] = 0.1;
    rq_send_extposllh_->enable_msl_altitude = 0;
    lk.unlock();

    cl_extposllh_->async_send_request(rq_send_extposllh_,
        std::bind(&ServiceAdapter::res_cb_extposllh, this, std::placeholders::_1));
}

void ServiceAdapter::cb_odometry(const OdometryMsg& msg) {
    RCLCPP_INFO(node_->get_logger(), "Received %s", std::string("OdometryMsg").c_str());
}

void ServiceAdapter::cb_posewithcovariancestamped(const PoseWithCovarianceStampedMsg& msg) {
    RCLCPP_INFO(node_->get_logger(), "Received %s", std::string("PoseWithCovarianceStampedMsg").c_str());
}

void ServiceAdapter::cb_timereference(const TimeReferenceMsg& msg) {
    RCLCPP_INFO(node_->get_logger(), "Received %s", std::string("TimeReferenceMsg").c_str());
}

void ServiceAdapter::cb_twiststamped(const TwistStampedMsg& msg) {
    RCLCPP_INFO(node_->get_logger(), "Received %s, calling %s service.",
        std::string("TwistStampedMsg").c_str(), CL_EXTVEL.c_str());

    std::unique_lock<std::mutex> lk(mx_extvel_);
    rq_send_extvel_->time_stamp = get_timestamp(msg.header.stamp.sec, msg.header.stamp.nanosec);
    rq_send_extvel_->time_mode = 0;
    // NED in iXCOM is END (see ICD and ROS2_iNAT README)
    rq_send_extvel_->velocity[0] = msg.twist.linear.y;  // east
    rq_send_extvel_->velocity[1] = msg.twist.linear.x;  // north
    rq_send_extvel_->velocity[2] = msg.twist.linear.z;  // down
    rq_send_extvel_->velocity_stddev[0] = 0.1;          // east
    rq_send_extvel_->velocity_stddev[1] = 0.1;          // north
    rq_send_extvel_->velocity_stddev[2] = 0.1;          // down
    lk.unlock();

    cl_extvel_->async_send_request(rq_send_extvel_,
        std::bind(&ServiceAdapter::res_cb_extvel, this, std::placeholders::_1));

    // cl_extvel_->async_send_request(rq_send_extvel_,
    //     std::bind(&ServiceAdapter::res_cb, this, std::placeholders::_1, std::placeholders::_2));
}

void ServiceAdapter::res_cb_extposllh(rclcpp::Client<if_extaid_posllh>::SharedFuture result) {
    RCLCPP_INFO(node_->get_logger(), "%s success: %s", CL_EXTPOSLLH.c_str(), result.get()->success ? "OK" : "FAIL");
}

void ServiceAdapter::res_cb_extposecef(rclcpp::Client<if_extaid_posecef>::SharedFuture result){
    RCLCPP_INFO(node_->get_logger(), "%s success: %s", CL_EXTPOSECEF.c_str(), result.get()->success ? "OK" : "FAIL");
}

void ServiceAdapter::res_cb_extposutm(rclcpp::Client<if_extaid_posutm>::SharedFuture result){
    RCLCPP_INFO(node_->get_logger(), "%s success: %s", CL_EXTPOSUTM.c_str(), result.get()->success ? "OK" : "FAIL");
}

void ServiceAdapter::res_cb_extposmgrs(rclcpp::Client<if_extaid_posmgrs>::SharedFuture result){
    RCLCPP_INFO(node_->get_logger(), "%s success: %s", CL_EXTPOSMGRS.c_str(), result.get()->success ? "OK" : "FAIL");
}

void ServiceAdapter::res_cb_exthdg(rclcpp::Client<if_extaid_hdg>::SharedFuture result){
    RCLCPP_INFO(node_->get_logger(), "%s success: %s", CL_EXTHDG.c_str(), result.get()->success ? "OK" : "FAIL");
}

void ServiceAdapter::res_cb_extvel(rclcpp::Client<if_extaid_vel>::SharedFuture result) {
    RCLCPP_INFO(node_->get_logger(), "%s success: %s", CL_EXTVEL.c_str(), result.get()->success ? "OK" : "FAIL");
}

void ServiceAdapter::res_cb_extvelbody(rclcpp::Client<if_extaid_velbody>::SharedFuture result){
    RCLCPP_INFO(node_->get_logger(), "%s success: %s", CL_EXTVELBODY.c_str(), result.get()->success ? "OK" : "FAIL");
}

void ServiceAdapter::res_cb_extheight(rclcpp::Client<if_extaid_height>::SharedFuture result){
    RCLCPP_INFO(node_->get_logger(), "%s success: %s", CL_EXTHEIGHT.c_str(), result.get()->success ? "OK" : "FAIL");
}


