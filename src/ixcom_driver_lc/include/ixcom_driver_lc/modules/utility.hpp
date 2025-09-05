#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ixcom/ixcom.h>
#include <ixcom_driver_lc/ixcom_driver_conf.hpp>

rclcpp::Time UpdateGPSTime(const XCOMHeader& header, int32_t leap_seconds);
double gpsTimeStamp(const double& time, int32_t leap_seconds);
uint8_t initFrameCounter();
uint8_t updateFrameCounter(uint8_t counter);

uint16_t calculateDividerForRate(float rate, uint16_t maintiming, uint16_t prescaler);
int32_t calculateRateForDivider(uint16_t divider, uint16_t maintiming, uint16_t prescaler);

bool operator!=(const Config::ImudataMode& lhs, const XCOM_PARDAT_IMU_Mode& rhs);
const std::string imu_mode_str(const XCOM_PARDAT_IMU_Mode imu_mode_conf);
const XCOM_PARDAT_IMU_Mode get_imu_mode_conf(const Config::ImudataMode imu_mode_conf);

void rclsleep(float t);
