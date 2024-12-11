#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ixcom/ixcom.h>

rclcpp::Time UpdateGPSTime(const XCOMHeader& header, int32_t leap_seconds);
double gpsTimeStamp(const double& time, int32_t leap_seconds);
uint8_t initFrameCounter();
uint8_t updateFrameCounter(uint8_t counter);

uint16_t calculateDividerForRate(float rate, uint16_t maintiming, uint16_t prescaler);
int32_t calculateRateForDivider(uint16_t divider, uint16_t maintiming, uint16_t prescaler);
