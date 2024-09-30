#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ixcom/ixcom.h>

rclcpp::Time UpdateGPSTime(const XCOMHeader& header, int32_t leap_seconds);

uint16_t calculateDividerForRate(float rate, uint16_t maintiming, uint16_t prescaler);
int32_t calculateRateForDivider(uint16_t divider, uint16_t maintiming, uint16_t prescaler);
