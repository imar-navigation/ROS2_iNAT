#include "ixcom_driver_lc/modules/utility.hpp"
#include <math.h>

rclcpp::Time UpdateGPSTime(const XCOMHeader& header, int32_t leap_seconds)
{
    return rclcpp::Time(
        header.gps_time_sec + header.gps_week * 604800 + 315964800 - leap_seconds,
        header.gps_time_usec * 1000);
}

uint16_t calculateDividerForRate(float rate, uint16_t maintiming, uint16_t prescaler) {

    if((rate == 0) || (maintiming == 0) || (prescaler == 0)) {
        return 0;
    }

    uint16_t divider = ceil(maintiming / rate / prescaler);
    if (divider < 1) {
        return 0;
    }

    return divider;
}

int32_t calculateRateForDivider(uint16_t divider, uint16_t maintiming, uint16_t prescaler) {

    if((divider == 0) || (maintiming == 0) || (prescaler == 0)) {
        return 0;
    }

    int32_t rate = maintiming / prescaler / divider;
    return rate;
}
