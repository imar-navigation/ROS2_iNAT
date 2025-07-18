#include "ixcom_driver_lc/modules/utility.hpp"
#include <math.h>
#include <random>

rclcpp::Time UpdateGPSTime(const XCOMHeader& header, int32_t leap_seconds) {
    return rclcpp::Time(
        header.gps_time_sec + header.gps_week * 604800 + 315964800 - leap_seconds,
        header.gps_time_usec * 1000);
}

// double gpsTimeStamp(rclcpp::Time time, int32_t leap_seconds) {
double gpsTimeStamp(const double& time, int32_t leap_seconds) {
    // int s = time.seconds() - 315964800 + leap_seconds;
    // int us = std::round(time.nanoseconds() / 1000);

    // uint32_t t_s = static_cast<uint32_t>(time);
    // uint32_t t_ns = (time - t_s) * 1e9;
    // uint32_t s = t_s - 315964800 + leap_seconds;
    // uint32_t us = std::round(t_ns / 1000);
    // return s + us * 1e-6;

    double s = time - 315964800.0 + leap_seconds; // GPS epoch starts at 1980-01-06
    double tow = fmod(s, 604800.0); // only seconds of week remain
    return tow;
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

uint8_t initFrameCounter() {
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist6(0, 255);
    return dist6(rng);
}

uint8_t updateFrameCounter(uint8_t counter) {
    if(counter == 255)
        return 0;
    return ++counter;
}
