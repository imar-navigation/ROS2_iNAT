/*.*******************************************************************
 FILENAME: crc16.cpp
 **********************************************************************
 *  PROJECT: ROS2_iNAT
 *
 *
 *---------------------------------------------------------------------
 * 	Copyright 2021, iMAR Navigation
 *---------------------------------------------------------------------
 * 	MODULE DESCRIPTION:
 *
 ---------------------------------------------------------------------*/
#include <ixcom/crc16.h>
namespace xcom {
Crc16::Crc16(uint16_t start_val) noexcept { _start_value = start_val; }
uint16_t Crc16::process(const uint8_t* pByte, std::size_t len) const noexcept {
    uint16_t crc = _start_value;
    while(len-- > 0) {
        crc = static_cast<uint16_t>((crc << 8) ^ CrcTable[(crc >> 8) ^ *pByte++]);
    }
    return crc;
}
}  // namespace xcom