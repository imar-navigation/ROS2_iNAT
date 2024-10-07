/*.*******************************************************************
 FILENAME: xcom_parser.cc
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
#include "ixcom/xcom_parser.h"
#include <cstring>
namespace xcom {
XComParser::XComParser() noexcept { init_parser(); }
uint8_t* XComParser::get_payload() noexcept { return &_rx_state.payload[0]; }
void XComParser::init_parser() noexcept {
    _rx_state.state         = XComSync;
    _rx_state.rx_cpy_idx    = 0;
    _rx_state.rx_idx        = 0;
    _rx_state.bytes_to_read = 1;
    std::memset(reinterpret_cast<void*>(_rx_state.payload), 0, MaxMessageSize);
}
XComParser::ParserCode XComParser::process_byte(uint8_t rxByte) noexcept {
    XComParser::ParserCode ret_val = XComParser::ParserCode::Running;
    if(_rx_state.rx_cpy_idx < MaxMessageSize) {
        _rx_state.payload[_rx_state.rx_cpy_idx] = rxByte;
        _rx_state.rx_cpy_idx++;
    }
    switch(_rx_state.state) {
        case XComSync:
            if(rxByte == _sync_byte) {
                _rx_state.state         = XComMsgSync;
                _rx_state.bytes_to_read = 5;
            } else {
                _rx_state.rx_cpy_idx    = 0;
                _rx_state.state         = XComSync;
                _rx_state.bytes_to_read = 5;
            }
            break;
        case XComMsgSync:
            _rx_state.state  = XComFrameCnt;
            _rx_state.msg_id = rxByte;
            break;
        case XComFrameCnt:
            _rx_state.state = XComReserved;
            break;
        case XComReserved:
            _rx_state.state = XComMsgLenLsb;
            break;
        case XComMsgLenLsb:
            _rx_state.msg_length = static_cast<uint16_t>(rxByte);
            _rx_state.state      = XComMsgLenMsb;
            break;
        case XComMsgLenMsb:
            _rx_state.msg_length |= static_cast<uint16_t>(rxByte) << 8U;
            if(_rx_state.msg_length > MaxMessageSize) {
                _rx_state.state         = XComSync;
                _rx_state.rx_cpy_idx    = 0;
                _rx_state.rx_idx        = 0;
                _rx_state.bytes_to_read = 1;
                ret_val                 = XComParser::ParserCode::InvalidLength;
            } else {
                _rx_state.bytes_to_read = static_cast<uint8_t>(_rx_state.msg_length - 6);
                _rx_state.state         = XComPayload;
            }
            break;
        case XComPayload:
            if((++_rx_state.rx_idx >= (_rx_state.msg_length - 2 - 6))) {
                _rx_state.state = XComCrcLsb;
            }
            if(_rx_state.rx_idx >= MaxMessageSize) {
                _rx_state.state      = XComSync;
                _rx_state.rx_cpy_idx = 0;
                _rx_state.rx_idx     = 0;
                ret_val              = XComParser::ParserCode::InvalidLength;
            }
            break;
        case XComCrcLsb:
            _rx_state.crc   = static_cast<uint16_t>(rxByte);
            _rx_state.state = XComCrcMsb;
            break;
        case XComCrcMsb:
            _rx_state.crc |= static_cast<uint16_t>(rxByte) << 8U;
            const uint16_t crc16 = _crc16.process(_rx_state.payload, static_cast<std::size_t>(_rx_state.rx_cpy_idx) - 2);
#ifdef USE_FUZZING
            ret_val = XComParser::ParserCode::Ok;  // skip CRC check
#else
            ret_val = _rx_state.crc == crc16 ? XComParser::ParserCode::Ok : XComParser::ParserCode::CrcError;
#endif
            _rx_state.state         = XComSync;
            _rx_state.rx_cpy_idx    = 0;
            _rx_state.rx_idx        = 0;
            _rx_state.bytes_to_read = 1;
            break;
    }
    return ret_val;
}
bool XComParser::is_cmd_open(int& channel) const noexcept {
    const auto* cmd_id      = reinterpret_cast<const uint16_t*>(&_rx_state.payload[sizeof(XCOMHeader)]);
    const auto* cmd         = reinterpret_cast<const uint16_t*>(&_rx_state.payload[sizeof(XCOMHeader) + 4]);
    const auto* cmd_channel = reinterpret_cast<const uint16_t*>(&_rx_state.payload[sizeof(XCOMHeader) + 6]);
    if(_rx_state.msg_length != sizeof(XCOMCmd_XCOM)) {
        return false;
    }
    if(_rx_state.msg_id == XCOM_MSGID_COMMAND) {
        if(*cmd_id == XCOM_CMDID_XCOM) {
            if(*cmd == XCOM_CMDXCOM_OPEN) {
                channel = static_cast<int>(*cmd_channel);
                return true;
            } else {
                channel = -1;
                return false;
            }
        } else {
            channel = -1;
            return false;
        }
    } else {
        channel = -1;
        return false;
    }
}
bool XComParser::is_cmd_close(int channel) const noexcept {
    const auto* cmd_id      = reinterpret_cast<const uint16_t*>(&_rx_state.payload[sizeof(XCOMHeader)]);
    const auto* cmd         = reinterpret_cast<const uint16_t*>(&_rx_state.payload[sizeof(XCOMHeader) + 4]);
    const auto* cmd_channel = reinterpret_cast<const uint16_t*>(&_rx_state.payload[sizeof(XCOMHeader) + 6]);
    if(_rx_state.msg_length != sizeof(XCOMCmd_XCOM)) {
        return false;
    }
    if(_rx_state.msg_id == XCOM_MSGID_COMMAND) {
        if(*cmd_id == XCOM_CMDID_XCOM) {
            if(*cmd == XCOM_CMDXCOM_CLOSE) {
                return channel == static_cast<int>(*cmd_channel);
            } else {
                return false;
            }
        } else {
            return false;
        }
    } else {
        return false;
    }
}
bool XComParser::is_param() const noexcept { return _rx_state.msg_id == XCOM_MSGID_PARAMETER; }
bool XComParser::is_cmd() const noexcept { return _rx_state.msg_id == XCOM_MSGID_COMMAND; }
uint16_t XComParser::get_param_id() const noexcept {
    const auto* param_id = reinterpret_cast<const uint16_t*>(&_rx_state.payload[sizeof(XCOMHeader)]);
    return *param_id;
}
std::size_t XComParser::get_payload_length() const noexcept { return static_cast<std::size_t>(_rx_state.msg_length); }
uint16_t XComParser::get_cmd_id() const noexcept {
    const auto* cmd_id = reinterpret_cast<const uint16_t*>(&_rx_state.payload[sizeof(XCOMHeader)]);
    return *cmd_id;
}
uint8_t XComParser::get_msg_id() const noexcept { return _rx_state.msg_id; }
uint16_t XComParser::get_param_id(const uint8_t* payload) noexcept {
    const auto* param_id = reinterpret_cast<const uint16_t*>(&payload[sizeof(XCOMHeader)]);
    return *param_id;
}
bool XComParser::is_param_request(const uint8_t* payload) noexcept {
    if(payload[1] != XCOM_MSGID_PARAMETER) {
        return false;
    }
    return static_cast<bool>(payload[sizeof(XCOMHeader) + 3]);
}
bool XComParser::is_param(const uint8_t* payload) noexcept { return static_cast<bool>(payload[1] == XCOM_MSGID_PARAMETER); }
bool XComParser::is_msg() const noexcept { return (_rx_state.msg_id < XCOM_MSGID_COMMAND); }
void XComParser::set_sync_byte(uint8_t sync_byte) noexcept { _sync_byte = sync_byte; }
uint8_t XComParser::get_sync_byte() const noexcept { return _sync_byte; }
}  // namespace xcom