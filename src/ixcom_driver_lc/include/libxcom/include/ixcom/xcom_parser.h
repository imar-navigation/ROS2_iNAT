/*.*******************************************************************
 FILENAME: xcom_parser.h
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
#ifndef LIBXCOM_PROTOCOL_XCOM_XCOMPARSER_H_
#define LIBXCOM_PROTOCOL_XCOM_XCOMPARSER_H_
#include <cstdint>
#include <ixcom/XCOMdat.h>
#include <ixcom/crc16.h>
namespace xcom {
class XComParser {
public:
    XComParser() noexcept;
    ~XComParser() = default;
    enum class ParserCode : int {
        Ok,
        Running,
        CrcError,
        InvalidLength
    };
    ParserCode process_byte(uint8_t rxByte) noexcept;
    bool is_cmd_open(int& channel) const noexcept;
    [[nodiscard]] bool is_cmd_close(int channel) const noexcept;
    [[nodiscard]] bool is_param() const noexcept;
    [[nodiscard]] uint16_t get_param_id() const noexcept;
    static uint16_t get_param_id(const uint8_t* payload) noexcept;
    static bool is_param_request(const uint8_t* payload) noexcept;
    static bool is_param(const uint8_t* payload) noexcept;
    [[nodiscard]] uint16_t get_cmd_id() const noexcept;
    uint8_t* get_payload() noexcept;
    [[nodiscard]] std::size_t get_payload_length() const noexcept;
    [[nodiscard]] bool is_cmd() const noexcept;
    [[nodiscard]] bool is_msg() const noexcept;
    [[nodiscard]] uint8_t get_msg_id() const noexcept;
private:
    static constexpr int MaxMessageSize = XCOM_MAX_MESSAGE_LENGTH;
    typedef enum {
        XComSync,
        XComMsgSync,
        XComFrameCnt,
        XComReserved,
        XComMsgLenLsb,
        XComMsgLenMsb,
        XComPayload,
        XComCrcLsb,
        XComCrcMsb
    } XCOMParserStates;
    typedef struct xcom_message {
        XCOMParserStates state = XComSync;
        uint16_t rx_cpy_idx    = 0;
        uint16_t rx_idx        = 0;
        uint8_t msg_id         = 0;
        uint8_t bytes_to_read  = 0;
        uint16_t msg_length    = 0;
        uint16_t hdr_gps_week  = 0;
        double hdr_gps_time    = 0.0;
        uint8_t payload[MaxMessageSize];
        uint16_t crc = 0;
    } XCOMMessage;
    XCOMMessage _rx_state;
    Crc16 _crc16;
    void init_parser() noexcept;
};
}  // namespace xcom
#endif /* LIBXCOM_PROTOCOL_XCOM_XCOMPARSER_H_ */
