/*.*******************************************************************
 FILENAME: xcom.h
 **********************************************************************
 *  PROJECT: ROS2_ROS2_iNAT
 *
 *
 *---------------------------------------------------------------------
 * 	Copyright 2021, iMAR Navigation
 *---------------------------------------------------------------------
 * 	MODULE DESCRIPTION:
 *
 ---------------------------------------------------------------------*/
#ifndef LIB_IXCOM_X_COM_H
#define LIB_IXCOM_X_COM_H
#include "../../src/parameter_traits.h"
#include <array>
#include <ixcom/crc16.h>
#include <ixcom/xcom_parser.h>
#include <optional>
#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif
namespace xcom {
class IReader {
public:
    virtual ~IReader()                                                        = default;
    virtual int32_t read(uint8_t* buffer, std::size_t buffer_length) noexcept = 0;
    virtual bool initialize() noexcept { return true; }
};
class IWriter {
public:
    virtual ~IWriter()                                                               = default;
    virtual int32_t write(const uint8_t* buffer, std::size_t buffer_length) noexcept = 0;
    virtual bool initialize() noexcept { return true; }
};
enum class CallBackType {
    Message,
    Parameter,
    Error,
    Command,
    Response
};
typedef struct xcom_msg_callbacks_node {
    uint16_t msg_id;
    uint16_t param_id;
    CallBackType cb_type;
    void* cb;
    void* context;
    struct xcom_msg_callbacks_node* next;
} xcom_callbacks_node_t;
class XComState {
    using ErrorCode = XComParser::ParserCode;
    typedef void (*xcom_msg_callback_t)(uint8_t msg[], void* context);
    typedef void (*xcom_param_callback_t)(uint8_t msg[], void* context);
    typedef void (*xcom_command_callback_t)(uint16_t command_id, std::size_t frame_len, uint8_t frame[], void* context);
    typedef void (*xcom_response_callback_t)(uint16_t command_id, std::size_t frame_len, uint8_t frame[], void* context);
    typedef void (*xcom_error_callback_t)(const ErrorCode& error_code, void* context);
public:
    XComState() noexcept          = default;
    virtual ~XComState() noexcept = default;
    bool initialize() noexcept;
    enum class ReturnCode {
        Ok,
        Timeout,
        InvalidResponse,
        InvalidReader
    };
    ReturnCode process() noexcept;
    int register_message_callback(uint8_t msg_id, xcom_msg_callback_t cb, void* context, xcom_callbacks_node_t* node) noexcept;
    int register_parameter_callback(uint16_t param_id, xcom_param_callback_t cb, void* context, xcom_callbacks_node_t* node) noexcept;
    int register_command_callback(xcom_command_callback_t cb, void* context, xcom_callbacks_node_t* node) noexcept;
    int register_response_callback(xcom_response_callback_t cb, void* context, xcom_callbacks_node_t* node) noexcept;
    int register_error_callback(xcom_error_callback_t cb, void* context, xcom_callbacks_node_t* node) noexcept;
    void set_reader(IReader* reader) noexcept { _reader = reader; }
    void set_writer(IWriter* writer) noexcept { _writer = writer; }
    void enable_forwarding() noexcept;
    void disable_forwarding() noexcept;
    void skip_frame() noexcept;
    void forced_exit() noexcept;
    void set_rc(ReturnCode rc) noexcept;
    uint8_t* get_payload() noexcept;
    [[nodiscard]] std::size_t get_payload_length() const noexcept;
    void set_sync_byte(uint8_t sync_byte) noexcept;
    [[nodiscard]] uint8_t get_sync_byte() const noexcept;
    struct system_status {
        uint32_t mode          = 0;
        uint32_t system_status = 0;
        std::optional<uint32_t> status_imu{};
        std::optional<uint32_t> status_gnss{};
        std::optional<uint32_t> status_mag{};
        std::optional<uint32_t> status_madc{};
        std::optional<std::array<uint32_t, 2>> status_ekfaiding{};
        std::optional<uint32_t> status_ekfgeneral{};
        std::optional<std::array<uint32_t, 4>> status_addimu{};
        std::optional<float> status_remaining_aligntime{};
        uint16_t global_status = 0;
    };
    static std::optional<system_status> process_msg_sysstat(const uint8_t* data, std::size_t len);
    // XCOM commands
    XCOMCmd_XCOM get_xcomcmd_open(uint16_t channel);
    XCOMCmd_XCOM get_xcomcmd_close(uint16_t channel);
    XCOMCmd_LOG get_xcomcmd_clearall();
    XCOMCmd_LOG get_xcomcmd_enablelog(XComMessageID id, XComLogTrigger trigger, uint16_t divider);
    XCOMCmd_CONF get_cmd_save_config() noexcept;
    XCOMCmd_XCOM get_cmd_reboot() noexcept;
    XCOMCmd_EXTAID_POSLLH get_xcomcmd_extaid_posllh(const double& timestamp, uint16_t timemode,
                                                    const std::array<double, 3>& pos, const std::array<double, 3>& pos_stddev,
                                                    const std::array<double, 3>& leverarm, const std::array<double, 3>& leverarm_stddev,
                                                    uint32_t enable_msl_alt);
    XCOMCmd_EXTAID_POSECEF get_xcomcmd_extaid_posecef(const double& timestamp, uint16_t timemode,
                                                      const std::array<double, 3>& pos, const std::array<double, 3>& pos_stddev,
                                                      const std::array<double, 3>& leverarm, const std::array<double, 3>& leverarm_stddev);
    XCOMCmd_EXTAID_POSUTM get_xcomcmd_extaid_posutm(const double& timestamp, uint16_t timemode,
                                                    int32_t zone, uint8_t north_hp,
                                                    const double& easting, const double& northing, const double& altitude,
                                                    const std::array<double, 3>& pos_stddev,
                                                    const std::array<double, 3>& leverarm, const std::array<double, 3>& leverarm_stddev);
    XCOMCmd_EXTAID_POSMGRS get_xcomcmd_extaid_posmgrs(const double& timestamp, uint16_t timemode,
                                                      int8_t mgrs[XCOM_EXTAID_POSMGRS_MAX_LENGTH],
                                                      const double& alt, const std::array<double, 3>& pos_stddev,
                                                      const std::array<double, 3>& leverarm, const std::array<double, 3>& leverarm_stddev);
    XCOMCmd_EXTAID_HDG get_xcomcmd_extaid_hdg(const double& timestamp, uint16_t timemode,
                                              const double& heading, const double& heading_stddev);
    XCOMCmd_EXTAID_VEL get_xcomcmd_extaid_vel(const double& timestamp, uint16_t timemode,
                                              const std::array<double, 3>& vel, const std::array<double, 3>& vel_stddev);
    XCOMCmd_EXTAID_VELBODY get_xcomcmd_extaid_velbody(const double& timestamp, uint16_t timemode,
                                                      const std::array<double, 3>& vel, const std::array<double, 3>& vel_stddev,
                                                      const std::array<double, 3>& leverarm, const std::array<double, 3>& leverarm_stddev);
    XCOMCmd_EXTAID_HEIGHT get_xcomcmd_extaid_height(const double& timestamp, uint16_t timemode,
                                                    const double& height, const double& height_stddev);
    XCOMCmdEKF_ALIGNCOMPLETE align_complete() noexcept;
    // XCOM parameter
    template<typename ParamType>
    ParamType get_generic_param() {
        ParamType frame{};
        build_header(frame, 0.0, 0, XComMessageID::XCOM_MSGID_PARAMETER, XComLogTrigger::XCOM_CMDLOG_TRIG_POLLED);
        build_parameter_header(frame.param_header, static_cast<const XComParameterID>(ParameterTraits<ParamType>::Id),
                               XCOMpar_action::XCOM_PAR_GET);
        complete_message(frame);
        return frame;
    }
    XCOMParXCOM_POSTPROC enable_postproc(uint8_t channel = 31) {
        auto frame                    = get_generic_param<XCOMParXCOM_POSTPROC>();
        frame.param_header.is_request = XCOMpar_action::XCOM_PAR_SET;
        frame.enable                  = 1;
        frame.channel                 = channel;
        complete_message(frame);
        return frame;
    }
    XCOMParXCOM_POSTPROC disable_postproc(uint8_t channel = 31) {
        auto frame                    = get_generic_param<XCOMParXCOM_POSTPROC>();
        frame.param_header.is_request = XCOMpar_action::XCOM_PAR_SET;
        frame.enable                  = 0;
        frame.channel                 = channel;
        complete_message(frame);
        return frame;
    }
    XCOMParEKF_STARTUPV2 initialize_ekf(const std::array<double, 3> pos, const std::array<float, 3> pos_stddev, const float& hdg,
                                        const float& hdg_stddev, const std::array<float, 3> la, const std::array<float, 3> la_stddev,
                                        uint8_t position_mode, uint8_t heading_mode, bool start_alignment) {
        auto frame                    = get_generic_param<XCOMParEKF_STARTUPV2>();
        frame.param_header.is_request = XCOMpar_action::XCOM_PAR_SET;
        frame.initpos_lat             = pos[0];
        frame.initpos_lon             = pos[1];
        frame.initpos_alt             = static_cast<float>(pos[2]);
        for(int idx = 0; idx < 3; idx++) {
            frame.initpos_stddev[idx]   = pos_stddev[idx];
            frame.lever_arm[idx]        = la[idx];
            frame.lever_arm_stddev[idx] = la_stddev[idx];
        }
        frame.inithdg           = hdg;
        frame.inithdg_stdddev   = hdg_stddev;
        frame.forced_inmotion   = 0;
        frame.automatic_restart = PAREKF_STARTUPV2_RESTART_DISABLE;
        frame.enable_alt_msl    = PAREKF_STARTUPV2_ALTMSL_DISABLE;
        frame.realignment       = start_alignment;
        frame.position_mode     = position_mode;
        frame.hdg_mode          = heading_mode;
        complete_message(frame);
        return frame;
    }
    template<typename MsgType>
    int32_t send_message(MsgType&& msg) noexcept {
        auto p             = reinterpret_cast<uint8_t*>(&msg);
        const auto msg_len = sizeof(MsgType);
        msg.footer.crc16   = _crc16.process(p, msg_len - sizeof(uint16_t));
        if(_writer != nullptr) {
            return (_writer->write(p, msg_len));
        }
        return -1;
    }
    template<typename MsgType>
    void complete_message(MsgType& msg) noexcept {
        auto p             = reinterpret_cast<uint8_t*>(&msg);
        const auto msg_len = sizeof(MsgType);
        msg.footer.crc16   = _crc16.process(p, msg_len - sizeof(uint16_t));
    }
    template<typename MsgT>
    static double get_timestamp(const MsgT& msg) noexcept {
        return static_cast<double>(msg.header.gps_time_sec) + (static_cast<double>(msg.header.gps_time_usec) * 1e-6);
    }
    template<typename MsgT>
    static void build_header(MsgT& msg, double gps_time, uint16_t gps_week, XComMessageID id, XComLogTrigger trig_src) {
        msg.header.frame_counter++;
        msg.header.msg_id         = id;
        msg.header.trigger_source = trig_src;
        msg.header.sync           = XCOM_SYNC_BYTE;
        msg.header.gps_time_sec   = static_cast<uint32_t>(gps_time);
        const double tmp          = (gps_time - static_cast<double>(msg.header.gps_time_sec)) * 1e6;
        msg.header.gps_time_usec  = static_cast<uint32_t>(tmp);
        msg.header.gps_week       = gps_week;
        msg.header.msg_len        = static_cast<uint16_t>(sizeof(MsgT));
    }
    static void build_parameter_header(XCOMParHeader& subheader, const XComParameterID paramID, XCOMpar_action is_request) {
        subheader.param_id   = paramID;
        subheader.is_request = is_request;
        subheader.reserved   = 0;
    }
private:
    static constexpr int BufferSize = 1024;
    std::array<uint8_t, BufferSize> _buf{};
    XComParser _parser{};
    typedef struct {
        uint16_t msg_type;
        uint16_t frame_len;
        uint8_t frame_buff[4096];
        uint8_t* msg_buff;
        void* io_context;
        xcom_callbacks_node_t* xcom_msg_callbacks_head;
    } xcom_state_t;
    xcom_state_t _state{};
    IReader* _reader        = nullptr;
    IWriter* _writer        = nullptr;
    bool _enable_forwarding = false;
    bool _skip_frame        = false;
    bool _forced_exit       = false;
    ReturnCode _rc          = ReturnCode::Ok;
    Crc16 _crc16;
    static void state_init(xcom_state_t& state) noexcept;
    static void process_message(xcom_state_t& s, uint8_t msg_id, uint8_t payload[]) noexcept;
    static void process_parameter(xcom_state_t& s, uint16_t param_id, uint8_t payload[]) noexcept;
    static void process_command(xcom_state_t& s, uint16_t cmd_id, std::size_t payload_length, uint8_t payload[]) noexcept;
    static void process_response(xcom_state_t& s, uint16_t cmd_id, std::size_t payload_length, uint8_t payload[]) noexcept;
    static void process_error(xcom_state_t& s, XComParser::ParserCode ec) noexcept;
    template<typename T>
    int register_callback(T id, xcom_msg_callback_t cb, void* context, xcom_callbacks_node_t* node, CallBackType cb_type) noexcept;
};
}  // namespace xcom
#endif  // LIB_IXCOM_X_COM_H
