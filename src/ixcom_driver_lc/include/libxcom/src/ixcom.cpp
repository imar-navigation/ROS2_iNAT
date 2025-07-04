/*.*******************************************************************
 FILENAME: xcom.cc
 **********************************************************************
 *  PROJECT: iXCOM_SDK
 *
 *
 *---------------------------------------------------------------------
 * 	Copyright 2021, iMAR Navigation
 *---------------------------------------------------------------------
 * 	MODULE DESCRIPTION:
 *
 ---------------------------------------------------------------------*/
#include <cstring>
#include <ixcom/ixcom.h>
namespace xcom {
bool XComState::initialize() noexcept {
    state_init(_state);
    return true;
}
XComState::ReturnCode XComState::process() noexcept {
    if(_reader == nullptr) {
        return XComState::ReturnCode::InvalidReader;
    }
    int32_t len;
    while(!_forced_exit && (len = _reader->read(_buf.data(), _buf.size())) > 0) {
        for(int32_t idx = 0; idx < len; idx++) {
            auto rc = _parser.process_byte(_buf[idx]);
            if(rc == XComParser::ParserCode::Ok) {
                if(_parser.is_msg()) {
                    process_message(_state, _parser.get_msg_id(), _parser.get_payload(), _parser.get_payload_length());
                } else if(_parser.is_param()) {
                    process_parameter(_state, _parser.get_param_id(), _parser.get_payload());
                } else if(_parser.is_cmd()) {
                    process_command(_state, _parser.get_cmd_id(), _parser.get_payload_length(), _parser.get_payload());
                } else {
                    // must be a response message
                    process_response(_state, _parser.get_cmd_id(), _parser.get_payload_length(), _parser.get_payload());
                }
                // forward message
                if((_writer != nullptr) && _enable_forwarding && !_skip_frame) {
                    _writer->write(_parser.get_payload(), _parser.get_payload_length());
                }
                _skip_frame = false;
            } else if(rc != XComParser::ParserCode::Running) {
                process_error(_state, rc);
            }
        }
    }
    if(_forced_exit) {
        _forced_exit = false;
        return _rc;
    }
    return XComState::ReturnCode::Ok;
}
void XComState::state_init(xcom_state_t& state) noexcept { state.xcom_msg_callbacks_head = nullptr; }
int XComState::register_message_callback(uint8_t msg_id, xcom_msg_callback_t cb, void* context, xcom_callbacks_node_t* node) noexcept {
    return register_callback<uint8_t>(msg_id, cb, context, node, CallBackType::Message);
}
int XComState::register_generic_message_callback(XComMessageID msg_id, xcom_generic_msg_callback_t cb, void* context,
                                                 xcom_callbacks_node_t* node) noexcept {
    return register_callback<XComMessageID>(msg_id, cb, context, node, CallBackType::MessageGeneric);
}
void XComState::process_message(xcom_state_t& s, const uint8_t msg_id, uint8_t* payload, std::size_t length) noexcept {
    for(const auto* node = s.xcom_msg_callbacks_head; node; node = node->next) {
        if((node->msg_id == msg_id) && (node->param_id == XCOMPAR_INVALID)) {
            if(node->cb_type == CallBackType::Message) {
                reinterpret_cast<xcom_msg_callback_t>(node->cb)(payload, node->context);
            }
            if(node->cb_type == CallBackType::MessageGeneric) {
                reinterpret_cast<xcom_generic_msg_callback_t>(node->cb)(static_cast<XComMessageID>(msg_id), payload, length, node->context);
            }
        }
    }
}
void XComState::process_parameter(XComState::xcom_state_t& s, uint16_t param_id, uint8_t* payload) noexcept {
    for(const auto* node = s.xcom_msg_callbacks_head; node; node = node->next) {
        if((node->param_id == param_id) && (node->msg_id == XCOM_MSGID_PARAMETER)) {
            reinterpret_cast<xcom_param_callback_t>(node->cb)(payload, node->context);
        }
    }
}
int XComState::register_parameter_callback(uint16_t param_id, XComState::xcom_param_callback_t cb, void* context,
                                           xcom_callbacks_node_t* node) noexcept {
    return register_callback<uint16_t>(param_id, cb, context, node, CallBackType::Parameter);
}
void XComState::enable_forwarding() noexcept { _enable_forwarding = true; }
void XComState::disable_forwarding() noexcept { _enable_forwarding = false; }
void XComState::skip_frame() noexcept { _skip_frame = true; }
template<typename T, typename CallbackType>
int XComState::register_callback(T id, CallbackType cb, void* context, xcom_callbacks_node_t* node, CallBackType cb_type) noexcept {
    if(cb == nullptr) {
        return -1;
    }
    if(node == nullptr) {
        return -1;
    }
    if(cb_type == CallBackType::Message || cb_type == CallBackType::MessageGeneric) {
        for(xcom_callbacks_node_t* n = _state.xcom_msg_callbacks_head; n; n = n->next) {
            if((n == node) || ((n->cb == cb) && (n->msg_id == id) && (n->context == context))) {
                return -1;
            }
        }
        node->msg_id   = id;
        node->param_id = XCOMPAR_INVALID;
    } else if(cb_type == CallBackType::Parameter) {
        for(xcom_callbacks_node_t* n = _state.xcom_msg_callbacks_head; n; n = n->next) {
            if((n == node) || ((n->cb == cb) && (n->param_id == id) && (n->context == context))) {
                return -1;
            }
        }
        node->msg_id   = XCOM_MSGID_PARAMETER;
        node->param_id = id;
    } else if(cb_type == CallBackType::Error) {
        node->msg_id   = XCOM_MSGID_PARAMETER;
        node->param_id = id;
    }
    node->cb      = reinterpret_cast<void*>(cb);
    node->context = context;
    node->cb_type = cb_type;
    node->next    = nullptr;
    /* If our linked list is empty then just add the new node to the start. */
    if(_state.xcom_msg_callbacks_head == nullptr) {
        _state.xcom_msg_callbacks_head = node;
        return 0;
    }
    /* Find the tail of our linked list and add our new node to the end. */
    xcom_callbacks_node_t* p = _state.xcom_msg_callbacks_head;
    while(p->next) {
        p = p->next;
    }
    p->next = node;
    return 0;
}
int XComState::register_error_callback(XComState::xcom_error_callback_t cb, void* context, xcom_callbacks_node_t* node) noexcept {
    return register_callback<uint16_t>(XCOMPAR_INVALID, reinterpret_cast<xcom_msg_callback_t>(cb), context, node, CallBackType::Error);
}
void XComState::process_error(XComState::xcom_state_t& s, XComParser::ParserCode ec) noexcept {  // NOLINT
    for(const auto* node = s.xcom_msg_callbacks_head; node; node = node->next) {
        if(node->cb_type == CallBackType::Error) {
            reinterpret_cast<xcom_error_callback_t>(node->cb)(ec, node->context);
        }
    }
}
void XComState::process_command(XComState::xcom_state_t& s, uint16_t cmd_id, std::size_t payload_length, uint8_t* payload) noexcept {
    for(const auto* node = s.xcom_msg_callbacks_head; node; node = node->next) {
        if(node->cb_type == CallBackType::Command) {
            reinterpret_cast<xcom_command_callback_t>(node->cb)(cmd_id, payload_length, payload, node->context);
        }
    }
}
void XComState::process_response(XComState::xcom_state_t& s, uint16_t cmd_id, std::size_t payload_length, uint8_t* payload) noexcept {
    for(const auto* node = s.xcom_msg_callbacks_head; node; node = node->next) {
        if(node->cb_type == CallBackType::Response) {
            reinterpret_cast<xcom_response_callback_t>(node->cb)(cmd_id, payload_length, payload, node->context);
        }
    }
}
int XComState::register_command_callback(XComState::xcom_command_callback_t cb, void* context, xcom_callbacks_node_t* node) noexcept {
    if(cb == nullptr) {
        return -1;
    }
    if(node == nullptr) {
        return -1;
    }
    node->msg_id   = XCOM_MSGID_COMMAND;
    node->param_id = XCOMPAR_INVALID;
    node->cb       = reinterpret_cast<void*>(cb);
    node->context  = context;
    node->cb_type  = CallBackType::Command;
    node->next     = nullptr;
    /* If our linked list is empty then just add the new node to the start. */
    if(_state.xcom_msg_callbacks_head == nullptr) {
        _state.xcom_msg_callbacks_head = node;
        return 0;
    }
    /* Find the tail of our linked list and add our new node to the end. */
    xcom_callbacks_node_t* p = _state.xcom_msg_callbacks_head;
    while(p->next) {
        p = p->next;
    }
    p->next = node;
    return 0;
}
int XComState::register_response_callback(XComState::xcom_command_callback_t cb, void* context, xcom_callbacks_node_t* node) noexcept {
    if(cb == nullptr) {
        return -1;
    }
    if(node == nullptr) {
        return -1;
    }
    node->msg_id   = XCOM_MSGID_RESPONSE;
    node->param_id = XCOMPAR_INVALID;
    node->cb       = reinterpret_cast<void*>(cb);
    node->context  = context;
    node->cb_type  = CallBackType::Response;
    node->next     = nullptr;
    /* If our linked list is empty then just add the new node to the start. */
    if(_state.xcom_msg_callbacks_head == nullptr) {
        _state.xcom_msg_callbacks_head = node;
        return 0;
    }
    /* Find the tail of our linked list and add our new node to the end. */
    xcom_callbacks_node_t* p = _state.xcom_msg_callbacks_head;
    while(p->next) {
        p = p->next;
    }
    p->next = node;
    return 0;
}
XCOMCmd_XCOM XComState::get_xcomcmd_open(uint16_t channel) {
    XCOMCmd_XCOM frame{};
    build_header(frame, 0.0, 0, XComMessageID::XCOM_MSGID_COMMAND, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC);
    frame.cmd_header.cmd_id   = XCOM_CMDID_XCOM;
    frame.cmd_header.specific = 0;
    frame.xcom_cmd            = XCOM_CMDXCOM_OPEN;
    frame.channel             = channel;
    complete_message(frame);
    return frame;
}
XCOMCmd_XCOM XComState::get_xcomcmd_close(uint16_t channel) {
    XCOMCmd_XCOM frame{};
    build_header(frame, 0.0, 0, XComMessageID::XCOM_MSGID_COMMAND, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC);
    frame.cmd_header.cmd_id   = XCOM_CMDID_XCOM;
    frame.cmd_header.specific = 0;
    frame.xcom_cmd            = XCOM_CMDXCOM_CLOSE;
    frame.channel             = channel;
    complete_message(frame);
    return frame;
}
void XComState::forced_exit() noexcept { _forced_exit = true; }
XCOMCmd_LOG XComState::get_xcomcmd_clearall() {
    XCOMCmd_LOG frame{};
    build_header(frame, 0.0, 0, XComMessageID::XCOM_MSGID_COMMAND, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC);
    frame.cmd_header.cmd_id   = XCOM_CMDID_LOG;
    frame.cmd_header.specific = 0;
    frame.log_cmd             = XCOM_CMDLOG_CLEARALL;
    frame.msg_id              = XCOM_MSGID_IMURAW;
    frame.divider             = 1;
    frame.trigger             = XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC;
    complete_message(frame);
    return frame;
}
XCOMCmd_LOG XComState::get_xcomcmd_enablelog(XComMessageID id, XComLogTrigger trigger, uint16_t divider) {
    XCOMCmd_LOG frame{};
    build_header(frame, 0.0, 0, XComMessageID::XCOM_MSGID_COMMAND, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC);
    frame.cmd_header.cmd_id   = XCOM_CMDID_LOG;
    frame.cmd_header.specific = 0;
    frame.log_cmd             = XCOM_CMDLOG_ADD;
    frame.msg_id              = id;
    frame.divider             = divider;
    frame.trigger             = trigger;
    complete_message(frame);
    return frame;
}
void XComState::set_rc(XComState::ReturnCode rc) noexcept { _rc = rc; }
XCOMCmd_CONF XComState::get_cmd_save_config() noexcept {
    XCOMCmd_CONF frame{};
    build_header(frame, 0.0, 0, XComMessageID::XCOM_MSGID_COMMAND, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC);
    frame.cmd_header.cmd_id   = XCOM_CMDID_CONF;
    frame.cmd_header.specific = 0;
    frame.conf_cmd            = XCOM_CMDCONF_SAVE;
    complete_message(frame);
    return frame;
}
XCOMCmd_XCOM XComState::get_cmd_reboot() noexcept {
    XCOMCmd_XCOM frame{};
    build_header(frame, 0.0, 0, XComMessageID::XCOM_MSGID_COMMAND, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC);
    frame.cmd_header.cmd_id   = XCOM_CMDID_XCOM;
    frame.cmd_header.specific = 0;
    frame.xcom_cmd            = XCOM_CMDXCOM_COLDRESET;
    complete_message(frame);
    return frame;
}
XCOMCmdEKF_STOREPOS XComState::get_cmd_storepos() noexcept {
    XCOMCmdEKF_STOREPOS frame{};
    build_header(frame, 0.0, 0, XComMessageID::XCOM_MSGID_COMMAND, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC);
    frame.cmd_header.cmd_id        = XCOM_CMDID_EKF;
    frame.cmd_header.specific      = 0;
    frame.ekf_arguments.ekf_cmd_id = XCOM_CMDEKF_STOREPOS;
    complete_message(frame);
    return frame;
}
uint8_t* XComState::get_payload() noexcept { return _parser.get_payload(); }
std::size_t XComState::get_payload_length() const noexcept { return _parser.get_payload_length(); }
std::optional<XComState::system_status> XComState::process_msg_sysstat(const uint8_t* data, std::size_t len) {
    system_status sysstat{};
    XCOMHeader hdr{};
    std::memcpy(reinterpret_cast<void*>(&hdr), data, sizeof(XCOMHeader));
    if(hdr.msg_id != XCOM_MSGID_SYSSTAT) {
        return {};
    }
    auto idx     = sizeof(XCOMHeader);
    sysstat.mode = *reinterpret_cast<const uint32_t*>(&data[idx]);
    idx += sizeof(sysstat.mode);
    sysstat.status_system = *reinterpret_cast<const uint32_t*>(&data[idx]);
    idx += sizeof(sysstat.status_system);
    if(sysstat.mode & PARDAT_SYSSTAT_MASK_IMU) {
        sysstat.status_imu.emplace(*reinterpret_cast<const uint32_t*>(&data[idx]));
        idx += sizeof(uint32_t);
    }
    if(sysstat.mode & PARDAT_SYSSTAT_MASK_GNSS) {
        sysstat.status_gnss.emplace(*reinterpret_cast<const uint32_t*>(&data[idx]));
        idx += sizeof(uint32_t);
    }
    if(sysstat.mode & PARDAT_SYSSTAT_MASK_MAG) {
        sysstat.status_mag.emplace(*reinterpret_cast<const uint32_t*>(&data[idx]));
        idx += sizeof(uint32_t);
    }
    if(sysstat.mode & PARDAT_SYSSTAT_MASK_MADC) {
        sysstat.status_madc.emplace(*reinterpret_cast<const uint32_t*>(&data[idx]));
        idx += sizeof(uint32_t);
    }
    if(sysstat.mode & PARDAT_SYSSTAT_MASK_EKFAIDING) {
        std::array<uint32_t, 2> ekf_aiding{};
        ekf_aiding[0] = *reinterpret_cast<const uint32_t*>(&data[idx]);
        idx += sizeof(uint32_t);
        ekf_aiding[1] = *reinterpret_cast<const uint32_t*>(&data[idx]);
        idx += sizeof(uint32_t);
        sysstat.status_ekfaiding.emplace(ekf_aiding);
    }
    if(sysstat.mode & PARDAT_SYSSTAT_MASK_EKFGENERAL) {
        sysstat.status_ekfgeneral.emplace(*reinterpret_cast<const uint32_t*>(&data[idx]));
        idx += sizeof(uint32_t);
    }
    if(sysstat.mode & PARDAT_SYSSTAT_MASK_ADDSTATUS) {
        std::array<uint32_t, 4> add_status{};
        add_status[0] = *reinterpret_cast<const uint32_t*>(&data[idx]);
        idx += sizeof(uint32_t);
        add_status[1] = *reinterpret_cast<const uint32_t*>(&data[idx]);
        idx += sizeof(uint32_t);
        add_status[2] = *reinterpret_cast<const uint32_t*>(&data[idx]);
        idx += sizeof(uint32_t);
        add_status[3] = *reinterpret_cast<const uint32_t*>(&data[idx]);
        idx += sizeof(uint32_t);
        sysstat.status_addimu.emplace(add_status);
    }
    if(sysstat.mode & PARDAT_SYSSTAT_MASK_REMALIGNTIME) {
        sysstat.status_remaining_aligntime.emplace(*reinterpret_cast<const float*>(&data[idx]));
        idx += sizeof(float);
    }
    if(sysstat.mode & PARDAT_SYSSTAT_MASK_SYSSTAT2) {
        sysstat.status_system2.emplace(*reinterpret_cast<const uint32_t*>(&data[idx]));
        idx += sizeof(uint32_t);
    }
    sysstat.global_status = *reinterpret_cast<const uint32_t*>(&data[len - sizeof(XCOMFooter)]);
    return sysstat;
}
std::tuple<std::vector<XCOMmsg_CANGATEWAY_MsgType>, XCOMGlobalStatus> XComState::process_cangateway(const XCOMmsg_CANGATEWAY& msg) {
    auto buffer_idx               = 0;
    const auto payload_length     = (msg.header.msg_len - sizeof(XCOMHeader) - sizeof(XCOMCanGatewayHeader) - sizeof(XCOMFooter));
    constexpr auto CanFrameLength = sizeof(XCOMmsg_CANGATEWAY_MsgType);
    const auto number_of_elements = payload_length / CanFrameLength;
    std::vector<XCOMmsg_CANGATEWAY_MsgType> can_frames{};
    can_frames.reserve(number_of_elements);
    for(std::size_t idx = 0; idx < number_of_elements; idx++) {
        XCOMmsg_CANGATEWAY_MsgType can_frame{};
        memcpy(&can_frame, &msg.buffer[buffer_idx], sizeof(XCOMmsg_CANGATEWAY_MsgType));
        can_frames.emplace_back(can_frame);
        buffer_idx += sizeof(XCOMmsg_CANGATEWAY_MsgType);
    }
    union {
        XCOMGlobalStatus bits;
        uint16_t value;
    } global_status{};
    memcpy(&global_status.value, &msg.buffer[buffer_idx], sizeof(uint16_t));
    return {can_frames, global_status.bits};
}
void XComState::set_sync_byte(uint8_t sync_byte) noexcept { _parser.set_sync_byte(sync_byte); }
uint8_t XComState::get_sync_byte() const noexcept { return _parser.get_sync_byte(); }
XCOMCmdEKF_ALIGNCOMPLETE XComState::align_complete() noexcept {
    XCOMCmdEKF_ALIGNCOMPLETE frame{};
    build_header(frame, 0.0, 0, XComMessageID::XCOM_MSGID_COMMAND, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC);
    frame.cmd_header.cmd_id                 = XCOM_CMDID_EKF;
    frame.cmd_header.specific               = 0;
    frame.ekf_arguments.ekf_cmd_id          = XCOM_CMDEKF_ALIGNCOMPLETE;
    frame.ekf_arguments.number_of_arguments = 0U;
    complete_message(frame);
    return frame;
}
XCOMCmdEKF_STARTALIGN XComState::align_start() noexcept {
    XCOMCmdEKF_STARTALIGN frame{};
    build_header(frame, 0.0, 0, XComMessageID::XCOM_MSGID_COMMAND, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC);
    frame.cmd_header.cmd_id                 = XCOM_CMDID_EKF;
    frame.cmd_header.specific               = 0;
    frame.ekf_arguments.ekf_cmd_id          = XCOM_CMDEKF_STARTALIGN;
    frame.ekf_arguments.number_of_arguments = 0U;
    complete_message(frame);
    return frame;
}
XCOMCmd_EXTAID_POSLLH XComState::get_xcomcmd_extaid_posllh(const double& timestamp, uint16_t timemode, const std::array<double, 3>& pos,
                                                           const std::array<double, 3>& pos_stddev, const std::array<double, 3>& leverarm,
                                                           const std::array<double, 3>& leverarm_stddev, uint32_t enable_msl_alt) {
    XCOMCmd_EXTAID_POSLLH frame{};
    build_header(frame, 0.0, 0, XComMessageID::XCOM_MSGID_COMMAND, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC);
    frame.cmd_header.cmd_id    = XCOM_CMDID_EXTAID;
    frame.cmd_header.specific  = 0;
    frame.command_parameter_id = XCOM_CMDEXTAID_POS_LLH;
    frame.time_stamp           = timestamp;
    frame.time_mode            = timemode;
    for(int i = 0; i < 3; i++) {
        frame.position[i]         = pos[i];
        frame.position_stddev[i]  = pos_stddev[i];
        frame.lever_arm[i]        = leverarm[i];
        frame.lever_arm_stddev[i] = leverarm_stddev[i];
    }
    frame.enable_msl_altitude = enable_msl_alt;
    complete_message(frame);
    return frame;
}
XCOMCmd_EXTAID_POSECEF XComState::get_xcomcmd_extaid_posecef(const double& timestamp, uint16_t timemode, const std::array<double, 3>& pos,
                                                             const std::array<double, 3>& pos_stddev, const std::array<double, 3>& leverarm,
                                                             const std::array<double, 3>& leverarm_stddev) {
    XCOMCmd_EXTAID_POSECEF frame{};
    build_header(frame, 0.0, 0, XComMessageID::XCOM_MSGID_COMMAND, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC);
    frame.cmd_header.cmd_id    = XCOM_CMDID_EXTAID;
    frame.cmd_header.specific  = 0;
    frame.command_parameter_id = XCOM_CMDEXTAID_POS_ECEF;
    frame.time_stamp           = timestamp;
    frame.time_mode            = timemode;
    for(int i = 0; i < 3; i++) {
        frame.position[i]         = pos[i];
        frame.position_stddev[i]  = pos_stddev[i];
        frame.lever_arm[i]        = leverarm[i];
        frame.lever_arm_stddev[i] = leverarm_stddev[i];
    }
    complete_message(frame);
    return frame;
}
XCOMCmd_EXTAID_POSUTM XComState::get_xcomcmd_extaid_posutm(const double& timestamp, uint16_t timemode, int32_t zone, uint8_t north_hp,
                                                           const double& easting, const double& northing, const double& altitude,
                                                           const std::array<double, 3>& pos_stddev, const std::array<double, 3>& leverarm,
                                                           const std::array<double, 3>& leverarm_stddev) {
    XCOMCmd_EXTAID_POSUTM frame{};
    build_header(frame, 0.0, 0, XComMessageID::XCOM_MSGID_COMMAND, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC);
    frame.cmd_header.cmd_id    = XCOM_CMDID_EXTAID;
    frame.cmd_header.specific  = 0;
    frame.command_parameter_id = XCOM_CMDEXTAID_POS_UTM;
    frame.time_stamp           = timestamp;
    frame.time_mode            = timemode;
    frame.zone                 = zone;
    frame.north_hp             = north_hp;
    frame.easting              = easting;
    frame.northing             = northing;
    frame.altitude             = altitude;
    for(int i = 0; i < 3; i++) {
        frame.position_stddev[i]  = pos_stddev[i];
        frame.lever_arm[i]        = leverarm[i];
        frame.lever_arm_stddev[i] = leverarm_stddev[i];
    }
    complete_message(frame);
    return frame;
}
XCOMCmd_EXTAID_POSMGRS XComState::get_xcomcmd_extaid_posmgrs(const double& timestamp, uint16_t timemode,
                                                             int8_t mgrs[XCOM_EXTAID_POSMGRS_MAX_LENGTH], const double& alt,
                                                             const std::array<double, 3>& pos_stddev, const std::array<double, 3>& leverarm,
                                                             const std::array<double, 3>& leverarm_stddev) {
    XCOMCmd_EXTAID_POSMGRS frame{};
    build_header(frame, 0.0, 0, XComMessageID::XCOM_MSGID_COMMAND, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC);
    frame.cmd_header.cmd_id    = XCOM_CMDID_EXTAID;
    frame.cmd_header.specific  = 0;
    frame.command_parameter_id = XCOM_CMDEXTAID_POS_MGRS;
    frame.time_stamp           = timestamp;
    frame.time_mode            = timemode;
    frame.altitude             = alt;
    std::copy(mgrs, mgrs + XCOM_EXTAID_POSMGRS_MAX_LENGTH, frame.mgrs);
    for(int i = 0; i < 3; i++) {
        frame.position_stddev[i]  = pos_stddev[i];
        frame.lever_arm[i]        = leverarm[i];
        frame.lever_arm_stddev[i] = leverarm_stddev[i];
    }
    complete_message(frame);
    return frame;
}
XCOMCmd_EXTAID_HDG XComState::get_xcomcmd_extaid_hdg(const double& timestamp, uint16_t timemode, const double& heading,
                                                     const double& heading_stddev) {
    XCOMCmd_EXTAID_HDG frame{};
    build_header(frame, 0.0, 0, XComMessageID::XCOM_MSGID_COMMAND, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC);
    frame.cmd_header.cmd_id    = XCOM_CMDID_EXTAID;
    frame.cmd_header.specific  = 0;
    frame.command_parameter_id = XCOM_CMDEXTAID_HDG;
    frame.time_stamp           = timestamp;
    frame.time_mode            = timemode;
    frame.heading              = heading;
    frame.heading_stddev       = heading_stddev;
    complete_message(frame);
    return frame;
}
XCOMCmd_EXTAID_VEL XComState::get_xcomcmd_extaid_vel_ned(const double& timestamp, uint16_t timemode, const std::array<double, 3>& vel,
                                                         const std::array<double, 3>& vel_stddev) {
    XCOMCmd_EXTAID_VEL frame{};
    build_header(frame, 0.0, 0, XComMessageID::XCOM_MSGID_COMMAND, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC);
    frame.cmd_header.cmd_id    = XCOM_CMDID_EXTAID;
    frame.cmd_header.specific  = 0;
    frame.command_parameter_id = XCOM_CMDEXTAID_VEL_NED;
    frame.time_stamp           = timestamp;
    frame.time_mode            = timemode;
    for(int i = 0; i < 3; i++) {
        frame.velocity[i]        = vel[i];
        frame.velocity_stddev[i] = vel_stddev[i];
    }
    complete_message(frame);
    return frame;
}
XCOMCmd_EXTAID_VELBODY XComState::get_xcomcmd_extaid_vel_body(const double& timestamp, uint16_t timemode, const std::array<double, 3>& vel,
                                                              const std::array<double, 3>& vel_stddev,
                                                              const std::array<double, 3>& leverarm,
                                                              const std::array<double, 3>& leverarm_stddev) {
    XCOMCmd_EXTAID_VELBODY frame{};
    build_header(frame, 0.0, 0, XComMessageID::XCOM_MSGID_COMMAND, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC);
    frame.cmd_header.cmd_id    = XCOM_CMDID_EXTAID;
    frame.cmd_header.specific  = 0;
    frame.command_parameter_id = XCOM_CMDEXTAID_VEL_BODY;
    frame.time_stamp           = timestamp;
    frame.time_mode            = timemode;
    for(int i = 0; i < 3; i++) {
        frame.velocity[i]         = vel[i];
        frame.velocity_stddev[i]  = vel_stddev[i];
        frame.lever_arm[i]        = leverarm[i];
        frame.lever_arm_stddev[i] = leverarm_stddev[i];
    }
    complete_message(frame);
    return frame;
}
XCOMCmd_EXTAID_HEIGHT XComState::get_xcomcmd_extaid_height(const double& timestamp, uint16_t timemode, const double& height,
                                                           const double& height_stddev) {
    XCOMCmd_EXTAID_HEIGHT frame{};
    build_header(frame, 0.0, 0, XComMessageID::XCOM_MSGID_COMMAND, XComLogTrigger::XCOM_CMDLOG_TRIG_SYNC);
    frame.cmd_header.cmd_id    = XCOM_CMDID_EXTAID;
    frame.cmd_header.specific  = 0;
    frame.command_parameter_id = XCOM_CMDEXTAID_HGT;
    frame.time_stamp           = timestamp;
    frame.time_mode            = timemode;
    frame.height               = height;
    frame.height_stddev        = height_stddev;
    complete_message(frame);
    return frame;
}
}  // namespace xcom