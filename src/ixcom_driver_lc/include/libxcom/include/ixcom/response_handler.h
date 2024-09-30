/*.*******************************************************************
 FILENAME: response_handler.h
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
#ifndef LIB_IXCOM_RESPONSE_HANDLER_H
#define LIB_IXCOM_RESPONSE_HANDLER_H
#include "ixcom.h"
#include <cassert>
#include <cstring>
namespace xcom {
template<typename ClassT>
inline void xcom_response_cb_passthrough(uint16_t command_id, std::size_t frame_len, uint8_t frame[], void* context) {
    UNUSED(command_id);
    UNUSED(frame_len);
    assert(nullptr != context);
    uint16_t response;
    std::memcpy(&response, &frame[16], sizeof(uint16_t));
    auto instance = static_cast<ClassT*>(context);
    instance->handle_response(static_cast<XCOMResp>(response));
}
class ResponseHandler {
    XComState& _state;
    xcom_callbacks_node_t _callback_node;
public:
    explicit ResponseHandler(XComState* state)
        : _state(*state),
          _callback_node() {
        _state.register_response_callback(xcom_response_cb_passthrough<ResponseHandler>, this, &_callback_node);
    }
    virtual ~ResponseHandler() = default;
    ResponseHandler(const ResponseHandler&)            = delete;
    ResponseHandler(ResponseHandler&& other)           = delete;
    ResponseHandler& operator=(const ResponseHandler&) = delete;
    ResponseHandler& operator=(ResponseHandler&&)      = delete;
    virtual void handle_response(XCOMResp response) = 0;
};
}  // namespace xcom
#endif  // LIB_IXCOM_RESPONSE_HANDLER_H
