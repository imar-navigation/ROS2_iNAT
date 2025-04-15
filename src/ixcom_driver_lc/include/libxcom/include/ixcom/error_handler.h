/*.*******************************************************************
 FILENAME: error_handler.h
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
#ifndef LIB_IXCOM_ERROR_HANDLER_H
#define LIB_IXCOM_ERROR_HANDLER_H
#include "ixcom.h"
#include "ixcom/xcom_parser.h"
#include <cassert>
namespace xcom {
class XComState;
using ErrorCode = XComParser::ParserCode;
template<typename ClassT, typename ArgT>
using CallbackErrMemFn = void (ClassT::*)(const ArgT&);
template<typename MsgT, typename ClassT, CallbackErrMemFn<ClassT, MsgT> Func>
inline void xcom_err_cb_passthrough(const ErrorCode& error_code, void* context) noexcept {
    assert(nullptr != context);
    auto instance = static_cast<ClassT*>(context);
    ((*instance).*(Func))(error_code);
}
class ErrorCallbackInterface {
public:
    ErrorCallbackInterface()          = default;
    virtual ~ErrorCallbackInterface() = default;
    virtual void handle_xcom_error(const ErrorCode& ec) noexcept = 0;
protected:
    void register_callback(XComState* state, xcom_callbacks_node_t nodes[]) noexcept {
        state->register_error_callback(
            &xcom_err_cb_passthrough<ErrorCode, ErrorCallbackInterface, &ErrorCallbackInterface::handle_xcom_error>, this, &nodes[0]);
    }
};
class ErrorHandler : public ErrorCallbackInterface {
    static constexpr size_t MsgCount = 1;
    XComState& _state;
    std::array<xcom_callbacks_node_t, MsgCount> _callback_nodes;
public:
    explicit ErrorHandler(XComState* state)
        : ErrorCallbackInterface(),
          _state(*state),
          _callback_nodes() {
        ErrorCallbackInterface::register_callback(&_state, _callback_nodes.data());
    }
    ~ErrorHandler() override                     = default;
    ErrorHandler(const ErrorHandler&)            = delete;
    ErrorHandler(ErrorHandler&& other)           = delete;
    ErrorHandler& operator=(const ErrorHandler&) = delete;
    ErrorHandler& operator=(ErrorHandler&&)      = delete;
    using ErrorCallbackInterface::handle_xcom_error;
};
}  // namespace xcom
#endif  // LIB_IXCOM_ERROR_HANDLER_H
