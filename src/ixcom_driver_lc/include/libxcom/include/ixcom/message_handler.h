/*.*******************************************************************
 FILENAME: message_handler.h
 **********************************************************************
 *  PROJECT: iNAT
 *  MODULE NAME: message_handler
 *  DESIGNER: T. Schneider
 *
 * 	CHANGE HISTORY:
 *
 * 	1.0 - 05.03.21: T. Schneider - File created
 *---------------------------------------------------------------------
 * 	Copyright 2021, iMAR Navigation
 *---------------------------------------------------------------------
 * 	MODULE DESCRIPTION:
 *
 ---------------------------------------------------------------------*/

#ifndef LIB_IXCOM_MESSAGE_HANDLER_H
#define LIB_IXCOM_MESSAGE_HANDLER_H

#include "../../src/message_traits.h"
#include "ixcom.h"
#include <cassert>

namespace xcom {
class XComState;
template<typename ClassT, typename ArgT>
using CallbackMsgMemFn = void (ClassT::*)(const ArgT&);

template<typename MsgT, typename ClassT, CallbackMsgMemFn<ClassT, MsgT> Func>
inline void xcom_msg_cb_passthrough(uint8_t* msg, void* context) {  // NOLINT
    assert(nullptr != context);

    auto instance = static_cast<ClassT*>(context);
    auto val      = reinterpret_cast<MsgT*>(msg);
    ((*instance).*(Func))(*val);
}

template<typename MsgType, typename... OtherTypes>
class CallbackInterface : CallbackInterface<OtherTypes...> {
public:
    CallbackInterface()           = default;
    ~CallbackInterface() override = default;

    using CallbackInterface<OtherTypes...>::handle_xcom_msg;
    virtual void handle_xcom_msg(const MsgType& msg) = 0;

protected:
    void register_callback(XComState* state, xcom_callbacks_node_t nodes[]) {  // NOLINT
        state->register_message_callback(MessageTraits<MsgType>::Id,
                                         &xcom_msg_cb_passthrough<MsgType, CallbackInterface, &CallbackInterface::handle_xcom_msg>, this,
                                         &nodes[0]);
        CallbackInterface<OtherTypes...>::register_callback(state, &nodes[1]);
    }
};

template<typename MsgType>
class CallbackInterface<MsgType> {
public:
    CallbackInterface()          = default;
    virtual ~CallbackInterface() = default;

    virtual void handle_xcom_msg(const MsgType& msg) = 0;

protected:
    void register_callback(XComState* state, xcom_callbacks_node_t nodes[]) {
        state->register_message_callback(MessageTraits<MsgType>::Id,
                                         &xcom_msg_cb_passthrough<MsgType, CallbackInterface, &CallbackInterface::handle_xcom_msg>, this,
                                         &nodes[0]);
    }
};

template<typename... MsgTypes>
class MessageHandler : public CallbackInterface<MsgTypes...> {
    static constexpr size_t MsgCount = sizeof...(MsgTypes);

    XComState& _state;
    std::array<xcom_callbacks_node_t, MsgCount> _callback_nodes;

public:
    explicit MessageHandler(XComState* state)
        : CallbackInterface<MsgTypes...>(),
          _state(*state),
          _callback_nodes() {
        CallbackInterface<MsgTypes...>::register_callback(&_state, _callback_nodes.data());
    }

    ~MessageHandler() override                       = default;
    MessageHandler(const MessageHandler&)            = delete;
    MessageHandler(MessageHandler&& other)           = delete;
    MessageHandler& operator=(const MessageHandler&) = delete;
    MessageHandler& operator=(MessageHandler&&)      = delete;

    using CallbackInterface<MsgTypes...>::handle_xcom_msg;
};
}  // namespace xcom

#endif  // LIB_IXCOM_MESSAGE_HANDLER_H
