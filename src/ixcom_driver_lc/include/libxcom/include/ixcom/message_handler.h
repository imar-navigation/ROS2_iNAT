/*.*******************************************************************
 FILENAME: message_handler.h
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
#ifndef LIB_IXCOM_MESSAGE_HANDLER_H
#define LIB_IXCOM_MESSAGE_HANDLER_H
#include "ixcom.h"
#include "ixcom/message_traits.h"
#include <cassert>
namespace xcom {
class XComState;
template<typename ClassT, typename ArgT>
using CallbackMsgMemFn = void (ClassT::*)(const ArgT&) noexcept;
template<typename MsgT, typename ClassT, CallbackMsgMemFn<ClassT, MsgT> Func>
inline void xcom_msg_cb_passthrough(uint8_t* msg, void* context) noexcept {  // NOLINT
    assert(nullptr != context);
    auto instance = static_cast<ClassT*>(context);
    assert(nullptr != instance);
    auto val = reinterpret_cast<MsgT*>(msg);
    ((*instance).*(Func))(*val);
}
template<typename MsgType, typename... OtherTypes>
class CallbackInterface : CallbackInterface<OtherTypes...> {
public:
    CallbackInterface()           = default;
    ~CallbackInterface() override = default;
    using CallbackInterface<OtherTypes...>::handle_xcom_msg;
    virtual void handle_xcom_msg(const MsgType& msg) noexcept = 0;  // NOLINT
protected:
    void register_callback(XComState* state, xcom_callbacks_node_t nodes[]) noexcept {  // NOLINT
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
    virtual void handle_xcom_msg(const MsgType& msg) noexcept = 0;
protected:
    void register_callback(XComState* state, xcom_callbacks_node_t nodes[]) noexcept {  // NOLINT
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
template<typename ClassT, typename... Args>
using CallbackGenericMsgMemFn = void (ClassT::*)(Args... arg);
template<typename ClassT, CallbackGenericMsgMemFn<ClassT, XComMessageID, uint8_t[], std::size_t> Func>
inline void xcom_generic_msg_cb_passthrough(XComMessageID id, uint8_t* msg, std::size_t length, void* context) noexcept {  // NOLINT
    assert(nullptr != context);
    auto instance = static_cast<ClassT*>(context);
    assert(nullptr != instance);
    ((*instance).*(Func))(id, msg, length);
}
template<XComMessageID... MsgIds>
class GenericMessageHandler {
    static constexpr size_t IdCount                      = sizeof...(MsgIds);
    static constexpr XComMessageID AvailableIds[IdCount] = {MsgIds...};
    XComState& _state;
    std::array<xcom_callbacks_node_t, IdCount> _callback_nodes;
public:
    explicit GenericMessageHandler(XComState* state)
        : _state(*state),
          _callback_nodes() {
        for(const auto id: AvailableIds) {
            _state.register_generic_message_callback(
                id, &xcom_generic_msg_cb_passthrough<GenericMessageHandler, &GenericMessageHandler::handle_generic_xcom_msg>, this,
                &_callback_nodes[0]);
        }
    }
    virtual ~GenericMessageHandler()                               = default;
    GenericMessageHandler(const GenericMessageHandler&)            = delete;
    GenericMessageHandler(GenericMessageHandler&& other)           = delete;
    GenericMessageHandler& operator=(const GenericMessageHandler&) = delete;
    GenericMessageHandler& operator=(GenericMessageHandler&&)      = delete;
    virtual void handle_generic_xcom_msg(XComMessageID id, uint8_t msg[], std::size_t length) noexcept = 0;
};
}  // namespace xcom
#endif  // LIB_IXCOM_MESSAGE_HANDLER_H
