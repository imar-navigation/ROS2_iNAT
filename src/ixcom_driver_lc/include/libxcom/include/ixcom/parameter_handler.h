/*.*******************************************************************
 FILENAME: parameter_handler.h
 **********************************************************************
 *  PROJECT: iNAT
 *  MODULE NAME: parameter_handler
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

#ifndef LIB_IXCOM_PARAMETER_HANDLER_H
#define LIB_IXCOM_PARAMETER_HANDLER_H

#include "../../src/parameter_traits.h"
#include "ixcom.h"
#include <cassert>

namespace xcom {
class XComState;
template<typename ClassT, typename ArgT>
using CallbackParamMemFn = void (ClassT::*)(const ArgT&);

template<typename ParamT, typename ClassT, CallbackParamMemFn<ClassT, ParamT> Func>
inline void xcom_param_cb_passthrough(uint8_t* param, void* context) {  // NOLINT
    assert(nullptr != context);

    auto instance = static_cast<ClassT*>(context);
    auto val      = reinterpret_cast<ParamT*>(param);
    ((*instance).*(Func))(*val);
}

template<typename ParamType, typename... OtherTypes>
class ParamCallbackInterface : ParamCallbackInterface<OtherTypes...> {
public:
    ParamCallbackInterface()           = default;
    ~ParamCallbackInterface() override = default;

    using ParamCallbackInterface<OtherTypes...>::handle_xcom_param;
    virtual void handle_xcom_param(const ParamType& msg) = 0;

protected:
    void register_callback(XComState* state, xcom_callbacks_node_t nodes[]) {  // NOLINT
        state->register_parameter_callback(
            ParameterTraits<ParamType>::Id,
            &xcom_param_cb_passthrough<ParamType, ParamCallbackInterface, &ParamCallbackInterface::handle_xcom_param>, this, &nodes[0]);
        ParamCallbackInterface<OtherTypes...>::register_callback(state, &nodes[1]);
    }
};

template<typename ParamType>
class ParamCallbackInterface<ParamType> {
public:
    ParamCallbackInterface()          = default;
    virtual ~ParamCallbackInterface() = default;

    virtual void handle_xcom_param(const ParamType& param) = 0;

protected:
    void register_callback(XComState* state, xcom_callbacks_node_t nodes[]) {
        state->register_parameter_callback(
            ParameterTraits<ParamType>::Id,
            &xcom_param_cb_passthrough<ParamType, ParamCallbackInterface, &ParamCallbackInterface::handle_xcom_param>, this, &nodes[0]);
    }
};

template<typename... ParamTypes>
class ParameterHandler : public ParamCallbackInterface<ParamTypes...> {
    static constexpr size_t MsgCount = sizeof...(ParamTypes);

    XComState& _state;
    std::array<xcom_callbacks_node_t, MsgCount> _callback_nodes;

public:
    explicit ParameterHandler(XComState* state)
        : ParamCallbackInterface<ParamTypes...>(),
          _state(*state),
          _callback_nodes() {
        ParamCallbackInterface<ParamTypes...>::register_callback(&_state, _callback_nodes.data());
    }

    ~ParameterHandler() override                         = default;
    ParameterHandler(const ParameterHandler&)            = delete;
    ParameterHandler(ParameterHandler&& other)           = delete;
    ParameterHandler& operator=(const ParameterHandler&) = delete;
    ParameterHandler& operator=(ParameterHandler&&)      = delete;

    using ParamCallbackInterface<ParamTypes...>::handle_xcom_param;
};
}  // namespace xcom

#endif  // LIB_IXCOM_PARAMETER_HANDLER_H
