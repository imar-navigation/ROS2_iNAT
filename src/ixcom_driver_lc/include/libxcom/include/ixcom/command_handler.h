/*.*******************************************************************
 FILENAME: command_handler.h
 **********************************************************************
 *  PROJECT: iNAT
 *  MODULE NAME: command_handler
 *  DESIGNER: T. Schneider
 *
 * 	CHANGE HISTORY:
 *
 * 	1.0 - 21.04.21: T. Schneider - File created
 *---------------------------------------------------------------------
 * 	Copyright 2021, iMAR Navigation
 *---------------------------------------------------------------------
 * 	MODULE DESCRIPTION:
 *
 ---------------------------------------------------------------------*/

#ifndef LIB_IXCOM_COMMAND_HANDLER_H
#define LIB_IXCOM_COMMAND_HANDLER_H

#include "ixcom.h"
#include <cassert>

namespace xcom {

template<typename ClassT>
inline void xcom_command_cb_passthrough(uint16_t command_id, std::size_t frame_len, uint8_t frame[], void* context) {
    assert(nullptr != context);
    auto instance = static_cast<ClassT*>(context);
    instance->handle_command(command_id, frame_len, frame);
}

class CommandHandler {
    XComState& _state;
    xcom_callbacks_node_t _callback_node;

public:
    explicit CommandHandler(XComState* state)
        : _state(*state),
          _callback_node() {
        _state.register_command_callback(xcom_command_cb_passthrough<CommandHandler>, this, &_callback_node);
    }

    virtual ~CommandHandler() = default;

    CommandHandler(const CommandHandler&)            = delete;
    CommandHandler(CommandHandler&& other)           = delete;
    CommandHandler& operator=(const CommandHandler&) = delete;
    CommandHandler& operator=(CommandHandler&&)      = delete;

    virtual void handle_command(uint16_t cmd_id, std::size_t frame_len, uint8_t frame[]) = 0;
};

}  // namespace xcom
#endif  // LIB_IXCOM_COMMAND_HANDLER_H
