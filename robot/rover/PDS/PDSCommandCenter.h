#ifndef PDSCOMMANDCENTER_H
#define PDSCOMMANDCENTER_H

#include "CommandCenter.h" 
#define COMMAND_PING 49
#define COMMAND_DISABLE_ALL_MOTORS 51
#define COMMAND_ENABLE_ALL_MOTORS 52
#define COMMAND_MOTOR 53
#define COMMAND_FAN 54
#define COMMAND_RESET_GENERAL_ERROR_FLAGS 55
#define COMMAND_RESET_CURRENT_READING_ERROR_FLAGS 56
#define COMMAND_TOGGLE_AUTO_MODE 57

class PDSCommandCenter : public internal_comms::CommandCenter {
    public:
        void executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength) override;
};

#endif //PDSCOMMANDCENTER_H
