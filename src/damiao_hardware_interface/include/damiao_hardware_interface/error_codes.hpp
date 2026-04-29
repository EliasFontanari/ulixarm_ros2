#ifndef ULIXARM_HARDWARE__ERROR_CODES_HPP_
#define ULIXARM_HARDWARE__ERROR_CODES_HPP_

#include <cstdio>
#include <cstdint>

enum class ErrorCode : int
{
    SUCCESS                 =  1,
 
    // Serial port errors
    PORT_NOT_OPEN           = -1,
    PORT_OPEN_FAILED        = -2,
    PORT_CONFIG_FAILED      = -3,
    WAIT_TIMEOUT            = -4,
    WAIT_SELECT_FAILED      = -5,
    READ_FAILED             = -6,
    WRONG_END_BYTE          = -7,
 
    // Motor / CAN errors
    RECEIVE_FAIL            = -10,
    SEND_FAIL               = -11,
    COMMUNICATION           = -12,
    OVERVOLTAGE             = -13,
    UNDERVOLTAGE            = -14,
    MOS_OVERTEMPERATURE     = -15,
    COIL_OVERTEMPERATURE    = -16,
    COMMUNICATION_LOSS      = -17,
    OVERLOAD                = -18,
    SERIAL_READ             = -19,
    MOTOR_NOT_FOUND         = -20,
    MASTER_ID_NOT_FOUND     = -21,
    UNKNOWN_CMD             = -22,
};

inline int to_int(ErrorCode code) { return static_cast<int>(code); }
 
inline const char* error_message(ErrorCode code)
{
    switch (code)
    {
    case ErrorCode::SUCCESS:                  return "Success.";
    // Serial
    case ErrorCode::PORT_NOT_OPEN:        return "Port not open.";
    case ErrorCode::PORT_OPEN_FAILED:     return "Failed to open serial port.";
    case ErrorCode::PORT_CONFIG_FAILED:   return "Failed to configure serial port.";
    case ErrorCode::WAIT_TIMEOUT:         return "Timeout waiting for byte.";
    case ErrorCode::WAIT_SELECT_FAILED:   return "select() error while waiting for byte.";
    case ErrorCode::READ_FAILED:          return "read() returned no data.";
    case ErrorCode::WRONG_END_BYTE:       return "Last byte does not match expected end byte.";
    // Motor / CAN
    case ErrorCode::RECEIVE_FAIL:         return "Receive fail.";
    case ErrorCode::SEND_FAIL:            return "Send fail.";
    case ErrorCode::COMMUNICATION:        return "Communication error.";
    case ErrorCode::OVERVOLTAGE:          return "Overvoltage.";
    case ErrorCode::UNDERVOLTAGE:         return "Undervoltage.";
    case ErrorCode::MOS_OVERTEMPERATURE:  return "MOS overtemperature.";
    case ErrorCode::COIL_OVERTEMPERATURE: return "Motor coil overtemperature.";
    case ErrorCode::COMMUNICATION_LOSS:   return "Communication loss.";
    case ErrorCode::OVERLOAD:             return "Overload.";
    case ErrorCode::SERIAL_READ:          return "Could not receive motor status.";
    case ErrorCode::MOTOR_NOT_FOUND:      return "Motor name not found.";
    case ErrorCode::MASTER_ID_NOT_FOUND:  return "Could not find motor by master ID.";
    case ErrorCode::UNKNOWN_CMD:          return "Unknown command byte received.";
    default:                              return "Unknown error.";
    }
}
 
// Log an error and return its integer value — used as a one-liner in return statements.
inline int log_error(ErrorCode code, const char* extra = nullptr)
{
    if (extra)
        fprintf(stderr, "[ ERROR ] %s — %s\n", error_message(code), extra);
    else
        fprintf(stderr, "[ ERROR ] %s\n", error_message(code));
    return to_int(code);
}


#endif // ULIXARM_HARDWARE__ERROR_CODES_HPP_