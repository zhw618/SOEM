#ifndef __SERVO_PDO_H
#define __SERVO_PDO_H

#include <stdint.h>

#pragma pack(push,1)
typedef struct RxPDO1
{
    /* 共7个字段, 19个字节*/
    uint16_t control_word;
    int32_t  target_position;
    int32_t  target_velocity;
    int16_t  target_torque;
    int8_t   operation_mode;  
    uint16_t probe_function;
    uint32_t max_velocity;  
} RxPDO1t;

typedef struct TxPDO1
{
    /* 共9个字段, 25个字节*/
    uint16_t error_code;
    uint16_t status_word;
    int32_t  current_position;
    int16_t  current_torque;
    int8_t   current_operation_mode;
    uint16_t probe_status;
    int32_t  probe_up_edge_pos1;
    int32_t  probe_up_edge_pos2;
    uint32_t digital_input_status;  
} TxPDO1t;

#pragma pack(pop)


/* Mode of Operation */
enum mode_of_operation_t
{
  NONE = 0,                          // None Mode
  PROFILE_POSITION = 1,              // Profile Position Mode
  PROFILE_VELOCITY = 3,              // Profile Velocity Mode
  TORQUE_PROFILE = 4,                // Torque Profile Mode
  HOMING = 6,                        // Homing Mode
  CYCLIC_SYNCHRONOUS_POSITION = 8,   // Cyclic Synchronous Position Mode
  CYCLIC_SYNCHRONOUS_VELOCITY = 9,   // Cyclic Synchronous Velocity Mode
  CYCLIC_SYNCHRONOUS_TORQUE = 10,    // Cyclic Synchronous Torque Mode
};


#endif