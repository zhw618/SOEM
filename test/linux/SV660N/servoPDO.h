#ifndef __SERVO_PDO_H
#define __SERVO_PDO_H

/* 共享内存SHM交换区,数据默认为big-endian大端字节序存储 */
//#define SHM_BIG_ENDIAN   

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
#pragma pack(pop)

void RxPDO1_copy_to(RxPDO1t *  rPDO,  uint8_t * data_ptr)
{
#ifndef SHM_BIG_ENDIAN   //SHM缓冲区数据为 Little-endian 存储的情况.

     memcpy( data_ptr, rPDO, sizeof(RxPDO1t) );   
 
#else  //SHM缓冲区数据为 Big-endian 存储的情况: 大小端转化!

    *data_ptr++ = (rPDO->control_word >> 0) & 0xFF;
    *data_ptr++ = (rPDO->control_word >> 8) & 0xFF;

    *data_ptr++ = (rPDO->target_position >> 0) & 0xFF;
    *data_ptr++ = (rPDO->target_position >> 8) & 0xFF;
    *data_ptr++ = (rPDO->target_position >> 16) & 0xFF;
    *data_ptr++ = (rPDO->target_position >> 24) & 0xFF;

    *data_ptr++ = (rPDO->target_velocity >> 0) & 0xFF;
    *data_ptr++ = (rPDO->target_velocity >> 8) & 0xFF;
    *data_ptr++ = (rPDO->target_velocity >> 16) & 0xFF;
    *data_ptr++ = (rPDO->target_velocity >> 24) & 0xFF;

    *data_ptr++ = (rPDO->target_torque >> 0) & 0xFF;
    *data_ptr++ = (rPDO->target_torque >> 8) & 0xFF;

    *data_ptr++ = (rPDO->operation_mode >> 0) & 0xFF;

    *data_ptr++ = (rPDO->probe_function >> 0) & 0xFF;
    *data_ptr++ = (rPDO->probe_function >> 8) & 0xFF;

    *data_ptr++ = (rPDO->max_velocity >> 0) & 0xFF;
    *data_ptr++ = (rPDO->max_velocity >> 8) & 0xFF;
    *data_ptr++ = (rPDO->max_velocity >> 16) & 0xFF;
    *data_ptr++ = (rPDO->max_velocity >> 24) & 0xFF;
  
#endif
}


#pragma pack(push,1)
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

void TxPDO1_copy_from(TxPDO1t *tPDO, uint8_t *data_ptr)
{
#ifndef SHM_BIG_ENDIAN   //SHM缓冲区数据为 Little-endian 存储的情况.

    memcpy( tPDO, data_ptr, sizeof(TxPDO1t) );  //直接拷贝

#else  //SHM缓冲区为 Big-endian 的情况: 大小端转化!

    tPDO->error_code = 0x0000;
    tPDO->status_word = 0x0000;
    tPDO->current_position = 0x00000000;
    tPDO->current_torque = 0x0000;
    tPDO->current_operation_mode = 0x00;
    tPDO->probe_status = 0x0000;
    tPDO->probe_up_edge_pos1 = 0x00000000;
    tPDO->probe_up_edge_pos2 = 0x00000000;
    tPDO->digital_input_status = 0x00000000;

    tPDO->error_code |= (0x00FF & *data_ptr++) << 0;
    tPDO->error_code |= (0x00FF & *data_ptr++) << 8;

    tPDO->status_word |= (0x00FF & *data_ptr++) << 0;
    tPDO->status_word |= (0x00FF & *data_ptr++) << 8;

    tPDO->current_position |= (0x000000FF & *data_ptr++) << 0;
    tPDO->current_position |= (0x000000FF & *data_ptr++) << 8;
    tPDO->current_position |= (0x000000FF & *data_ptr++) << 16;
    tPDO->current_position |= (0x000000FF & *data_ptr++) << 24;

    tPDO->current_torque |= (0x000000FF & *data_ptr++) << 0;
    tPDO->current_torque |= (0x000000FF & *data_ptr++) << 8;

    tPDO->current_operation_mode |= (0xFF & *data_ptr++) << 0;

    tPDO->probe_status |= (0x00FF & *data_ptr++) << 0;
    tPDO->probe_status |= (0x00FF & *data_ptr++) << 8;

    tPDO->probe_up_edge_pos1 |= (0x000000FF & *data_ptr++) << 0;
    tPDO->probe_up_edge_pos1 |= (0x000000FF & *data_ptr++) << 8;
    tPDO->probe_up_edge_pos1 |= (0x000000FF & *data_ptr++) << 16;
    tPDO->probe_up_edge_pos1 |= (0x000000FF & *data_ptr++) << 24;

    tPDO->probe_up_edge_pos2 |= (0x000000FF & *data_ptr++) << 0;
    tPDO->probe_up_edge_pos2 |= (0x000000FF & *data_ptr++) << 8;
    tPDO->probe_up_edge_pos2 |= (0x000000FF & *data_ptr++) << 16;
    tPDO->probe_up_edge_pos2 |= (0x000000FF & *data_ptr++) << 24;

    tPDO->digital_input_status |= (0x000000FF & *data_ptr++) << 0;
    tPDO->digital_input_status |= (0x000000FF & *data_ptr++) << 8;
    tPDO->digital_input_status |= (0x000000FF & *data_ptr++) << 16;
    tPDO->digital_input_status |= (0x000000FF & *data_ptr++) << 24;

#endif
}



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