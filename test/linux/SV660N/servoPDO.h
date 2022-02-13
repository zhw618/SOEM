#ifndef __SERVO_PDO_H

#define __SERVO_PDO_H

/* 共享内存SHM交换区,数据默认为big-endian大端字节序存储 */
#define SHM_BIG_ENDIAN   


#include <stdint.h>

//**************原地逆转字节序的两个函数******************
void reverse_byte_order_uint16(uint16_t * pNum)
{
    uint8_t * pb = (uint8_t*) pNum;
    uint8_t  val ;
    //原地交换两个字节内容
    val= *(pb);
    *(pb) = *(pb+1);
    *(pb+1) = val;
}

void  reverse_byte_order_uint32(uint32_t * pNum)
{
    uint8_t * pb = (uint8_t*) pNum;
    uint8_t   val ;
    //原地交换1和4两个字节内容
    val = (*pb);
    (*pb) = *(pb+3);
    *(pb+3) = val;
    //再原地交换2和3两个字节内容
    val = *(pb+1);
    *(pb+1) = *(pb+2);
    *(pb+2) = val;
}
//**************以上: 逆转字节序的两个函数******************



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



/* ********************************************************************
 *  拷贝RxPDO数据到新空间, 根据宏定义决定是否大小端(Little/Big-endian)转换!
 * ********************************************************************/
void RxPDO1_copyTo(RxPDO1t *  rPDO,  void * data_ptr)
{
    memcpy( data_ptr, rPDO, sizeof(RxPDO1t) );

#ifdef SHM_BIG_ENDIAN
    RxPDO1t * newPDO = (RxPDO1t*) data_ptr;
    reverse_byte_order_uint16(              &(newPDO->control_word)    );
    reverse_byte_order_uint32( (uint32_t *) &(newPDO->target_position) );
    reverse_byte_order_uint32( (uint32_t *) &(newPDO->target_velocity) );
    reverse_byte_order_uint16( (uint16_t *) &(newPDO->target_torque)   );
    /* -------- 这里有个 int8 的不需要转换  ---------*/
    reverse_byte_order_uint16(              &(newPDO->probe_function)  );
    reverse_byte_order_uint32(              &(newPDO->max_velocity)    );
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


/* ***********************************************************************
 *   拷贝TxPDO数据到新空间, 根据宏定义决定是否大小端(Little/Big-endian)转换!
 * ***********************************************************************/
void TxPDO1_copyTo(TxPDO1t *  tPDO,  void * data_ptr)
{
    memcpy( data_ptr, tPDO, sizeof(TxPDO1t) );

#ifdef SHM_BIG_ENDIAN
    TxPDO1t * newPDO = (TxPDO1t*) data_ptr;
    reverse_byte_order_uint16(             &(newPDO->error_code)            );
    reverse_byte_order_uint16(             &(newPDO->status_word)           );
    reverse_byte_order_uint32( (uint32_t*) &(newPDO->current_position)      );
    reverse_byte_order_uint16( (uint16_t*) &(newPDO->current_torque)        );
    /* -------- 这里有个 int8 的不需要转换  ---------*/
    reverse_byte_order_uint16(             &(newPDO->probe_status)          );
    reverse_byte_order_uint32( (uint32_t*) &(newPDO->probe_up_edge_pos1)    );
    reverse_byte_order_uint32( (uint32_t*) &(newPDO->probe_up_edge_pos2)    );
    reverse_byte_order_uint32(             &(newPDO->digital_input_status)  );
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

#endif  //#ifndef


/* ***************************************************
 *  从小端(little-endian)存储空间, 拷贝数据到TxPDO中
 * ***************************************************/
/* --------------不再使用 -------------------
void TxPDO1_copy_from_little(TxPDO1t *tPDO, uint8_t *data_ptr)
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


}
------------------- 以上不再使用----------------------------*/