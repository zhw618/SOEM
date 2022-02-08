#ifndef EWDL_ETHERCAT_RXPDO_H
#define EWDL_ETHERCAT_RXPDO_H

#include <stdint.h>


/* =======  第2组: 0x1702 ======= */
typedef struct RxPDO2
{
  uint16_t control_word;
  int32_t target_position;
  int32_t target_velocity;
  int16_t target_torque;
  int8_t mode_of_operation;
  uint16_t touch_probe_function;
  uint32_t max_velocity;

} RxPDO2t;
void RxPDO2_write_to_addr(RxPDO2t *  rPDO,  uint8_t * data_ptr)
  {
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

    *data_ptr++ = (rPDO->mode_of_operation >> 0) & 0xFF;
    //data_ptr++;

    *data_ptr++ = (rPDO->touch_probe_function >> 0) & 0xFF;
    *data_ptr++ = (rPDO->touch_probe_function >> 8) & 0xFF;

    *data_ptr++ = (rPDO->max_velocity >> 0) & 0xFF;
    *data_ptr++ = (rPDO->max_velocity >> 8) & 0xFF;
    *data_ptr++ = (rPDO->max_velocity >> 16) & 0xFF;
    *data_ptr++ = (rPDO->max_velocity >> 24) & 0xFF;

  }

// union RxPDO
// {
//   RxPDO1 _0;
//   RxPDO2 _1;
//   RxPDO3 _2;
//   RxPDO4 _3;
// };

#endif
