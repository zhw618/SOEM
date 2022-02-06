#ifndef EWDL_ETHERCAT_TXPDO_H
#define EWDL_ETHERCAT_TXPDO_H
#include <stdint.h>


/* =======  第2组: 0x1B02  ======= */
typedef struct TxPDO2
{
  uint16_t error_code;
  uint16_t status_word;
  int32_t position_actual_value;
  int32_t torque_actual_value;
  int8_t mode_of_operation_display;
  
  uint16_t touch_probe_status;
  int32_t touch_probe_pos1_pos_value;
  int32_t touch_probe_pos2_pos_value;
  uint32_t digital_inputs;

} TxPDO2t;
void TxPDO2_read_from_addr(TxPDO2t *tPDO, uint8_t *data_ptr)
{
  tPDO->error_code = 0x0000;
  tPDO->status_word = 0x0000;
  tPDO->position_actual_value = 0x00000000;
  tPDO->torque_actual_value = 0x00000000;
  tPDO->mode_of_operation_display = 0x00;
  tPDO->touch_probe_status = 0x0000;
  tPDO->touch_probe_pos1_pos_value = 0x00000000;
  tPDO->touch_probe_pos2_pos_value = 0x00000000;
  tPDO->digital_inputs = 0x00000000;

  tPDO->error_code |= (0x00FF & *data_ptr++) << 0;
  tPDO->error_code |= (0x00FF & *data_ptr++) << 8;

  tPDO->status_word |= (0x00FF & *data_ptr++) << 0;
  tPDO->status_word |= (0x00FF & *data_ptr++) << 8;

  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 0;
  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 8;
  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 16;
  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 24;

  tPDO->torque_actual_value |= (0x000000FF & *data_ptr++) << 0;
  tPDO->torque_actual_value |= (0x000000FF & *data_ptr++) << 8;
  tPDO->torque_actual_value |= (0x000000FF & *data_ptr++) << 16;
  tPDO->torque_actual_value |= (0x000000FF & *data_ptr++) << 24;

  tPDO->mode_of_operation_display |= (0xFF & *data_ptr++) << 0;

  tPDO->touch_probe_status |= (0x00FF & *data_ptr++) << 0;
  tPDO->touch_probe_status |= (0x00FF & *data_ptr++) << 8;

  tPDO->touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 0;
  tPDO->touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 8;
  tPDO->touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 16;
  tPDO->touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 24;

  tPDO->touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 0;
  tPDO->touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 8;
  tPDO->touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 16;
  tPDO->touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 24;

  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 0;
  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 8;
  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 16;
  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 24;
}



// union TxPDO
// {
//   TxPDO0 _0;
//   TxPDO1 _1;
//   TxPDO2 _2;
//   TxPDO3 _3;
// };

#endif
