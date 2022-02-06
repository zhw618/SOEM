#ifndef EWDL_ETHERCAT_TXPDO_H
#define EWDL_ETHERCAT_TXPDO_H
#include <stdint.h>

/* =======  第0组: 0x1A00  ======= */
typedef struct TxPDO0
{
  uint16_t error_code;
  uint16_t status_word;
  //int8_t mode_of_operation_display;
  int32_t position_actual_value;
  //int32_t torque_actual_value;
  //int32_t position_diff;  //位置偏差
  
  //int32_t follow_error_actual_value;
  int32_t touch_probe_pos2_pos_value;
  //int32_t touch_probe_pos2_neg_value;
  uint16_t touch_probe_status;
  int32_t touch_probe_pos1_pos_value;
  //int32_t touch_probe_pos1_neg_value;

  uint32_t digital_inputs;

} TxPDO0t;
void TxPDO0_read_from_addr(TxPDO0t *tPDO, uint8_t *data_ptr)
{
  tPDO->error_code = 0x0000;
  tPDO->status_word = 0x0000;
  //tPDO->mode_of_operation_display = 0x00;
  tPDO->position_actual_value = 0x00000000;
  //tPDO->torque_actual_value = 0x00000000;
  //tPDO->position_diff = 0x00000000;
  //tPDO->follow_error_actual_value = 0x00000000;
  tPDO->touch_probe_pos2_pos_value = 0x00000000;
  //tPDO->touch_probe_pos2_neg_value = 0x00000000;
  tPDO->touch_probe_status = 0x0000;
  tPDO->touch_probe_pos1_pos_value = 0x00000000;
  //tPDO->touch_probe_pos1_neg_value = 0x00000000;
  tPDO->digital_inputs = 0x00000000;

  tPDO->error_code |= (0x00FF & *data_ptr++) << 0;
  tPDO->error_code |= (0x00FF & *data_ptr++) << 8;

  tPDO->status_word |= (0x00FF & *data_ptr++) << 0;
  tPDO->status_word |= (0x00FF & *data_ptr++) << 8;

  //tPDO->mode_of_operation_display |= (0xFF & *data_ptr++) << 0;

  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 0;
  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 8;
  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 16;
  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 24;

  // tPDO->torque_actual_value |= (0x000000FF & *data_ptr++) << 0;
  // tPDO->torque_actual_value |= (0x000000FF & *data_ptr++) << 8;
  // tPDO->torque_actual_value |= (0x000000FF & *data_ptr++) << 16;
  // tPDO->torque_actual_value |= (0x000000FF & *data_ptr++) << 24;

  tPDO->touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 0;
  tPDO->touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 8;
  tPDO->touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 16;
  tPDO->touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 24;

  // tPDO->touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 0;
  // tPDO->touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 8;
  // tPDO->touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 16;
  // tPDO->touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 24;

  tPDO->touch_probe_status |= (0x00FF & *data_ptr++) << 0;
  tPDO->touch_probe_status |= (0x00FF & *data_ptr++) << 8;

  tPDO->touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 0;
  tPDO->touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 8;
  tPDO->touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 16;
  tPDO->touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 24;

  // tPDO->touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 0;
  // tPDO->touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 8;
  // tPDO->touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 16;
  // tPDO->touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 24;

  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 0;
  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 8;
  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 16;
  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 24;
}



/* =======  第1组: 0x1B01  ======= */
typedef struct TxPDO1
{
  uint16_t error_code;
  uint16_t status_word;
  //int8_t mode_of_operation_display;
  int32_t position_actual_value;
  int32_t torque_actual_value;
  int32_t position_diff;  //位置偏差
  
  //int32_t follow_error_actual_value;
  uint16_t touch_probe_status;
  int32_t touch_probe_pos1_pos_value;
  //int32_t touch_probe_pos1_neg_value;
  int32_t touch_probe_pos2_pos_value;
  //int32_t touch_probe_pos2_neg_value;
  uint32_t digital_inputs;

} TxPDO1t;
void TxPDO1_read_from_addr(TxPDO1t *tPDO, uint8_t *data_ptr)
{
  tPDO->error_code = 0x0000;
  tPDO->status_word = 0x0000;
  //tPDO->mode_of_operation_display = 0x00;
  tPDO->position_actual_value = 0x00000000;
  tPDO->torque_actual_value = 0x00000000;
  tPDO->position_diff = 0x00000000;
  //tPDO->follow_error_actual_value = 0x00000000;
  tPDO->touch_probe_status = 0x0000;
  tPDO->touch_probe_pos1_pos_value = 0x00000000;
  //tPDO->touch_probe_pos1_neg_value = 0x00000000;
  tPDO->touch_probe_pos2_pos_value = 0x00000000;
  //tPDO->touch_probe_pos2_neg_value = 0x00000000;
  tPDO->digital_inputs = 0x00000000;

  tPDO->error_code |= (0x00FF & *data_ptr++) << 0;
  tPDO->error_code |= (0x00FF & *data_ptr++) << 8;

  tPDO->status_word |= (0x00FF & *data_ptr++) << 0;
  tPDO->status_word |= (0x00FF & *data_ptr++) << 8;

  //tPDO->mode_of_operation_display |= (0xFF & *data_ptr++) << 0;

  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 0;
  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 8;
  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 16;
  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 24;

  tPDO->torque_actual_value |= (0x000000FF & *data_ptr++) << 0;
  tPDO->torque_actual_value |= (0x000000FF & *data_ptr++) << 8;
  tPDO->torque_actual_value |= (0x000000FF & *data_ptr++) << 16;
  tPDO->torque_actual_value |= (0x000000FF & *data_ptr++) << 24;

  tPDO->position_diff |= (0x000000FF & *data_ptr++) << 0;
  tPDO->position_diff |= (0x000000FF & *data_ptr++) << 8;
  tPDO->position_diff |= (0x000000FF & *data_ptr++) << 16;
  tPDO->position_diff |= (0x000000FF & *data_ptr++) << 24;

  tPDO->touch_probe_status |= (0x00FF & *data_ptr++) << 0;
  tPDO->touch_probe_status |= (0x00FF & *data_ptr++) << 8;

  tPDO->touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 0;
  tPDO->touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 8;
  tPDO->touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 16;
  tPDO->touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 24;

  // tPDO->touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 0;
  // tPDO->touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 8;
  // tPDO->touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 16;
  // tPDO->touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 24;

  tPDO->touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 0;
  tPDO->touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 8;
  tPDO->touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 16;
  tPDO->touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 24;

  // tPDO->touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 0;
  // tPDO->touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 8;
  // tPDO->touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 16;
  // tPDO->touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 24;

  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 0;
  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 8;
  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 16;
  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 24;
}


/* =======  第2组: 0x1B02  ======= */
typedef struct TxPDO2
{
  uint16_t error_code;
  uint16_t status_word;
  int32_t position_actual_value;
  int32_t torque_actual_value;
  //int32_t position_diff;  //位置偏差
  int8_t mode_of_operation_display;
  
  //int32_t follow_error_actual_value;
  uint16_t touch_probe_status;
  int32_t touch_probe_pos1_pos_value;
  //int32_t touch_probe_pos1_neg_value;
  int32_t touch_probe_pos2_pos_value;
  //int32_t touch_probe_pos2_neg_value;
  uint32_t digital_inputs;

} TxPDO2t;
void TxPDO2_read_from_addr(TxPDO2t *tPDO, uint8_t *data_ptr)
{
  tPDO->error_code = 0x0000;
  tPDO->status_word = 0x0000;
  tPDO->position_actual_value = 0x00000000;
  tPDO->torque_actual_value = 0x00000000;
  //tPDO->velocity_actual_value = 0x00000000;
  tPDO->mode_of_operation_display = 0x00;
  //tPDO->follow_error_actual_value = 0x00000000;
  tPDO->touch_probe_status = 0x0000;
  tPDO->touch_probe_pos1_pos_value = 0x00000000;
  //tPDO->touch_probe_pos1_neg_value = 0x00000000;
  tPDO->touch_probe_pos2_pos_value = 0x00000000;
  //tPDO->touch_probe_pos2_neg_value = 0x00000000;
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

  // tPDO->follow_error_actual_value |= (0x000000FF & *data_ptr++) << 0;
  // tPDO->follow_error_actual_value |= (0x000000FF & *data_ptr++) << 8;
  // tPDO->follow_error_actual_value |= (0x000000FF & *data_ptr++) << 16;
  // tPDO->follow_error_actual_value |= (0x000000FF & *data_ptr++) << 24;

  tPDO->touch_probe_status |= (0x00FF & *data_ptr++) << 0;
  tPDO->touch_probe_status |= (0x00FF & *data_ptr++) << 8;

  tPDO->touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 0;
  tPDO->touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 8;
  tPDO->touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 16;
  tPDO->touch_probe_pos1_pos_value |= (0x000000FF & *data_ptr++) << 24;

  // tPDO->touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 0;
  // tPDO->touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 8;
  // tPDO->touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 16;
  // tPDO->touch_probe_pos1_neg_value |= (0x000000FF & *data_ptr++) << 24;

  tPDO->touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 0;
  tPDO->touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 8;
  tPDO->touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 16;
  tPDO->touch_probe_pos2_pos_value |= (0x000000FF & *data_ptr++) << 24;

  // tPDO->touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 0;
  // tPDO->touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 8;
  // tPDO->touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 16;
  // tPDO->touch_probe_pos2_neg_value |= (0x000000FF & *data_ptr++) << 24;

  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 0;
  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 8;
  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 16;
  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 24;
}


/* =======  第3组: 0x1B03   未核对  ======= */
typedef struct TxPDO3
{
  uint16_t error_code;
  uint16_t status_word;
  int8_t mode_of_operation_display;
  int32_t position_actual_value;
  int32_t velocity_actual_value;
  int32_t follow_error_actual_value;
  uint32_t digital_inputs;

  // std::stream operator <<(std::stream &os, const T &obj) {
  //   os << "RxPDO0:\n");
  //   printf("\tErrorCode: %x\n", rx_pdo.error_code);
  //   printf("\tStatusWord: %d\n", rx_pdo.status_word);
  //   printf("\tModes of Operation Display: %d\n", rx_pdo.modes_of_operation_display);
  //   printf("\tPosition Actual Value: %d\n", rx_pdo.position_actual_value);
  //   printf("\tFollow Error Actual Value: %d\n", rx_pdo.follow_error_actual_value);
  //   printf("\tTouch Probe Status: %d\n", rx_pdo.touch_probe_status);
  //   printf("\tTouch probe pos1 neg value: %d\n", rx_pdo.touch_probe_pos1_neg_value);
  //   printf("\tTouch probe pos1 pos value: %d\n", rx_pdo.touch_probe_pos1_pos_value);
  //   printf("\tTouch probe pos2 pos value: %d\n", rx_pdo.touch_probe_pos2_pos_value);
  //   printf("\tTouch probe pos2 neg value: %d\n", rx_pdo.touch_probe_pos2_neg_value);
  //   printf("\tDigital Inputs: %.32x\n", rx_pdo.digital_inputs);
  // }

} TxPDO3t;
void TxPDO3_read_from_addr(TxPDO3t *tPDO, uint8_t *data_ptr)
{
  tPDO->error_code = 0x0000;
  tPDO->status_word = 0x0000;
  tPDO->mode_of_operation_display = 0x00;
  tPDO->position_actual_value = 0x00000000;
  tPDO->velocity_actual_value = 0x00000000;
  tPDO->follow_error_actual_value = 0x00000000;
  tPDO->digital_inputs = 0x00000000;

  tPDO->error_code |= (0x00FF & *data_ptr++) << 0;
  tPDO->error_code |= (0x00FF & *data_ptr++) << 8;

  tPDO->status_word |= (0x00FF & *data_ptr++) << 0;
  tPDO->status_word |= (0x00FF & *data_ptr++) << 8;

  tPDO->mode_of_operation_display |= (0xFF & *data_ptr++) << 0;

  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 0;
  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 8;
  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 16;
  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 24;

  tPDO->velocity_actual_value |= (0x000000FF & *data_ptr++) << 0;
  tPDO->velocity_actual_value |= (0x000000FF & *data_ptr++) << 8;
  tPDO->velocity_actual_value |= (0x000000FF & *data_ptr++) << 16;
  tPDO->velocity_actual_value |= (0x000000FF & *data_ptr++) << 24;

  tPDO->follow_error_actual_value |= (0x000000FF & *data_ptr++) << 0;
  tPDO->follow_error_actual_value |= (0x000000FF & *data_ptr++) << 8;
  tPDO->follow_error_actual_value |= (0x000000FF & *data_ptr++) << 16;
  tPDO->follow_error_actual_value |= (0x000000FF & *data_ptr++) << 24;

  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 0;
  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 8;
  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 16;
  tPDO->digital_inputs |= (0x000000FF & *data_ptr++) << 24;
}


/* =======  第4组: 0x1B04   未核对  ======= */
typedef struct TxPDO4
{
  uint16_t error_code;
  uint16_t status_word;
  int8_t mode_of_operation_display;
  int32_t position_actual_value;
  int32_t velocity_actual_value;
  int32_t follow_error_actual_value;
  uint32_t digital_inputs;

  // std::stream operator <<(std::stream &os, const T &obj) {
  //   os << "RxPDO0:\n");
  //   printf("\tErrorCode: %x\n", rx_pdo.error_code);
  //   printf("\tStatusWord: %d\n", rx_pdo.status_word);
  //   printf("\tModes of Operation Display: %d\n", rx_pdo.modes_of_operation_display);
  //   printf("\tPosition Actual Value: %d\n", rx_pdo.position_actual_value);
  //   printf("\tFollow Error Actual Value: %d\n", rx_pdo.follow_error_actual_value);
  //   printf("\tTouch Probe Status: %d\n", rx_pdo.touch_probe_status);
  //   printf("\tTouch probe pos1 neg value: %d\n", rx_pdo.touch_probe_pos1_neg_value);
  //   printf("\tTouch probe pos1 pos value: %d\n", rx_pdo.touch_probe_pos1_pos_value);
  //   printf("\tTouch probe pos2 pos value: %d\n", rx_pdo.touch_probe_pos2_pos_value);
  //   printf("\tTouch probe pos2 neg value: %d\n", rx_pdo.touch_probe_pos2_neg_value);
  //   printf("\tDigital Inputs: %.32x\n", rx_pdo.digital_inputs);
  // }

} TxPDO4t;
void TxPDO4_read_from_addr(TxPDO4t *tPDO, uint8_t *data_ptr)
{
  tPDO->error_code = 0x0000;
  tPDO->status_word = 0x0000;
  tPDO->mode_of_operation_display = 0x00;
  tPDO->position_actual_value = 0x00000000;
  tPDO->velocity_actual_value = 0x00000000;
  tPDO->follow_error_actual_value = 0x00000000;
  tPDO->digital_inputs = 0x00000000;

  tPDO->error_code |= (0x00FF & *data_ptr++) << 0;
  tPDO->error_code |= (0x00FF & *data_ptr++) << 8;

  tPDO->status_word |= (0x00FF & *data_ptr++) << 0;
  tPDO->status_word |= (0x00FF & *data_ptr++) << 8;

  tPDO->mode_of_operation_display |= (0xFF & *data_ptr++) << 0;

  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 0;
  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 8;
  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 16;
  tPDO->position_actual_value |= (0x000000FF & *data_ptr++) << 24;

  tPDO->velocity_actual_value |= (0x000000FF & *data_ptr++) << 0;
  tPDO->velocity_actual_value |= (0x000000FF & *data_ptr++) << 8;
  tPDO->velocity_actual_value |= (0x000000FF & *data_ptr++) << 16;
  tPDO->velocity_actual_value |= (0x000000FF & *data_ptr++) << 24;

  tPDO->follow_error_actual_value |= (0x000000FF & *data_ptr++) << 0;
  tPDO->follow_error_actual_value |= (0x000000FF & *data_ptr++) << 8;
  tPDO->follow_error_actual_value |= (0x000000FF & *data_ptr++) << 16;
  tPDO->follow_error_actual_value |= (0x000000FF & *data_ptr++) << 24;

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
