#ifndef EWDL_ETHERCAT_RXPDO_H
#define EWDL_ETHERCAT_RXPDO_H

#include <stdint.h>

/* =======  第0组: 0x1600 ======= */
typedef struct RxPDO0
{
  uint16_t control_word;
  int32_t target_position;
  uint16_t touch_probe_function;
  
  /* 以下是未使用的 */
  //int8_t mode_of_operation;
  //uint32_t physical_outputs;

} RxPDO0t;
void RxPDO0_write_to_addr(RxPDO0t * rPDO,  uint8_t * data_ptr)
{
    *data_ptr++ = (rPDO->control_word >> 0) & 0xFF;
    *data_ptr++ = (rPDO->control_word >> 8) & 0xFF;

    //*data_ptr++ = (rPDO->mode_of_operation >> 0) & 0xFF;

    *data_ptr++ = (rPDO->target_position >> 0) & 0xFF;
    *data_ptr++ = (rPDO->target_position >> 8) & 0xFF;
    *data_ptr++ = (rPDO->target_position >> 16) & 0xFF;
    *data_ptr++ = (rPDO->target_position >> 24) & 0xFF;

    *data_ptr++ = (rPDO->touch_probe_function >> 0) & 0xFF;
    *data_ptr++ = (rPDO->touch_probe_function >> 8) & 0xFF;

    // *data_ptr++ = (rPDO->physical_outputs >> 0) & 0xFF;
    // *data_ptr++ = (rPDO->physical_outputs >> 8) & 0xFF;
    // *data_ptr++ = (rPDO->physical_outputs >> 16) & 0xFF;
    // *data_ptr++ = (rPDO->physical_outputs >> 24) & 0xFF;
}


/* =======  第1组: 0x1701 ======= */
typedef struct RxPDO1
{
  uint16_t control_word;
  int32_t target_position;
  uint16_t touch_probe_function;
  uint32_t physical_outputs;
  
  /* 以下是未使用的 */
  //int8_t mode_of_operation;

} RxPDO1t;
void RxPDO1_write_to_addr(RxPDO1t * rPDO,  uint8_t * data_ptr)
{
    *data_ptr++ = (rPDO->control_word >> 0) & 0xFF;
    *data_ptr++ = (rPDO->control_word >> 8) & 0xFF;

    //*data_ptr++ = (rPDO->mode_of_operation >> 0) & 0xFF;

    *data_ptr++ = (rPDO->target_position >> 0) & 0xFF;
    *data_ptr++ = (rPDO->target_position >> 8) & 0xFF;
    *data_ptr++ = (rPDO->target_position >> 16) & 0xFF;
    *data_ptr++ = (rPDO->target_position >> 24) & 0xFF;

    *data_ptr++ = (rPDO->touch_probe_function >> 0) & 0xFF;
    *data_ptr++ = (rPDO->touch_probe_function >> 8) & 0xFF;

    *data_ptr++ = (rPDO->physical_outputs >> 0) & 0xFF;
    *data_ptr++ = (rPDO->physical_outputs >> 8) & 0xFF;
    *data_ptr++ = (rPDO->physical_outputs >> 16) & 0xFF;
    *data_ptr++ = (rPDO->physical_outputs >> 24) & 0xFF;
}


/* =======  第2组: 0x1702 ======= */
typedef struct RxPDO2
{
  uint16_t control_word;
  int32_t target_position;
  int32_t target_velocity;
  int32_t target_torque;
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
    *data_ptr++ = (rPDO->target_torque >> 16) & 0xFF;
    *data_ptr++ = (rPDO->target_torque >> 24) & 0xFF;

    *data_ptr++ = (rPDO->mode_of_operation >> 0) & 0xFF;

    *data_ptr++ = (rPDO->touch_probe_function >> 0) & 0xFF;
    *data_ptr++ = (rPDO->touch_probe_function >> 8) & 0xFF;

    *data_ptr++ = (rPDO->max_velocity >> 0) & 0xFF;
    *data_ptr++ = (rPDO->max_velocity >> 8) & 0xFF;
    *data_ptr++ = (rPDO->max_velocity >> 16) & 0xFF;
    *data_ptr++ = (rPDO->max_velocity >> 24) & 0xFF;

  }

/* =======  第3组: 0x1703 暂未核对 ======= */
typedef struct RxPDO3
{
  uint16_t control_word;
  int8_t mode_of_operation;
  int32_t target_position;
  uint32_t profile_velocity;
  uint32_t physical_outputs;

  // std::stream operator <<(std::stream &os, const T &obj)
  // {
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

} RxPDO3t;
void RxPDO3_write_to_addr(RxPDO3t *  rPDO,  uint8_t * data_ptr)
  {
    *data_ptr++ = (rPDO->control_word >> 0) & 0xFF;
    *data_ptr++ = (rPDO->control_word >> 8) & 0xFF;

    *data_ptr++ = (rPDO->mode_of_operation >> 0) & 0xFF;

    *data_ptr++ = (rPDO->target_position >> 0) & 0xFF;
    *data_ptr++ = (rPDO->target_position >> 8) & 0xFF;
    *data_ptr++ = (rPDO->target_position >> 16) & 0xFF;
    *data_ptr++ = (rPDO->target_position >> 24) & 0xFF;

    *data_ptr++ = (rPDO->profile_velocity >> 0) & 0xFF;
    *data_ptr++ = (rPDO->profile_velocity >> 8) & 0xFF;
    *data_ptr++ = (rPDO->profile_velocity >> 16) & 0xFF;
    *data_ptr++ = (rPDO->profile_velocity >> 24) & 0xFF;

    *data_ptr++ = (rPDO->physical_outputs >> 0) & 0xFF;
    *data_ptr++ = (rPDO->physical_outputs >> 8) & 0xFF;
    *data_ptr++ = (rPDO->physical_outputs >> 16) & 0xFF;
    *data_ptr++ = (rPDO->physical_outputs >> 24) & 0xFF;
  }


/* =======  第4组: 0x1704 暂未核对 ======= */
typedef struct RxPDO4
{
  uint16_t control_word;
  int8_t mode_of_operation;
  int32_t target_velocity;
  uint32_t physical_outputs;

  // std::stream operator <<(std::stream &os, const T &obj)
  // {
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

} RxPDO4t;
void RxPDO4_write_to_addr(RxPDO4t *  rPDO,  uint8_t * data_ptr)
  {
    *data_ptr++ = (rPDO->control_word >> 0) & 0xFF;
    *data_ptr++ = (rPDO->control_word >> 8) & 0xFF;

    *data_ptr++ = (rPDO->mode_of_operation >> 0) & 0xFF;

    *data_ptr++ = (rPDO->target_velocity >> 0) & 0xFF;
    *data_ptr++ = (rPDO->target_velocity >> 8) & 0xFF;
    *data_ptr++ = (rPDO->target_velocity >> 16) & 0xFF;
    *data_ptr++ = (rPDO->target_velocity >> 24) & 0xFF;

    *data_ptr++ = (rPDO->physical_outputs >> 0) & 0xFF;
    *data_ptr++ = (rPDO->physical_outputs >> 8) & 0xFF;
    *data_ptr++ = (rPDO->physical_outputs >> 16) & 0xFF;
    *data_ptr++ = (rPDO->physical_outputs >> 24) & 0xFF;
  }

// union RxPDO
// {
//   RxPDO1 _0;
//   RxPDO2 _1;
//   RxPDO3 _2;
//   RxPDO4 _3;
// };

#endif
