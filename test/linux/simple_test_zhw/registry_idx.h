#ifndef EWDL_ETHERCAT_REGISTRY_IDX_H
#define EWDL_ETHERCAT_REGISTRY_IDX_H

#define SYNC_MANAGER_OUTPUT_PARAMETER_IDX           0x1C32
#define SYNC_MANAGER_INPUT_PARAMETER_IDX            0x1C33
// CiA 402
#define ERROR_CODE_IDX                              0x603F
#define CONTROL_WORD_IDX                            0x6040
#define STATUS_WORD_IDX                             0x6041
#define MODE_OF_OPERATION_IDX                       0x6060
#define MODE_OF_OPERATION_DISPLAY_IDX               0x6061
#define POSITION_ACTUAL_VALUE_IDX                   0x6064
#define FOLLOWING_ERROR_WINDOW_IDX                  0x6065
#define VELOCITY_ACTUAL_VALUE_IDX                   0x606C
#define TARGET_TORQUE_IDX                           0x6071
#define MAX_CURRENT_IDX                             0x6073
#define TORQUE_DEMAND_VALUE_IDX                     0x6074
#define CURRENT_ACTUAL_VALUE_IDX                    0x6078
#define TARGET_POSITION_IDX                         0x607A
#define HOME_OFFSET_IDX                             0x607C
#define MAX_PROFILE_SPEED_IDX                       0x607F
#define PROFILE_VELOCITY_IDX                        0x6081
#define PROFILE_ACCELERATION_IDX                    0x6083
#define PROFILE_DECELERATION_IDX                    0x6084
#define QUICKSTOP_DECELERATION_IDX                  0x6085
#define TORQUE_SLOPE_IDX                            0x6087
#define HOMING_METHOD_IDX                           0x6098
#define HOMING_SPEED_IDX                            0x6099
#define HOMING_ACCELERATION_IDX                     0x609A
#define TOUCH_PROBE_FUNCTION_IDX                    0x60B8
#define TOUCH_PROBE_STATUS_IDX                      0x60B9
#define TOUCH_PROBE_POSITION_1_POSITIVE_VALUE_IDX   0x60BA
#define TOUCH_PROBE_POSITION_1_NEGATIVE_VALUE_IDX   0x60BB
#define TOUCH_PROBE_POSITION_2_POSITIVE_VALUE_IDX   0x60BC
#define TOUCH_PROBE_POSITION_2_NEGATIVE_VALUE_IDX   0x60BD
#define FOLLOWING_ERROR_ACTUAL_VALUE_IDX            0x60F4
#define DIGITAL_INPUTS_IDX                          0x60FD
#define DIGITAL_OUTPUTS_IDX                         0x60FE
#define TARGET_VELOCITY_IDX                         0x60FF
#define SUPPORTED_DRIVE_MODES_IDX                   0x6502
// Manufacturer Specific
#define HOME_SWITCH_IDX                             0x2001
#define OUTPUT_STATUS_IDX                           0x2002
#define TORQUE_CONSTANT_IDX                         0x2005
#define CLEAR_ALARM_IDX                             0x2006
#define Q_SEGMENT_NUMBER_IDX                        0x2007
#define STATUS_CODE_IDX                             0x200B
#define ZERO_POSITION_IDX                           0x200C
#define ALARM_CODE_IDX                              0x200F
#define POSITION_LOOP_DIFFERENTIAL_GAIN_IDX         0x2011
#define POSITION_LOOP_DIFFERENTIAL_FILTER_IDX       0x2012
#define VELOCITY_LOOP_PROPORTIONAL_GAIN_IDX         0x2013
#define VELOCITY_LOOP_INTEGRATOR_GAIN_IDX           0x2014
#define ACCELERAATION_FEEDFORWARD_GAIN_IDX          0x2015
#define PID_FILTER_IDX                              0x2016
#define DRIVE_TEMPERATURE_IDX                       0x2019
#define IN_POSITION_COUNTS_IDX                      0x201A
#define ENCODER_RESOLUTION_IDX                      0x201C
#define IN_POSITION_ERROR_RANGE_IDX                 0x201D
#define IN_POSITION_TIMING_IDX                      0x201E
#define USER_REGISTERS_IDX                          0x2100

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
// In SM synchronization mode, PP, PV, TQ, HM and Q modes are supported.
// In DC synchronization mode, CSP, CSV and HM modes are supported.

#endif
