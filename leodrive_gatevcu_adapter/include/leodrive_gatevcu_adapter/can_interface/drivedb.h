#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// DBC file version
#define VER_DRIVEDB_MAJ (0U)
#define VER_DRIVEDB_MIN (0U)

// include current dbc-driver compilation config
#include "drivedb-config.h"

#ifdef DRIVEDB_USE_DIAG_MONITORS
// This file must define:
// base monitor struct
#include "canmonitorutil.h"

#endif // DRIVEDB_USE_DIAG_MONITORS


// DLC maximum value which is used as the limit for frame's data buffer size.
// Client can set its own value (not sure why) in driver-config
// or can test it on some limit specified by application
// e.g.: static_assert(TESTDB_MAX_DLC_VALUE <= APPLICATION_FRAME_DATA_SIZE, "Max DLC value in the driver is too big")
#ifndef DRIVEDB_MAX_DLC_VALUE
// The value which was found out by generator (real max value)
#define DRIVEDB_MAX_DLC_VALUE 8U
#endif

// The limit is used for setting frame's data bytes
#define DRIVEDB_VALIDATE_DLC(msgDlc) (((msgDlc) <= (DRIVEDB_MAX_DLC_VALUE)) ? (msgDlc) : (DRIVEDB_MAX_DLC_VALUE))

// Initial byte value to be filles in data bytes of the frame before pack signals
// User can define its own custom value in driver-config file
#ifndef DRIVEDB_INITIAL_BYTE_VALUE
#define DRIVEDB_INITIAL_BYTE_VALUE 0U
#endif


// def @LongitudinalCommandsV1 CAN Message (1024 0x400)
#define LongitudinalCommandsV1_IDE (0U)
#define LongitudinalCommandsV1_DLC (8U)
#define LongitudinalCommandsV1_CANID (0x400U)

typedef struct
{
#ifdef DRIVEDB_USE_BITS_SIGNAL

  // -
  int32_t set_long_accel;                    //  [-] Bits=32 Unit:'m/s^2'

  // -
  int32_t set_limit_velocity;                //  [-] Bits=32 Unit:'m/s'

#else

  // -
  int32_t set_long_accel;                    //  [-] Bits=32 Unit:'m/s^2'

  // -
  int32_t set_limit_velocity;                //  [-] Bits=32 Unit:'m/s'

#endif // DRIVEDB_USE_BITS_SIGNAL

#ifdef DRIVEDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // DRIVEDB_USE_DIAG_MONITORS

} LongitudinalCommandsV1_t;

// def @LongitudinalCommandsV2 CAN Message (1025 0x401)
#define LongitudinalCommandsV2_IDE (0U)
#define LongitudinalCommandsV2_DLC (8U)
#define LongitudinalCommandsV2_CANID (0x401U)
// signal: @set_gas_pedal_pos_ro
#define DRIVEDB_set_gas_pedal_pos_ro_CovFactor (0.01)
#define DRIVEDB_set_gas_pedal_pos_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.01)) )
#define DRIVEDB_set_gas_pedal_pos_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @set_brake_force_ro
#define DRIVEDB_set_brake_force_ro_CovFactor (8)
#define DRIVEDB_set_brake_force_ro_toS(x) ( (uint16_t) ((x) / (8)) )
#define DRIVEDB_set_brake_force_ro_fromS(x) ( ((x) * (8)) )

typedef struct
{
#ifdef DRIVEDB_USE_BITS_SIGNAL

  uint16_t set_gas_pedal_pos_ro;             //      Bits=16 Factor= 0.01            Unit:'-'

#ifdef DRIVEDB_USE_SIGFLOAT
  sigfloat_t set_gas_pedal_pos_phys;
#endif // DRIVEDB_USE_SIGFLOAT

  uint16_t set_brake_force_ro;               //      Bits=16 Factor= 8               Unit:'Newton'

#ifdef DRIVEDB_USE_SIGFLOAT
  uint32_t set_brake_force_phys;
#endif // DRIVEDB_USE_SIGFLOAT

#else

  uint16_t set_gas_pedal_pos_ro;             //      Bits=16 Factor= 0.01            Unit:'-'

#ifdef DRIVEDB_USE_SIGFLOAT
  sigfloat_t set_gas_pedal_pos_phys;
#endif // DRIVEDB_USE_SIGFLOAT

  uint16_t set_brake_force_ro;               //      Bits=16 Factor= 8               Unit:'Newton'

#ifdef DRIVEDB_USE_SIGFLOAT
  uint32_t set_brake_force_phys;
#endif // DRIVEDB_USE_SIGFLOAT

#endif // DRIVEDB_USE_BITS_SIGNAL

#ifdef DRIVEDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // DRIVEDB_USE_DIAG_MONITORS

} LongitudinalCommandsV2_t;

// def @VehicleCommands CAN Message (1026 0x402)
#define VehicleCommands_IDE (0U)
#define VehicleCommands_DLC (8U)
#define VehicleCommands_CANID (0x402U)

typedef struct
{
#ifdef DRIVEDB_USE_BITS_SIGNAL

  // -
  uint8_t blinker;                           //      Bits= 8 Unit:'-'

  // -
  uint8_t headlgiht;                         //      Bits= 8 Unit:'-'

  // Currently we have no control
  uint8_t wiper;                             //      Bits= 8 Unit:'-'

  // D-N-R-P
  uint8_t gear;                              //      Bits= 8 Unit:'-'

  // 0-> manual 1-> autonomous
  uint8_t mode;                              //      Bits= 8 Unit:'-'

  // Currently we have no control
  uint8_t hand_brake;                        //      Bits= 8 Unit:'-'

  // Emergency
  uint8_t TakeoverRequest;                   //      Bits= 8 Unit:'-'

  // 0 -> Acceleration command, 1-> Throttle Brake command
  uint8_t Long_mode;                         //      Bits= 8 Unit:'-'

#else

  // -
  uint8_t blinker;                           //      Bits= 8 Unit:'-'

  // -
  uint8_t headlgiht;                         //      Bits= 8 Unit:'-'

  // Currently we have no control
  uint8_t wiper;                             //      Bits= 8 Unit:'-'

  // D-N-R-P
  uint8_t gear;                              //      Bits= 8 Unit:'-'

  // 0-> manual 1-> autonomous
  uint8_t mode;                              //      Bits= 8 Unit:'-'

  // Currently we have no control
  uint8_t hand_brake;                        //      Bits= 8 Unit:'-'

  // Emergency
  uint8_t TakeoverRequest;                   //      Bits= 8 Unit:'-'

  // 0 -> Acceleration command, 1-> Throttle Brake command
  uint8_t Long_mode;                         //      Bits= 8 Unit:'-'

#endif // DRIVEDB_USE_BITS_SIGNAL

#ifdef DRIVEDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // DRIVEDB_USE_DIAG_MONITORS

} VehicleCommands_t;

// def @FrontWheelCommands CAN Message (1027 0x403)
#define FrontWheelCommands_IDE (0U)
#define FrontWheelCommands_DLC (8U)
#define FrontWheelCommands_CANID (0x403U)
// signal: @set_steering_wheel_angle_ro
#define DRIVEDB_set_steering_wheel_angle_ro_CovFactor (0.02)
#define DRIVEDB_set_steering_wheel_angle_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.02)) )
#define DRIVEDB_set_steering_wheel_angle_ro_fromS(x) ( (((x) * (0.02)) + (0.0)) )

typedef struct
{
#ifdef DRIVEDB_USE_BITS_SIGNAL

  int16_t set_steering_wheel_angle_ro;       //  [-] Bits=16 Factor= 0.02            Unit:'degree'

#ifdef DRIVEDB_USE_SIGFLOAT
  sigfloat_t set_steering_wheel_angle_phys;
#endif // DRIVEDB_USE_SIGFLOAT

  // -
  uint8_t set_steering_wheel_torque;         //      Bits= 8 Unit:'-'

#else

  int16_t set_steering_wheel_angle_ro;       //  [-] Bits=16 Factor= 0.02            Unit:'degree'

#ifdef DRIVEDB_USE_SIGFLOAT
  sigfloat_t set_steering_wheel_angle_phys;
#endif // DRIVEDB_USE_SIGFLOAT

  // -
  uint8_t set_steering_wheel_torque;         //      Bits= 8 Unit:'-'

#endif // DRIVEDB_USE_BITS_SIGNAL

#ifdef DRIVEDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // DRIVEDB_USE_DIAG_MONITORS

} FrontWheelCommands_t;

// def @VehicleDynamicsInfo CAN Message (1041 0x411)
#define VehicleDynamicsInfo_IDE (0U)
#define VehicleDynamicsInfo_DLC (8U)
#define VehicleDynamicsInfo_CANID (0x411U)

typedef struct
{
#ifdef DRIVEDB_USE_BITS_SIGNAL

  // -
  int32_t linear_vehicle_velocity;           //  [-] Bits=32 Unit:'m/s'

  // -
  int32_t front_wheel_angle;                 //  [-] Bits=32 Unit:'rad'

#else

  // -
  int32_t linear_vehicle_velocity;           //  [-] Bits=32 Unit:'m/s'

  // -
  int32_t front_wheel_angle;                 //  [-] Bits=32 Unit:'rad'

#endif // DRIVEDB_USE_BITS_SIGNAL

#ifdef DRIVEDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // DRIVEDB_USE_DIAG_MONITORS

} VehicleDynamicsInfo_t;

// def @VehicleSignalStatus CAN Message (1042 0x412)
#define VehicleSignalStatus_IDE (0U)
#define VehicleSignalStatus_DLC (8U)
#define VehicleSignalStatus_CANID (0x412U)

typedef struct
{
#ifdef DRIVEDB_USE_BITS_SIGNAL

  // -
  uint8_t fuel;                              //      Bits= 8 Unit:'liter'

  // -
  uint8_t blinker;                           //      Bits= 8 Unit:'-'

  // -
  uint8_t headlight;                         //      Bits= 8 Unit:'-'

  // -
  uint8_t wiper;                             //      Bits= 8 Unit:'-'

  // -
  uint8_t gear;                              //      Bits= 8 Unit:'-'

  // -
  uint8_t mode;                              //      Bits= 8 Unit:'-'

  // -
  uint8_t hand_brake;                        //      Bits= 8 Unit:'-'

  // -
  uint8_t horn;                              //      Bits= 8 Unit:'-'

#else

  // -
  uint8_t fuel;                              //      Bits= 8 Unit:'liter'

  // -
  uint8_t blinker;                           //      Bits= 8 Unit:'-'

  // -
  uint8_t headlight;                         //      Bits= 8 Unit:'-'

  // -
  uint8_t wiper;                             //      Bits= 8 Unit:'-'

  // -
  uint8_t gear;                              //      Bits= 8 Unit:'-'

  // -
  uint8_t mode;                              //      Bits= 8 Unit:'-'

  // -
  uint8_t hand_brake;                        //      Bits= 8 Unit:'-'

  // -
  uint8_t horn;                              //      Bits= 8 Unit:'-'

#endif // DRIVEDB_USE_BITS_SIGNAL

#ifdef DRIVEDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // DRIVEDB_USE_DIAG_MONITORS

} VehicleSignalStatus_t;

// def @MotionInfo CAN Message (1043 0x413)
#define MotionInfo_IDE (0U)
#define MotionInfo_DLC (8U)
#define MotionInfo_CANID (0x413U)

typedef struct
{
#ifdef DRIVEDB_USE_BITS_SIGNAL

  // -
  uint8_t Steering_intervention : 1;         //      Bits= 1 Unit:'-'

  // -
  uint8_t Brake_intervention : 1;            //      Bits= 1 Unit:'-'

  // -
  uint8_t Acc_Pedal_intervention : 1;        //      Bits= 1 Unit:'-'

  // -
  uint8_t ready;                             //      Bits= 8 Unit:'-'

  // -
  uint8_t motion_allow;                      //      Bits= 8 Unit:'-'

  // -
  uint8_t throttle;                          //      Bits= 8 Unit:'-'

  // -
  uint8_t brake;                             //      Bits= 8 Unit:'-'

  // -
  uint16_t front_steer;                      //      Bits=16 Unit:'-'

#else

  // -
  uint8_t Steering_intervention;             //      Bits= 1 Unit:'-'

  // -
  uint8_t Brake_intervention;                //      Bits= 1 Unit:'-'

  // -
  uint8_t Acc_Pedal_intervention;            //      Bits= 1 Unit:'-'

  // -
  uint8_t ready;                             //      Bits= 8 Unit:'-'

  // -
  uint8_t motion_allow;                      //      Bits= 8 Unit:'-'

  // -
  uint8_t throttle;                          //      Bits= 8 Unit:'-'

  // -
  uint8_t brake;                             //      Bits= 8 Unit:'-'

  // -
  uint16_t front_steer;                      //      Bits=16 Unit:'-'

#endif // DRIVEDB_USE_BITS_SIGNAL

#ifdef DRIVEDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // DRIVEDB_USE_DIAG_MONITORS

} MotionInfo_t;

// def @motor_info CAN Message (1044 0x414)
#define motor_info_IDE (0U)
#define motor_info_DLC (8U)
#define motor_info_CANID (0x414U)

typedef struct
{
#ifdef DRIVEDB_USE_BITS_SIGNAL

  // -
  uint8_t motor_temp;                        //      Bits= 8 Unit:'-'

  // -
  uint16_t motor_rpm;                        //      Bits=16 Unit:'-'

  // -
  uint8_t kl75 : 1;                          //      Bits= 1 Unit:'-'

#else

  // -
  uint8_t motor_temp;                        //      Bits= 8 Unit:'-'

  // -
  uint16_t motor_rpm;                        //      Bits=16 Unit:'-'

  // -
  uint8_t kl75;                              //      Bits= 1 Unit:'-'

#endif // DRIVEDB_USE_BITS_SIGNAL

#ifdef DRIVEDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // DRIVEDB_USE_DIAG_MONITORS

} motor_info_t;

// def @VehicleErrors CAN Message (1045 0x415)
#define VehicleErrors_IDE (0U)
#define VehicleErrors_DLC (8U)
#define VehicleErrors_CANID (0x415U)

typedef struct
{
#ifdef DRIVEDB_USE_BITS_SIGNAL

  // -
  uint8_t isMotorRunning : 1;                //      Bits= 1 Unit:'-'

  // -
  uint8_t Kl75 : 1;                          //      Bits= 1 Unit:'-'

  // -
  uint8_t PDS_HeartbeatErr : 1;              //      Bits= 1 Unit:'-'

  // -
  uint8_t PDS_BusErr : 1;                    //      Bits= 1 Unit:'-'

  // -
  uint8_t By_wirePowerErr : 1;               //      Bits= 1 Unit:'-'

  // -
  uint8_t EPASPowerErr : 1;                  //      Bits= 1 Unit:'-'

  // -
  uint8_t BrakePowerErr : 1;                 //      Bits= 1 Unit:'-'

  // -
  uint8_t Throttle_ECU_HeartbeatErr : 1;     //      Bits= 1 Unit:'-'

  // -
  uint8_t G29_HeartbeatErr : 1;              //      Bits= 1 Unit:'-'

  // -
  uint8_t EPAS_systemErr : 1;                //      Bits= 1 Unit:'-'

  // -
  uint8_t EPAS_HeartbeatErr : 1;             //      Bits= 1 Unit:'-'

  // -
  uint8_t Brake_SystemErr : 1;               //      Bits= 1 Unit:'-'

  // -
  uint8_t Brake_HeartbeatErr : 1;            //      Bits= 1 Unit:'-'

  // -
  uint8_t PC_HeartbeatErr : 1;               //      Bits= 1 Unit:'-'

#else

  // -
  uint8_t isMotorRunning;                    //      Bits= 1 Unit:'-'

  // -
  uint8_t Kl75;                              //      Bits= 1 Unit:'-'

  // -
  uint8_t PDS_HeartbeatErr;                  //      Bits= 1 Unit:'-'

  // -
  uint8_t PDS_BusErr;                        //      Bits= 1 Unit:'-'

  // -
  uint8_t By_wirePowerErr;                   //      Bits= 1 Unit:'-'

  // -
  uint8_t EPASPowerErr;                      //      Bits= 1 Unit:'-'

  // -
  uint8_t BrakePowerErr;                     //      Bits= 1 Unit:'-'

  // -
  uint8_t Throttle_ECU_HeartbeatErr;         //      Bits= 1 Unit:'-'

  // -
  uint8_t G29_HeartbeatErr;                  //      Bits= 1 Unit:'-'

  // -
  uint8_t EPAS_systemErr;                    //      Bits= 1 Unit:'-'

  // -
  uint8_t EPAS_HeartbeatErr;                 //      Bits= 1 Unit:'-'

  // -
  uint8_t Brake_SystemErr;                   //      Bits= 1 Unit:'-'

  // -
  uint8_t Brake_HeartbeatErr;                //      Bits= 1 Unit:'-'

  // -
  uint8_t PC_HeartbeatErr;                   //      Bits= 1 Unit:'-'

#endif // DRIVEDB_USE_BITS_SIGNAL

#ifdef DRIVEDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // DRIVEDB_USE_DIAG_MONITORS

} VehicleErrors_t;

// def @Drive_Feedback CAN Message (1046 0x416)
#define Drive_Feedback_IDE (0U)
#define Drive_Feedback_DLC (8U)
#define Drive_Feedback_CANID (0x416U)
// signal: @Throttle_Pos_ro
#define DRIVEDB_Throttle_Pos_ro_CovFactor (0.4)
#define DRIVEDB_Throttle_Pos_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.4)) )
#define DRIVEDB_Throttle_Pos_ro_fromS(x) ( (((x) * (0.4)) + (0.0)) )

typedef struct
{
#ifdef DRIVEDB_USE_BITS_SIGNAL

  uint8_t Throttle_Pos_ro;                   //      Bits= 8 Factor= 0.4             Unit:'%'

#ifdef DRIVEDB_USE_SIGFLOAT
  sigfloat_t Throttle_Pos_phys;
#endif // DRIVEDB_USE_SIGFLOAT

  // Brake position
  uint8_t Brake_Pos;                         //      Bits= 8 Unit:'%'

#else

  uint8_t Throttle_Pos_ro;                   //      Bits= 8 Factor= 0.4             Unit:'%'

#ifdef DRIVEDB_USE_SIGFLOAT
  sigfloat_t Throttle_Pos_phys;
#endif // DRIVEDB_USE_SIGFLOAT

  // Brake position
  uint8_t Brake_Pos;                         //      Bits= 8 Unit:'%'

#endif // DRIVEDB_USE_BITS_SIGNAL

#ifdef DRIVEDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // DRIVEDB_USE_DIAG_MONITORS

} Drive_Feedback_t;

// Function signatures

uint32_t Unpack_LongitudinalCommandsV1_drivedb(LongitudinalCommandsV1_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef DRIVEDB_USE_CANSTRUCT
uint32_t Pack_LongitudinalCommandsV1_drivedb(LongitudinalCommandsV1_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_LongitudinalCommandsV1_drivedb(LongitudinalCommandsV1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // DRIVEDB_USE_CANSTRUCT

uint32_t Unpack_LongitudinalCommandsV2_drivedb(LongitudinalCommandsV2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef DRIVEDB_USE_CANSTRUCT
uint32_t Pack_LongitudinalCommandsV2_drivedb(LongitudinalCommandsV2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_LongitudinalCommandsV2_drivedb(LongitudinalCommandsV2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // DRIVEDB_USE_CANSTRUCT

uint32_t Unpack_VehicleCommands_drivedb(VehicleCommands_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef DRIVEDB_USE_CANSTRUCT
uint32_t Pack_VehicleCommands_drivedb(VehicleCommands_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VehicleCommands_drivedb(VehicleCommands_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // DRIVEDB_USE_CANSTRUCT

uint32_t Unpack_FrontWheelCommands_drivedb(FrontWheelCommands_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef DRIVEDB_USE_CANSTRUCT
uint32_t Pack_FrontWheelCommands_drivedb(FrontWheelCommands_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_FrontWheelCommands_drivedb(FrontWheelCommands_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // DRIVEDB_USE_CANSTRUCT

uint32_t Unpack_VehicleDynamicsInfo_drivedb(VehicleDynamicsInfo_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef DRIVEDB_USE_CANSTRUCT
uint32_t Pack_VehicleDynamicsInfo_drivedb(VehicleDynamicsInfo_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VehicleDynamicsInfo_drivedb(VehicleDynamicsInfo_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // DRIVEDB_USE_CANSTRUCT

uint32_t Unpack_VehicleSignalStatus_drivedb(VehicleSignalStatus_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef DRIVEDB_USE_CANSTRUCT
uint32_t Pack_VehicleSignalStatus_drivedb(VehicleSignalStatus_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VehicleSignalStatus_drivedb(VehicleSignalStatus_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // DRIVEDB_USE_CANSTRUCT

uint32_t Unpack_MotionInfo_drivedb(MotionInfo_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef DRIVEDB_USE_CANSTRUCT
uint32_t Pack_MotionInfo_drivedb(MotionInfo_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MotionInfo_drivedb(MotionInfo_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // DRIVEDB_USE_CANSTRUCT

uint32_t Unpack_motor_info_drivedb(motor_info_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef DRIVEDB_USE_CANSTRUCT
uint32_t Pack_motor_info_drivedb(motor_info_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_motor_info_drivedb(motor_info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // DRIVEDB_USE_CANSTRUCT

uint32_t Unpack_VehicleErrors_drivedb(VehicleErrors_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef DRIVEDB_USE_CANSTRUCT
uint32_t Pack_VehicleErrors_drivedb(VehicleErrors_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VehicleErrors_drivedb(VehicleErrors_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // DRIVEDB_USE_CANSTRUCT

uint32_t Unpack_Drive_Feedback_drivedb(Drive_Feedback_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef DRIVEDB_USE_CANSTRUCT
uint32_t Pack_Drive_Feedback_drivedb(Drive_Feedback_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_Drive_Feedback_drivedb(Drive_Feedback_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // DRIVEDB_USE_CANSTRUCT

#ifdef __cplusplus
}
#endif
