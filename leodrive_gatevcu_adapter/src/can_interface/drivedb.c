#include "leodrive_gatevcu_adapter/can_interface/drivedb.h"

// DBC file version
#if (VER_DRIVEDB_MAJ != (0U)) || (VER_DRIVEDB_MIN != (0U))
#error The DRIVEDB dbc source files have different versions
#endif

#ifdef DRIVEDB_USE_DIAG_MONITORS
// Function prototypes to be called each time CAN frame is unpacked
// FMon function may detect RC, CRC or DLC violation
#include "drivedb-fmon.h"

#endif // DRIVEDB_USE_DIAG_MONITORS

// This macro guard for the case when you need to enable
// using diag monitors but there is no necessity in proper
// SysTick provider. For providing one you need define macro
// before this line - in dbccodeconf.h

#ifndef GetSystemTick
#define GetSystemTick() (0u)
#endif

// This macro guard is for the case when you want to build
// app with enabled optoin auto CSM, but don't yet have
// proper getframehash implementation

#ifndef GetFrameHash
#define GetFrameHash(a,b,c,d,e) (0u)
#endif

// This function performs extension of sign for the signals
// which have non-aligned to power of 2 bit's width.
// The types 'bitext_t' and 'ubitext_t' define maximal bit width which
// can be correctly handled. You need to select type which can contain
// n+1 bits where n is the largest signed signal width. For example if
// the most wide signed signal has a width of 31 bits you need to set
// bitext_t as int32_t and ubitext_t as uint32_t
// Defined these typedefs in @dbccodeconf.h or locally in 'dbcdrvname'-config.h
static bitext_t __ext_sig__(ubitext_t val, uint8_t bits)
{
  ubitext_t const m = 1u << (bits - 1);
  return (val ^ m) - m;
}

uint32_t Unpack_LongitudinalCommandsV1_drivedb(LongitudinalCommandsV1_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->set_long_accel = (int32_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 24U) | ((_d[2] & (0xFFU)) << 16U) | ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 32);
  _m->set_limit_velocity = (int32_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 24U) | ((_d[6] & (0xFFU)) << 16U) | ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 32);

#ifdef DRIVEDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < LongitudinalCommandsV1_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_LongitudinalCommandsV1_drivedb(&_m->mon1, LongitudinalCommandsV1_CANID);
#endif // DRIVEDB_USE_DIAG_MONITORS

  return LongitudinalCommandsV1_CANID;
}

#ifdef DRIVEDB_USE_CANSTRUCT

uint32_t Pack_LongitudinalCommandsV1_drivedb(LongitudinalCommandsV1_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(LongitudinalCommandsV1_DLC); cframe->Data[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->set_long_accel & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->set_long_accel >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->set_long_accel >> 16U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->set_long_accel >> 24U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->set_limit_velocity & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->set_limit_velocity >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->set_limit_velocity >> 16U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->set_limit_velocity >> 24U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) LongitudinalCommandsV1_CANID;
  cframe->DLC = (uint8_t) LongitudinalCommandsV1_DLC;
  cframe->IDE = (uint8_t) LongitudinalCommandsV1_IDE;
  return LongitudinalCommandsV1_CANID;
}

#else

uint32_t Pack_LongitudinalCommandsV1_drivedb(LongitudinalCommandsV1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(LongitudinalCommandsV1_DLC); _d[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->set_long_accel & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->set_long_accel >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->set_long_accel >> 16U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->set_long_accel >> 24U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->set_limit_velocity & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->set_limit_velocity >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->set_limit_velocity >> 16U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->set_limit_velocity >> 24U) & (0xFFU)) );

  *_len = (uint8_t) LongitudinalCommandsV1_DLC;
  *_ide = (uint8_t) LongitudinalCommandsV1_IDE;
  return LongitudinalCommandsV1_CANID;
}

#endif // DRIVEDB_USE_CANSTRUCT

uint32_t Unpack_LongitudinalCommandsV2_drivedb(LongitudinalCommandsV2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->set_gas_pedal_pos_ro = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef DRIVEDB_USE_SIGFLOAT
  _m->set_gas_pedal_pos_phys = (sigfloat_t)(DRIVEDB_set_gas_pedal_pos_ro_fromS(_m->set_gas_pedal_pos_ro));
#endif // DRIVEDB_USE_SIGFLOAT

  _m->set_brake_force_ro = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
#ifdef DRIVEDB_USE_SIGFLOAT
  _m->set_brake_force_phys = (uint32_t) DRIVEDB_set_brake_force_ro_fromS(_m->set_brake_force_ro);
#endif // DRIVEDB_USE_SIGFLOAT

#ifdef DRIVEDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < LongitudinalCommandsV2_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_LongitudinalCommandsV2_drivedb(&_m->mon1, LongitudinalCommandsV2_CANID);
#endif // DRIVEDB_USE_DIAG_MONITORS

  return LongitudinalCommandsV2_CANID;
}

#ifdef DRIVEDB_USE_CANSTRUCT

uint32_t Pack_LongitudinalCommandsV2_drivedb(LongitudinalCommandsV2_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(LongitudinalCommandsV2_DLC); cframe->Data[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

#ifdef DRIVEDB_USE_SIGFLOAT
  _m->set_gas_pedal_pos_ro = (uint16_t) DRIVEDB_set_gas_pedal_pos_ro_toS(_m->set_gas_pedal_pos_phys);
  _m->set_brake_force_ro = (uint16_t) DRIVEDB_set_brake_force_ro_toS(_m->set_brake_force_phys);
#endif // DRIVEDB_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->set_gas_pedal_pos_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->set_gas_pedal_pos_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->set_brake_force_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->set_brake_force_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) LongitudinalCommandsV2_CANID;
  cframe->DLC = (uint8_t) LongitudinalCommandsV2_DLC;
  cframe->IDE = (uint8_t) LongitudinalCommandsV2_IDE;
  return LongitudinalCommandsV2_CANID;
}

#else

uint32_t Pack_LongitudinalCommandsV2_drivedb(LongitudinalCommandsV2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(LongitudinalCommandsV2_DLC); _d[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

#ifdef DRIVEDB_USE_SIGFLOAT
  _m->set_gas_pedal_pos_ro = (uint16_t) DRIVEDB_set_gas_pedal_pos_ro_toS(_m->set_gas_pedal_pos_phys);
  _m->set_brake_force_ro = (uint16_t) DRIVEDB_set_brake_force_ro_toS(_m->set_brake_force_phys);
#endif // DRIVEDB_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->set_gas_pedal_pos_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->set_gas_pedal_pos_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->set_brake_force_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->set_brake_force_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) LongitudinalCommandsV2_DLC;
  *_ide = (uint8_t) LongitudinalCommandsV2_IDE;
  return LongitudinalCommandsV2_CANID;
}

#endif // DRIVEDB_USE_CANSTRUCT

uint32_t Unpack_VehicleCommands_drivedb(VehicleCommands_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->blinker = (uint8_t) ( (_d[0] & (0xFFU)) );
  _m->headlgiht = (uint8_t) ( (_d[1] & (0xFFU)) );
  _m->wiper = (uint8_t) ( (_d[2] & (0xFFU)) );
  _m->gear = (uint8_t) ( (_d[3] & (0xFFU)) );
  _m->mode = (uint8_t) ( (_d[4] & (0xFFU)) );
  _m->hand_brake = (uint8_t) ( (_d[5] & (0xFFU)) );
  _m->TakeoverRequest = (uint8_t) ( (_d[6] & (0xFFU)) );
  _m->Long_mode = (uint8_t) ( (_d[7] & (0xFFU)) );

#ifdef DRIVEDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VehicleCommands_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VehicleCommands_drivedb(&_m->mon1, VehicleCommands_CANID);
#endif // DRIVEDB_USE_DIAG_MONITORS

  return VehicleCommands_CANID;
}

#ifdef DRIVEDB_USE_CANSTRUCT

uint32_t Pack_VehicleCommands_drivedb(VehicleCommands_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(VehicleCommands_DLC); cframe->Data[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->blinker & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->headlgiht & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->wiper & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->gear & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->mode & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->hand_brake & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->TakeoverRequest & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->Long_mode & (0xFFU)) );

  cframe->MsgId = (uint32_t) VehicleCommands_CANID;
  cframe->DLC = (uint8_t) VehicleCommands_DLC;
  cframe->IDE = (uint8_t) VehicleCommands_IDE;
  return VehicleCommands_CANID;
}

#else

uint32_t Pack_VehicleCommands_drivedb(VehicleCommands_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(VehicleCommands_DLC); _d[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->blinker & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->headlgiht & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->wiper & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->gear & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->mode & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->hand_brake & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->TakeoverRequest & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->Long_mode & (0xFFU)) );

  *_len = (uint8_t) VehicleCommands_DLC;
  *_ide = (uint8_t) VehicleCommands_IDE;
  return VehicleCommands_CANID;
}

#endif // DRIVEDB_USE_CANSTRUCT

uint32_t Unpack_FrontWheelCommands_drivedb(FrontWheelCommands_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->set_steering_wheel_angle_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef DRIVEDB_USE_SIGFLOAT
  _m->set_steering_wheel_angle_phys = (sigfloat_t)(DRIVEDB_set_steering_wheel_angle_ro_fromS(_m->set_steering_wheel_angle_ro));
#endif // DRIVEDB_USE_SIGFLOAT

  _m->set_steering_wheel_torque = (uint8_t) ( (_d[2] & (0xFFU)) );

#ifdef DRIVEDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < FrontWheelCommands_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_FrontWheelCommands_drivedb(&_m->mon1, FrontWheelCommands_CANID);
#endif // DRIVEDB_USE_DIAG_MONITORS

  return FrontWheelCommands_CANID;
}

#ifdef DRIVEDB_USE_CANSTRUCT

uint32_t Pack_FrontWheelCommands_drivedb(FrontWheelCommands_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(FrontWheelCommands_DLC); cframe->Data[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

#ifdef DRIVEDB_USE_SIGFLOAT
  _m->set_steering_wheel_angle_ro = (int16_t) DRIVEDB_set_steering_wheel_angle_ro_toS(_m->set_steering_wheel_angle_phys);
#endif // DRIVEDB_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->set_steering_wheel_angle_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->set_steering_wheel_angle_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->set_steering_wheel_torque & (0xFFU)) );

  cframe->MsgId = (uint32_t) FrontWheelCommands_CANID;
  cframe->DLC = (uint8_t) FrontWheelCommands_DLC;
  cframe->IDE = (uint8_t) FrontWheelCommands_IDE;
  return FrontWheelCommands_CANID;
}

#else

uint32_t Pack_FrontWheelCommands_drivedb(FrontWheelCommands_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(FrontWheelCommands_DLC); _d[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

#ifdef DRIVEDB_USE_SIGFLOAT
  _m->set_steering_wheel_angle_ro = (int16_t) DRIVEDB_set_steering_wheel_angle_ro_toS(_m->set_steering_wheel_angle_phys);
#endif // DRIVEDB_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->set_steering_wheel_angle_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->set_steering_wheel_angle_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->set_steering_wheel_torque & (0xFFU)) );

  *_len = (uint8_t) FrontWheelCommands_DLC;
  *_ide = (uint8_t) FrontWheelCommands_IDE;
  return FrontWheelCommands_CANID;
}

#endif // DRIVEDB_USE_CANSTRUCT

uint32_t Unpack_VehicleDynamicsInfo_drivedb(VehicleDynamicsInfo_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->linear_vehicle_velocity = (int32_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 24U) | ((_d[2] & (0xFFU)) << 16U) | ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 32);
  _m->front_wheel_angle = (int32_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 24U) | ((_d[6] & (0xFFU)) << 16U) | ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 32);

#ifdef DRIVEDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VehicleDynamicsInfo_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VehicleDynamicsInfo_drivedb(&_m->mon1, VehicleDynamicsInfo_CANID);
#endif // DRIVEDB_USE_DIAG_MONITORS

  return VehicleDynamicsInfo_CANID;
}

#ifdef DRIVEDB_USE_CANSTRUCT

uint32_t Pack_VehicleDynamicsInfo_drivedb(VehicleDynamicsInfo_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(VehicleDynamicsInfo_DLC); cframe->Data[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->linear_vehicle_velocity & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->linear_vehicle_velocity >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->linear_vehicle_velocity >> 16U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->linear_vehicle_velocity >> 24U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->front_wheel_angle & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->front_wheel_angle >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->front_wheel_angle >> 16U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->front_wheel_angle >> 24U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) VehicleDynamicsInfo_CANID;
  cframe->DLC = (uint8_t) VehicleDynamicsInfo_DLC;
  cframe->IDE = (uint8_t) VehicleDynamicsInfo_IDE;
  return VehicleDynamicsInfo_CANID;
}

#else

uint32_t Pack_VehicleDynamicsInfo_drivedb(VehicleDynamicsInfo_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(VehicleDynamicsInfo_DLC); _d[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->linear_vehicle_velocity & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->linear_vehicle_velocity >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->linear_vehicle_velocity >> 16U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->linear_vehicle_velocity >> 24U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->front_wheel_angle & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->front_wheel_angle >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->front_wheel_angle >> 16U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->front_wheel_angle >> 24U) & (0xFFU)) );

  *_len = (uint8_t) VehicleDynamicsInfo_DLC;
  *_ide = (uint8_t) VehicleDynamicsInfo_IDE;
  return VehicleDynamicsInfo_CANID;
}

#endif // DRIVEDB_USE_CANSTRUCT

uint32_t Unpack_VehicleSignalStatus_drivedb(VehicleSignalStatus_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->fuel = (uint8_t) ( (_d[0] & (0xFFU)) );
  _m->blinker = (uint8_t) ( (_d[1] & (0xFFU)) );
  _m->headlight = (uint8_t) ( (_d[2] & (0xFFU)) );
  _m->wiper = (uint8_t) ( (_d[3] & (0xFFU)) );
  _m->gear = (uint8_t) ( (_d[4] & (0xFFU)) );
  _m->mode = (uint8_t) ( (_d[5] & (0xFFU)) );
  _m->hand_brake = (uint8_t) ( (_d[6] & (0xFFU)) );
  _m->horn = (uint8_t) ( (_d[7] & (0xFFU)) );

#ifdef DRIVEDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VehicleSignalStatus_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VehicleSignalStatus_drivedb(&_m->mon1, VehicleSignalStatus_CANID);
#endif // DRIVEDB_USE_DIAG_MONITORS

  return VehicleSignalStatus_CANID;
}

#ifdef DRIVEDB_USE_CANSTRUCT

uint32_t Pack_VehicleSignalStatus_drivedb(VehicleSignalStatus_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(VehicleSignalStatus_DLC); cframe->Data[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->fuel & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->blinker & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->headlight & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->wiper & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->gear & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->mode & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->hand_brake & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->horn & (0xFFU)) );

  cframe->MsgId = (uint32_t) VehicleSignalStatus_CANID;
  cframe->DLC = (uint8_t) VehicleSignalStatus_DLC;
  cframe->IDE = (uint8_t) VehicleSignalStatus_IDE;
  return VehicleSignalStatus_CANID;
}

#else

uint32_t Pack_VehicleSignalStatus_drivedb(VehicleSignalStatus_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(VehicleSignalStatus_DLC); _d[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->fuel & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->blinker & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->headlight & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->wiper & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->gear & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->mode & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->hand_brake & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->horn & (0xFFU)) );

  *_len = (uint8_t) VehicleSignalStatus_DLC;
  *_ide = (uint8_t) VehicleSignalStatus_IDE;
  return VehicleSignalStatus_CANID;
}

#endif // DRIVEDB_USE_CANSTRUCT

uint32_t Unpack_MotionInfo_drivedb(MotionInfo_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->Steering_intervention = (uint8_t) ( (_d[0] & (0x01U)) );
  _m->Brake_intervention = (uint8_t) ( ((_d[0] >> 1U) & (0x01U)) );
  _m->Acc_Pedal_intervention = (uint8_t) ( ((_d[0] >> 2U) & (0x01U)) );
  _m->ready = (uint8_t) ( (_d[1] & (0xFFU)) );
  _m->motion_allow = (uint8_t) ( (_d[2] & (0xFFU)) );
  _m->throttle = (uint8_t) ( (_d[3] & (0xFFU)) );
  _m->brake = (uint8_t) ( (_d[4] & (0xFFU)) );
  _m->front_steer = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );

#ifdef DRIVEDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < MotionInfo_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_MotionInfo_drivedb(&_m->mon1, MotionInfo_CANID);
#endif // DRIVEDB_USE_DIAG_MONITORS

  return MotionInfo_CANID;
}

#ifdef DRIVEDB_USE_CANSTRUCT

uint32_t Pack_MotionInfo_drivedb(MotionInfo_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(MotionInfo_DLC); cframe->Data[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->Steering_intervention & (0x01U)) | ((_m->Brake_intervention & (0x01U)) << 1U) | ((_m->Acc_Pedal_intervention & (0x01U)) << 2U) );
  cframe->Data[1] |= (uint8_t) ( (_m->ready & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->motion_allow & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->throttle & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->brake & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->front_steer & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->front_steer >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) MotionInfo_CANID;
  cframe->DLC = (uint8_t) MotionInfo_DLC;
  cframe->IDE = (uint8_t) MotionInfo_IDE;
  return MotionInfo_CANID;
}

#else

uint32_t Pack_MotionInfo_drivedb(MotionInfo_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(MotionInfo_DLC); _d[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->Steering_intervention & (0x01U)) | ((_m->Brake_intervention & (0x01U)) << 1U) | ((_m->Acc_Pedal_intervention & (0x01U)) << 2U) );
  _d[1] |= (uint8_t) ( (_m->ready & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->motion_allow & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->throttle & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->brake & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->front_steer & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->front_steer >> 8U) & (0xFFU)) );

  *_len = (uint8_t) MotionInfo_DLC;
  *_ide = (uint8_t) MotionInfo_IDE;
  return MotionInfo_CANID;
}

#endif // DRIVEDB_USE_CANSTRUCT

uint32_t Unpack_motor_info_drivedb(motor_info_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->motor_temp = (uint8_t) ( (_d[0] & (0xFFU)) );
  _m->motor_rpm = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
  _m->kl75 = (uint8_t) ( (_d[3] & (0x01U)) );

#ifdef DRIVEDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < motor_info_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_motor_info_drivedb(&_m->mon1, motor_info_CANID);
#endif // DRIVEDB_USE_DIAG_MONITORS

  return motor_info_CANID;
}

#ifdef DRIVEDB_USE_CANSTRUCT

uint32_t Pack_motor_info_drivedb(motor_info_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(motor_info_DLC); cframe->Data[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->motor_temp & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->motor_rpm & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->motor_rpm >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->kl75 & (0x01U)) );

  cframe->MsgId = (uint32_t) motor_info_CANID;
  cframe->DLC = (uint8_t) motor_info_DLC;
  cframe->IDE = (uint8_t) motor_info_IDE;
  return motor_info_CANID;
}

#else

uint32_t Pack_motor_info_drivedb(motor_info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(motor_info_DLC); _d[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->motor_temp & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->motor_rpm & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->motor_rpm >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->kl75 & (0x01U)) );

  *_len = (uint8_t) motor_info_DLC;
  *_ide = (uint8_t) motor_info_IDE;
  return motor_info_CANID;
}

#endif // DRIVEDB_USE_CANSTRUCT

uint32_t Unpack_VehicleErrors_drivedb(VehicleErrors_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->isMotorRunning = (uint8_t) ( (_d[0] & (0x01U)) );
  _m->Kl75 = (uint8_t) ( ((_d[0] >> 1U) & (0x01U)) );
  _m->PDS_HeartbeatErr = (uint8_t) ( (_d[1] & (0x01U)) );
  _m->PDS_BusErr = (uint8_t) ( ((_d[1] >> 1U) & (0x01U)) );
  _m->By_wirePowerErr = (uint8_t) ( ((_d[1] >> 2U) & (0x01U)) );
  _m->EPASPowerErr = (uint8_t) ( ((_d[1] >> 3U) & (0x01U)) );
  _m->BrakePowerErr = (uint8_t) ( ((_d[1] >> 4U) & (0x01U)) );
  _m->Throttle_ECU_HeartbeatErr = (uint8_t) ( ((_d[1] >> 5U) & (0x01U)) );
  _m->G29_HeartbeatErr = (uint8_t) ( ((_d[1] >> 6U) & (0x01U)) );
  _m->EPAS_systemErr = (uint8_t) ( ((_d[1] >> 7U) & (0x01U)) );
  _m->EPAS_HeartbeatErr = (uint8_t) ( (_d[2] & (0x01U)) );
  _m->Brake_SystemErr = (uint8_t) ( ((_d[2] >> 1U) & (0x01U)) );
  _m->Brake_HeartbeatErr = (uint8_t) ( ((_d[2] >> 2U) & (0x01U)) );
  _m->PC_HeartbeatErr = (uint8_t) ( ((_d[2] >> 3U) & (0x01U)) );

#ifdef DRIVEDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VehicleErrors_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VehicleErrors_drivedb(&_m->mon1, VehicleErrors_CANID);
#endif // DRIVEDB_USE_DIAG_MONITORS

  return VehicleErrors_CANID;
}

#ifdef DRIVEDB_USE_CANSTRUCT

uint32_t Pack_VehicleErrors_drivedb(VehicleErrors_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(VehicleErrors_DLC); cframe->Data[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->isMotorRunning & (0x01U)) | ((_m->Kl75 & (0x01U)) << 1U) );
  cframe->Data[1] |= (uint8_t) ( (_m->PDS_HeartbeatErr & (0x01U)) | ((_m->PDS_BusErr & (0x01U)) << 1U) | ((_m->By_wirePowerErr & (0x01U)) << 2U) | ((_m->EPASPowerErr & (0x01U)) << 3U) | ((_m->BrakePowerErr & (0x01U)) << 4U) | ((_m->Throttle_ECU_HeartbeatErr & (0x01U)) << 5U) | ((_m->G29_HeartbeatErr & (0x01U)) << 6U) | ((_m->EPAS_systemErr & (0x01U)) << 7U) );
  cframe->Data[2] |= (uint8_t) ( (_m->EPAS_HeartbeatErr & (0x01U)) | ((_m->Brake_SystemErr & (0x01U)) << 1U) | ((_m->Brake_HeartbeatErr & (0x01U)) << 2U) | ((_m->PC_HeartbeatErr & (0x01U)) << 3U) );

  cframe->MsgId = (uint32_t) VehicleErrors_CANID;
  cframe->DLC = (uint8_t) VehicleErrors_DLC;
  cframe->IDE = (uint8_t) VehicleErrors_IDE;
  return VehicleErrors_CANID;
}

#else

uint32_t Pack_VehicleErrors_drivedb(VehicleErrors_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(VehicleErrors_DLC); _d[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->isMotorRunning & (0x01U)) | ((_m->Kl75 & (0x01U)) << 1U) );
  _d[1] |= (uint8_t) ( (_m->PDS_HeartbeatErr & (0x01U)) | ((_m->PDS_BusErr & (0x01U)) << 1U) | ((_m->By_wirePowerErr & (0x01U)) << 2U) | ((_m->EPASPowerErr & (0x01U)) << 3U) | ((_m->BrakePowerErr & (0x01U)) << 4U) | ((_m->Throttle_ECU_HeartbeatErr & (0x01U)) << 5U) | ((_m->G29_HeartbeatErr & (0x01U)) << 6U) | ((_m->EPAS_systemErr & (0x01U)) << 7U) );
  _d[2] |= (uint8_t) ( (_m->EPAS_HeartbeatErr & (0x01U)) | ((_m->Brake_SystemErr & (0x01U)) << 1U) | ((_m->Brake_HeartbeatErr & (0x01U)) << 2U) | ((_m->PC_HeartbeatErr & (0x01U)) << 3U) );

  *_len = (uint8_t) VehicleErrors_DLC;
  *_ide = (uint8_t) VehicleErrors_IDE;
  return VehicleErrors_CANID;
}

#endif // DRIVEDB_USE_CANSTRUCT

uint32_t Unpack_Drive_Feedback_drivedb(Drive_Feedback_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->Throttle_Pos_ro = (uint8_t) ( (_d[0] & (0xFFU)) );
#ifdef DRIVEDB_USE_SIGFLOAT
  _m->Throttle_Pos_phys = (sigfloat_t)(DRIVEDB_Throttle_Pos_ro_fromS(_m->Throttle_Pos_ro));
#endif // DRIVEDB_USE_SIGFLOAT

  _m->Brake_Pos = (uint8_t) ( (_d[1] & (0xFFU)) );

#ifdef DRIVEDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < Drive_Feedback_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_Drive_Feedback_drivedb(&_m->mon1, Drive_Feedback_CANID);
#endif // DRIVEDB_USE_DIAG_MONITORS

  return Drive_Feedback_CANID;
}

#ifdef DRIVEDB_USE_CANSTRUCT

uint32_t Pack_Drive_Feedback_drivedb(Drive_Feedback_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(Drive_Feedback_DLC); cframe->Data[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

#ifdef DRIVEDB_USE_SIGFLOAT
  _m->Throttle_Pos_ro = (uint8_t) DRIVEDB_Throttle_Pos_ro_toS(_m->Throttle_Pos_phys);
#endif // DRIVEDB_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->Throttle_Pos_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->Brake_Pos & (0xFFU)) );

  cframe->MsgId = (uint32_t) Drive_Feedback_CANID;
  cframe->DLC = (uint8_t) Drive_Feedback_DLC;
  cframe->IDE = (uint8_t) Drive_Feedback_IDE;
  return Drive_Feedback_CANID;
}

#else

uint32_t Pack_Drive_Feedback_drivedb(Drive_Feedback_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < DRIVEDB_VALIDATE_DLC(Drive_Feedback_DLC); _d[i++] = DRIVEDB_INITIAL_BYTE_VALUE);

#ifdef DRIVEDB_USE_SIGFLOAT
  _m->Throttle_Pos_ro = (uint8_t) DRIVEDB_Throttle_Pos_ro_toS(_m->Throttle_Pos_phys);
#endif // DRIVEDB_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->Throttle_Pos_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->Brake_Pos & (0xFFU)) );

  *_len = (uint8_t) Drive_Feedback_DLC;
  *_ide = (uint8_t) Drive_Feedback_IDE;
  return Drive_Feedback_CANID;
}

#endif // DRIVEDB_USE_CANSTRUCT

