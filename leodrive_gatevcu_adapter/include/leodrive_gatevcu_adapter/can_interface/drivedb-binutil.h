#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "dbccodeconf.h"
#include "drivedb.h"

typedef struct
{
  LongitudinalCommandsV1_t LongitudinalCommandsV1;
  LongitudinalCommandsV2_t LongitudinalCommandsV2;
  VehicleCommands_t VehicleCommands;
  FrontWheelCommands_t FrontWheelCommands;
  VehicleDynamicsInfo_t VehicleDynamicsInfo;
  VehicleSignalStatus_t VehicleSignalStatus;
  MotionInfo_t MotionInfo;
  motor_info_t motor_info;
  VehicleErrors_t VehicleErrors;
  Drive_Feedback_t Drive_Feedback;
} drivedb_rx_t;

// There is no any TX mapped massage.

uint32_t drivedb_Receive(drivedb_rx_t* m, const uint8_t* d, uint32_t msgid, uint8_t dlc);

#ifdef __DEF_DRIVEDB__

extern drivedb_rx_t drivedb_rx;

#endif // __DEF_DRIVEDB__

#ifdef __cplusplus
}
#endif
