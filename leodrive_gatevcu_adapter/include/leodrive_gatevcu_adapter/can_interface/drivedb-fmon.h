#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// DBC file version
#define VER_DRIVEDB_MAJ_FMON (0U)
#define VER_DRIVEDB_MIN_FMON (0U)

#include "drivedb-config.h"

#ifdef DRIVEDB_USE_DIAG_MONITORS

#include "canmonitorutil.h"
/*
This file contains the prototypes of all the functions that will be called
from each Unpack_*name* function to detect DBC related errors
It is the user responsibility to defined these functions in the
separated .c file. If it won't be done the linkage error will happen
*/

#ifdef DRIVEDB_USE_MONO_FMON

void _FMon_MONO_drivedb(FrameMonitor_t* _mon, uint32_t msgid);

#define FMon_LongitudinalCommandsV1_drivedb(x, y) _FMon_MONO_drivedb((x), (y))
#define FMon_LongitudinalCommandsV2_drivedb(x, y) _FMon_MONO_drivedb((x), (y))
#define FMon_VehicleCommands_drivedb(x, y) _FMon_MONO_drivedb((x), (y))
#define FMon_FrontWheelCommands_drivedb(x, y) _FMon_MONO_drivedb((x), (y))
#define FMon_VehicleDynamicsInfo_drivedb(x, y) _FMon_MONO_drivedb((x), (y))
#define FMon_VehicleSignalStatus_drivedb(x, y) _FMon_MONO_drivedb((x), (y))
#define FMon_MotionInfo_drivedb(x, y) _FMon_MONO_drivedb((x), (y))
#define FMon_motor_info_drivedb(x, y) _FMon_MONO_drivedb((x), (y))
#define FMon_VehicleErrors_drivedb(x, y) _FMon_MONO_drivedb((x), (y))
#define FMon_Drive_Feedback_drivedb(x, y) _FMon_MONO_drivedb((x), (y))

#else

void _FMon_LongitudinalCommandsV1_drivedb(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_LongitudinalCommandsV2_drivedb(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_VehicleCommands_drivedb(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_FrontWheelCommands_drivedb(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_VehicleDynamicsInfo_drivedb(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_VehicleSignalStatus_drivedb(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_MotionInfo_drivedb(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_motor_info_drivedb(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_VehicleErrors_drivedb(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_Drive_Feedback_drivedb(FrameMonitor_t* _mon, uint32_t msgid);

#define FMon_LongitudinalCommandsV1_drivedb(x, y) _FMon_LongitudinalCommandsV1_drivedb((x), (y))
#define FMon_LongitudinalCommandsV2_drivedb(x, y) _FMon_LongitudinalCommandsV2_drivedb((x), (y))
#define FMon_VehicleCommands_drivedb(x, y) _FMon_VehicleCommands_drivedb((x), (y))
#define FMon_FrontWheelCommands_drivedb(x, y) _FMon_FrontWheelCommands_drivedb((x), (y))
#define FMon_VehicleDynamicsInfo_drivedb(x, y) _FMon_VehicleDynamicsInfo_drivedb((x), (y))
#define FMon_VehicleSignalStatus_drivedb(x, y) _FMon_VehicleSignalStatus_drivedb((x), (y))
#define FMon_MotionInfo_drivedb(x, y) _FMon_MotionInfo_drivedb((x), (y))
#define FMon_motor_info_drivedb(x, y) _FMon_motor_info_drivedb((x), (y))
#define FMon_VehicleErrors_drivedb(x, y) _FMon_VehicleErrors_drivedb((x), (y))
#define FMon_Drive_Feedback_drivedb(x, y) _FMon_Drive_Feedback_drivedb((x), (y))

#endif

#endif // DRIVEDB_USE_DIAG_MONITORS

#ifdef __cplusplus
}
#endif
