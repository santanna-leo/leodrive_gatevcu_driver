#include "leodrive_gatevcu_adapter/can_interface/drivedb-fmon.h"

#ifdef DRIVEDB_USE_DIAG_MONITORS

/*
Put the monitor function content here, keep in mind -
next generation will completely clear all manually added code (!)
*/

#ifdef DRIVEDB_USE_MONO_FMON

void _FMon_MONO_drivedb(FrameMonitor_t* _mon, uint32_t msgid)
{
  (void)_mon;
  (void)msgid;
}

#else

void _FMon_LongitudinalCommandsV1_drivedb(FrameMonitor_t* _mon, uint32_t msgid)
{
  (void)_mon;
  (void)msgid;
}

void _FMon_LongitudinalCommandsV2_drivedb(FrameMonitor_t* _mon, uint32_t msgid)
{
  (void)_mon;
  (void)msgid;
}

void _FMon_VehicleCommands_drivedb(FrameMonitor_t* _mon, uint32_t msgid)
{
  (void)_mon;
  (void)msgid;
}

void _FMon_FrontWheelCommands_drivedb(FrameMonitor_t* _mon, uint32_t msgid)
{
  (void)_mon;
  (void)msgid;
}

void _FMon_VehicleDynamicsInfo_drivedb(FrameMonitor_t* _mon, uint32_t msgid)
{
  (void)_mon;
  (void)msgid;
}

void _FMon_VehicleSignalStatus_drivedb(FrameMonitor_t* _mon, uint32_t msgid)
{
  (void)_mon;
  (void)msgid;
}

void _FMon_MotionInfo_drivedb(FrameMonitor_t* _mon, uint32_t msgid)
{
  (void)_mon;
  (void)msgid;
}

void _FMon_motor_info_drivedb(FrameMonitor_t* _mon, uint32_t msgid)
{
  (void)_mon;
  (void)msgid;
}

void _FMon_VehicleErrors_drivedb(FrameMonitor_t* _mon, uint32_t msgid)
{
  (void)_mon;
  (void)msgid;
}

void _FMon_Drive_Feedback_drivedb(FrameMonitor_t* _mon, uint32_t msgid)
{
  (void)_mon;
  (void)msgid;
}

#endif // DRIVEDB_USE_MONO_FMON

#endif // DRIVEDB_USE_DIAG_MONITORS
