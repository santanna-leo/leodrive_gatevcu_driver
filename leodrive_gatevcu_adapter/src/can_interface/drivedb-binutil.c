#include "leodrive_gatevcu_adapter/can_interface/drivedb-binutil.h"

// DBC file version
#if (VER_DRIVEDB_MAJ != (0U)) || (VER_DRIVEDB_MIN != (0U))
#error The DRIVEDB binutil source file has inconsistency with core dbc lib!
#endif

#ifdef __DEF_DRIVEDB__

drivedb_rx_t drivedb_rx;

#endif // __DEF_DRIVEDB__

uint32_t drivedb_Receive(drivedb_rx_t* _m, const uint8_t* _d, uint32_t _id, uint8_t dlc_)
{
 uint32_t recid = 0;
 if ((_id >= 0x400U) && (_id < 0x412U)) {
  if ((_id >= 0x400U) && (_id < 0x402U)) {
   if (_id == 0x400U) {
    recid = Unpack_LongitudinalCommandsV1_drivedb(&(_m->LongitudinalCommandsV1), _d, dlc_);
   } else if (_id == 0x401U) {
    recid = Unpack_LongitudinalCommandsV2_drivedb(&(_m->LongitudinalCommandsV2), _d, dlc_);
   }
  } else {
   if (_id == 0x402U) {
    recid = Unpack_VehicleCommands_drivedb(&(_m->VehicleCommands), _d, dlc_);
   } else {
    if (_id == 0x403U) {
     recid = Unpack_FrontWheelCommands_drivedb(&(_m->FrontWheelCommands), _d, dlc_);
    } else if (_id == 0x411U) {
     recid = Unpack_VehicleDynamicsInfo_drivedb(&(_m->VehicleDynamicsInfo), _d, dlc_);
    }
   }
  }
 } else {
  if ((_id >= 0x412U) && (_id < 0x414U)) {
   if (_id == 0x412U) {
    recid = Unpack_VehicleSignalStatus_drivedb(&(_m->VehicleSignalStatus), _d, dlc_);
   } else if (_id == 0x413U) {
    recid = Unpack_MotionInfo_drivedb(&(_m->MotionInfo), _d, dlc_);
   }
  } else {
   if (_id == 0x414U) {
    recid = Unpack_motor_info_drivedb(&(_m->motor_info), _d, dlc_);
   } else {
    if (_id == 0x415U) {
     recid = Unpack_VehicleErrors_drivedb(&(_m->VehicleErrors), _d, dlc_);
    } else if (_id == 0x416U) {
     recid = Unpack_Drive_Feedback_drivedb(&(_m->Drive_Feedback), _d, dlc_);
    }
   }
  }
 }

 return recid;
}

