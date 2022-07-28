# %% Imports
from __future__ import annotations
from nicelib import load_lib, NiceLib, Sig, NiceObject, RetHandler, ret_ignore
from cffi import FFI
from inspect import getmembers
import warnings

# %%
package_ffi = FFI()

# %% Converting struct to Dictionary
def cdata_dict(cd, package_ffi: FFI):
    if isinstance(cd, package_ffi.CData):
        try:
            return package_ffi.string(cd).decode('utf-8')
        except TypeError:
            try:
                return [cdata_dict(x, package_ffi) for x in cd]
            except TypeError:
                return {k: cdata_dict(v, package_ffi) for k, v in getmembers(cd)}
    else:
        return cd
# %% Struct definitions.
package_ffi.cdef("""
struct TLI_DeviceInfo
{
   DWORD typeID;
   char description[65];
   char serialNo[16];
   DWORD PID;
   bool isKnownType;
   MOT_MotorTypes motorType;
   bool isPiezoDevice;
   bool isLaser;
   bool isCustomType;
   bool isRack;
   short maxChannels;
};
""", packed=1) # Defines struct, with packing.

package_ffi.cdef("""
struct TLI_HardwareInformation
{
   DWORD serialNumber;
   char modelNumber[8];
   WORD type;
   DWORD firmwareVersion;
   char notes[48];
   BYTE deviceDependantData[12];
   WORD hardwareVersion;
   WORD modificationState;
   short numChannels;
};
""", packed=1) # Defines struct, with packing.

package_ffi.cdef("""
struct MOT_VelocityParameters
{
   int minVelocity;
   int acceleration;
   int maxVelocity;
};
""", packed=1) # Defines struct, with packing.

package_ffi.cdef("""
struct MOT_JogParameters
{
   MOT_JogModes mode;
   unsigned int stepSize;
   MOT_VelocityParameters velParams;
   MOT_StopModes stopMode;
};
""", packed=1) # Defines struct, with packing.

package_ffi.cdef("""
struct MOT_HomingParameters
{
   MOT_TravelDirection direction;
   MOT_HomeLimitSwitchDirection limitSwitch;
   unsigned int velocity;
   unsigned int offsetDistance;
};
""", packed=1) # Defines struct, with packing.

package_ffi.cdef("""
struct MOT_LimitSwitchParameters
{
   MOT_LimitSwitchModes clockwiseHardwareLimit;
   MOT_LimitSwitchModes anticlockwiseHardwareLimit;
   DWORD clockwisePosition;
   DWORD anticlockwisePosition;
   MOT_LimitSwitchSWModes softLimitMode;
};
""", packed=1) # Defines struct, with packing.

package_ffi.cdef("""
struct MOT_PowerParameters
{
   WORD restPercentage;
   WORD movePercentage;
};
""", packed=1) # Defines struct, with packing.

package_ffi.cdef("""
struct KMOT_MMIParams
{
   KMOT_WheelMode WheelMode;
   __int32 WheelMaxVelocity;
   __int32 WheelAcceleration;
   MOT_DirectionSense WheelDirectionSense;
   __int32 PresetPos1;
   __int32 PresetPos2;
   __int16 DisplayIntensity;
   __int16 DisplayTimeout;
   __int16 DisplayDimIntensity;
   __int16 reserved[4];
};
""", packed=1) # Defines struct, with packing.

package_ffi.cdef("""
struct KMOT_TriggerConfig
{
   KMOT_TriggerPortMode Trigger1Mode;
   KMOT_TriggerPortPolarity Trigger1Polarity;
   KMOT_TriggerPortMode Trigger2Mode;
   KMOT_TriggerPortPolarity Trigger2Polarity;
   __int16 reserved[6];
};
""", packed=1) # Defines struct, with packing.

package_ffi.cdef("""
struct KMOT_TriggerParams
{
   __int32 TriggerStartPositionFwd;
   __int32 TriggerIntervalFwd;
   __int32 TriggerPulseCountFwd;
   __int32 TriggerStartPositionRev;
   __int32 TriggerIntervalRev;
   __int32 TriggerPulseCountRev;
   __int32 TriggerPulseWidth;
   __int32 CycleCount;
   __int32 reserved[6];
};
""", packed=1) # Defines struct, with packing.

package_ffi.cdef("""
struct MOT_PIDLoopEncoderParams
{
   MOT_PID_LoopMode loopMode;
   int proportionalGain;
   int integralGain;
   int differentialGain;
   int PIDOutputLimit;
   int PIDTolerance;
};
""", packed=1) # Defines struct, with packing.

package_ffi.cdef("""
struct tagSAFEARRAYBOUND
{
   ULONG cElements;
   LONG lLbound;
};
""", packed=1) # Defines struct, with packing.

package_ffi.cdef("""
struct tagSAFEARRAY
{
   USHORT cDims;
   USHORT fFeatures;
   ULONG cbElements;
   ULONG cLocks;
   PVOID pvData;
   SAFEARRAYBOUND rgsabound[ 1 ];
};
""", packed=1) # Defines struct, with packing.

