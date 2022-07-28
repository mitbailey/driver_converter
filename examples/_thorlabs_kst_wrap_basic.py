
# %% Imports
from __future__ import annotations
from nicelib import load_lib, NiceLib, Sig, NiceObject, RetHandler, ret_ignore
from cffi import FFI
from inspect import getmembers
import warnings
# %% Get current function name
def __funcname__():
    import inspect
    return inspect.stack()[1][3]

# %% Return value strings
err_codes = [
    'Success',
    'FT_InvalidHandle - The FTDI functions have not been initialized',
    'FT_DeviceNotFound - The Device could not be found',
    'FT_DeviceNotOpened - The Device must be opened before it can be accessed',
    'FT_IOError - An I/O Error has occured in the FTDI chip',
    'FT_InsufficientResources - There are Insufficient resources to run this application',
    'FT_InvalidParameter - An invalid parameter has been supplied to the device',
    'FT_DeviceNotPresent - The Device is no longer present',
    'FT_IncorrectDevice - The device detected does not match that expected'
]
# Enums
FT_Status = [
    'FT_OK',
    'FT_InvalidHandle',
    'FT_DeviceNotFound',
    'FT_DeviceNotOpened',
    'FT_IOError',
    'FT_InsufficientResources',
    'FT_InvalidParameter',
    'FT_DeviceNotPresent',
    'FT_IncorrectDevice'
]
MOT_MotorTypes = [
    'MOT_NotMotor',
    'MOT_DCMotor',
    'MOT_StepperMotor',
    'MOT_BrushlessMotor',
    'MOT_CustomMotor'
] 
MOT_TravelModes = [
    'MOT_TravelModeUndefined',
    'MOT_Linear',
    'MOT_Rotational'
]
MOT_TravelDirection = [
    'MOT_TravelDirectionUndefined',
    'MOT_Forwards',
    'MOT_Backwards',
]
KST_Stages = [
    'ZST6',
    'ZST13',
    'ZST25',
    'ZST206',
    'ZST213',
    'ZST225',
    'ZFS206',
    'ZFS213',
    'ZFS225',
    'DRV013_25MM',
    'DRV014_50MM',
    'NR360',
    'PLS_X25MM',
    'PLS_X25MM_HiRes',
    'FW103'
]
MOT_HomeLimitSwitchDirection = [
    'MOT_LimitSwitchDirectionUndefined',
    'MOT_ReverseLimitSwitch',
    'MOT_ForwardLimitSwitch'
]
MOT_DirectionSense = [
    'MOT_Normal',
    'MOT_Reverse'
]
MOT_JogModes = [
    'MOT_JogModeUndefined',
    'MOT_Continuous',
    'MOT_SingleStep',
]
MOT_StopModes = [
    'MOT_StopModeUndefined',
    'MOT_Immediate',
    'MOT_Profiled'
]
MOT_LimitSwitchModes = [
    'MOT_LimitSwitchModeUndefined',
    'MOT_LimitSwitchIgnoreSwitch',
    'MOT_LimitSwitchMakeOnContact',
    'MOT_LimitSwitchBreakOnContact',
    'MOT_LimitSwitchMakeOnHome',
    'MOT_LimitSwitchBreakOnHome',
    'MOT_PMD_Reserved',
    'MOT_LimitSwitchIgnoreSwitchSwapped',
    'MOT_LimitSwitchMakeOnContactSwapped',
    'MOT_LimitSwitchBreakOnContactSwapped',
    'MOT_LimitSwitchMakeOnHomeSwapped',
    'MOT_LimitSwitchBreakOnHomeSwapped'
]
MOT_LimitSwitchSWModes = [
    'MOT_LimitSwitchSWModeUndefined',
    'MOT_LimitSwitchIgnored',
    'MOT_LimitSwitchStopImmediate',
    'MOT_LimitSwitchStopProfiled',
    'MOT_LimitSwitchIgnored_Rotational',
    'MOT_LimitSwitchStopImmediate_Rotational',
    'MOT_LimitSwitchStopProfiled_Rotational'
]
MOT_LimitsSoftwareApproachPolicy = [
    'DisallowIllegalMoves',
    'AllowPartialMoves',
    'AllowAllMoves'
]
KMOT_WheelDirectionSense = [
    'KMOT_WM_Positive',
    'KMOT_WM_Negative'
]
KMOT_WheelMode = [
    'KMOT_WM_Velocity',
    'KMOT_WM_Jog',
    'KMOT_WM_MoveAbsolute'
]
KMOT_TriggerPortMode = [
    'KMOT_TrigDisabled',
    'KMOT_TrigIn_GPI',
    'KMOT_TrigIn_RelativeMove',
    'KMOT_TrigIn_AbsoluteMove',
    'KMOT_TrigIn_Home',
    'KMOT_TrigIn_Stop',
    'KMOT_TrigOut_GPO',
    'KMOT_TrigOut_InMotion',
    'KMOT_TrigOut_AtMaxVelocity',
    'KMOT_TrigOut_AtPositionSteps',
    'KMOT_TrigOut_Synch'
]
KMOT_TriggerPortPolarity = [
    'KMOT_TrigPolarityHigh',
    'KMOT_TrigPolarityLow'
]
MOT_PID_LoopMode = [
    'MOT_PIDLoopModeDisabled',
    'MOT_PIDOpenLoopMode',
    'MOT_PIDClosedLoopMode'
]
MOT_MovementModes = [
    'LinearRange',
    'RotationalUnlimited',
    'RotationalWrapping'
]
MOT_MovementDirections = [
    'Quickest',
    'Forwards',
    'Reverse'
]

# %% Return Handler Example
@RetHandler(num_retvals=0)
def ret_errcode(retval):
    return retval

# %% Converting struct to Dictionary
def cdata_dict(cd, ffi: FFI):
    if isinstance(cd, ffi.CData):
        try:
            return ffi.string(cd).decode('utf-8')
        except TypeError:
            try:
                return [cdata_dict(x, ffi) for x in cd]
            except TypeError:
                return {k: cdata_dict(v, ffi) for k, v in getmembers(cd)}
    else:
        return cd

# %% Struct definitions.
package_ffi = FFI()

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
ser_buf = package_ffi.new('struct TLI_DeviceInfo *') # Creates memory for struct.
TLI_DeviceInfo_DICT = cdata_dict(ser_buf, package_ffi)

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
ser_buf = package_ffi.new('struct TLI_HardwareInformation *') # Creates memory for struct.
TLI_HardwareInformation_DICT = cdata_dict(ser_buf, package_ffi)

package_ffi.cdef("""
struct MOT_VelocityParameters
{
   int minVelocity;
   int acceleration;
   int maxVelocity;
};
""", packed=1) # Defines struct, with packing.
ser_buf = package_ffi.new('struct MOT_VelocityParameters *') # Creates memory for struct.
MOT_VelocityParameters_DICT = cdata_dict(ser_buf, package_ffi)

package_ffi.cdef("""
struct MOT_JogParameters
{
   MOT_JogModes mode;
   unsigned int stepSize;
   MOT_VelocityParameters velParams;
   MOT_StopModes stopMode;
};
""", packed=1) # Defines struct, with packing.
ser_buf = package_ffi.new('struct MOT_JogParameters *') # Creates memory for struct.
MOT_JogParameters_DICT = cdata_dict(ser_buf, package_ffi)

package_ffi.cdef("""
struct MOT_HomingParameters
{
   MOT_TravelDirection direction;
   MOT_HomeLimitSwitchDirection limitSwitch;
   unsigned int velocity;
   unsigned int offsetDistance;
};
""", packed=1) # Defines struct, with packing.
ser_buf = package_ffi.new('struct MOT_HomingParameters *') # Creates memory for struct.
MOT_HomingParameters_DICT = cdata_dict(ser_buf, package_ffi)

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
ser_buf = package_ffi.new('struct MOT_LimitSwitchParameters *') # Creates memory for struct.
MOT_LimitSwitchParameters_DICT = cdata_dict(ser_buf, package_ffi)

package_ffi.cdef("""
struct MOT_PowerParameters
{
   WORD restPercentage;
   WORD movePercentage;
};
""", packed=1) # Defines struct, with packing.
ser_buf = package_ffi.new('struct MOT_PowerParameters *') # Creates memory for struct.
MOT_PowerParameters_DICT = cdata_dict(ser_buf, package_ffi)

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
ser_buf = package_ffi.new('struct KMOT_MMIParams *') # Creates memory for struct.
KMOT_MMIParams_DICT = cdata_dict(ser_buf, package_ffi)

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
ser_buf = package_ffi.new('struct KMOT_TriggerConfig *') # Creates memory for struct.
KMOT_TriggerConfig_DICT = cdata_dict(ser_buf, package_ffi)

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
ser_buf = package_ffi.new('struct KMOT_TriggerParams *') # Creates memory for struct.
KMOT_TriggerParams_DICT = cdata_dict(ser_buf, package_ffi)

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
ser_buf = package_ffi.new('struct MOT_PIDLoopEncoderParams *') # Creates memory for struct.
MOT_PIDLoopEncoderParams_DICT = cdata_dict(ser_buf, package_ffi)

package_ffi.cdef("""
struct tagSAFEARRAYBOUND
{
   ULONG cElements;
   LONG lLbound;
};
""", packed=1) # Defines struct, with packing.
ser_buf = package_ffi.new('struct tagSAFEARRAYBOUND *') # Creates memory for struct.
tagSAFEARRAYBOUND_DICT = cdata_dict(ser_buf, package_ffi)

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
ser_buf = package_ffi.new('struct tagSAFEARRAY *') # Creates memory for struct.
tagSAFEARRAY_DICT = cdata_dict(ser_buf, package_ffi)

# %% TLI Functions
class TLIBase(NiceLib): # this would be the base of the Thorlabs namespace, which would provide static methods common across all Thorlabs sub APIs
    _info_ = load_lib('thorlabs_kst_')
    _ret_ = ret_errcode
    _prefix_ = 'TLI_'

    # Function list auto-generated by fn_convert.py.
    BuildDeviceList = Sig()
    GetDeviceListSize = Sig()
    GetDeviceList = Sig('out')
    GetDeviceListByType = Sig('out', 'in')
    GetDeviceListByTypes = Sig('out', 'in', 'in')
    GetDeviceListExt = Sig('in', 'in')
    GetDeviceListByTypeExt = Sig('in', 'in', 'in')
    GetDeviceListByTypesExt = Sig('in', 'in', 'in', 'in')
    GetDeviceInfo = Sig('in', 'in')
    InitializeSimulations = Sig(ret=ret_ignore)
    UninitializeSimulations = Sig(ret=ret_ignore)

# %%
class TLI_KST(NiceLib): # This would be KST101, a subclass of Thorlabs
    _info_ = load_lib('thorlabs_kst_')
    _prefix_ = 'SCC_'

    # Function list auto-generated by fn_convert.py.
    Open = Sig('in')
    Close = Sig('in', ret = ret_ignore)
    CheckConnection = Sig('in')
    Identify = Sig('in', ret = ret_ignore)
    SetStageType = Sig('in', 'in')
    GetHardwareInfoBlock = Sig('in', 'in') # ser, TLI_HardwareInformation
    # Open = Sig('in')
    # Close = Sig('in', ret=ret_ignore)
    # CheckConnection = Sig('in')
    # Identify = Sig('in', ret=ret_ignore)
    # SetStageType = Sig('in', 'in')
    # GetHardwareInfoBlock = Sig('in', 'in')
    GetSoftwareVersion = Sig('in')
    SetCalibrationFile = Sig('in', 'in', 'in', ret=ret_ignore)
    IsCalibrationActive = Sig('in')
    GetCalibrationFile = Sig('in', 'in', 'in')
    GetHubBay = Sig('in')
    LoadSettings = Sig('in')
    LoadNamedSettings = Sig('in', 'in')
    PersistSettings = Sig('in')
    DisableChannel = Sig('in')
    EnableChannel = Sig('in')
    CanDeviceLockFrontPanel = Sig('in')
    GetFrontPanelLocked = Sig('in')
    RequestFrontPanelLocked = Sig('in')
    SetFrontPanelLock = Sig('in', 'in')
    GetNumberPositions = Sig('in')
    MoveToPosition = Sig('in', 'in')
    GetPosition = Sig('in')
    CanHome = Sig('in')
    NeedsHoming = Sig('in')
    CanMoveWithoutHomingFirst = Sig('in')
    Home = Sig('in')
    ClearMessageQueue = Sig('in', ret=ret_ignore)
    RegisterMessageCallback = Sig('in', 'in', ret=ret_ignore)
    MessageQueueSize = Sig('in')
    GetNextMessage = Sig('in', 'in', 'in', 'in')
    WaitForMessage = Sig('in', 'in', 'in', 'in')
    RequestHomingParams = Sig('in')
    GetHomingVelocity = Sig('in')
    SetHomingVelocity = Sig('in', 'in')
    MoveRelative = Sig('in', 'in')
    RequestJogParams = Sig('in')
    GetJogMode = Sig('in', 'in', 'in')
    SetJogMode = Sig('in', 'in', 'in')
    GetJogStepSize = Sig('in')
    SetJogStepSize = Sig('in', 'in')
    GetJogVelParams = Sig('in', 'in', 'in')
    SetJogVelParams = Sig('in', 'in', 'in')
    MoveJog = Sig('in', 'in')
    RequestVelParams = Sig('in')
    GetVelParams = Sig('in', 'in', 'in')
    SetVelParams = Sig('in', 'in', 'in')
    MoveAtVelocity = Sig('in', 'in')
    SetDirection = Sig('in', 'in', ret=ret_ignore)
    StopImmediate = Sig('in')
    StopProfiled = Sig('in')
    RequestBacklash = Sig('in')
    GetBacklash = Sig('in')
    SetBacklash = Sig('in', 'in')
    GetPositionCounter = Sig('in')
    SetPositionCounter = Sig('in', 'in')
    RequestEncoderCounter = Sig('in')
    GetEncoderCounter = Sig('in')
    SetEncoderCounter = Sig('in', 'in')
    RequestLimitSwitchParams = Sig('in')
    GetLimitSwitchParams = Sig('in', 'in', 'in', 'in', 'in', 'in')
    SetLimitSwitchParams = Sig('in', 'in', 'in', 'in', 'in', 'in')
    GetSoftLimitMode = Sig('in')
    SetLimitsSoftwareApproachPolicy = Sig('in', 'in', ret=ret_ignore)
    RequestMMIParams = Sig('in')
    RequestTriggerConfigParams = Sig('in')
    GetTriggerConfigParams = Sig('in', 'in', 'in', 'in', 'in')
    SetTriggerConfigParams = Sig('in', 'in', 'in', 'in', 'in')
    RequestPosTriggerParams = Sig('in')
    GetTriggerConfigParamsBlock = Sig('in', 'in')
    SetTriggerConfigParamsBlock = Sig('in', 'in')
    GetTriggerParamsParamsBlock = Sig('in', 'in')
    SetTriggerParamsParamsBlock = Sig('in', 'in')
    GetVelParamsBlock = Sig('in', 'in')
    SetVelParamsBlock = Sig('in', 'in')
    RequestMoveAbsolutePosition = Sig('in')
    SetMoveAbsolutePosition = Sig('in', 'in')
    GetMoveAbsolutePosition = Sig('in')
    MoveAbsolute = Sig('in')
    RequestMoveRelativeDistance = Sig('in')
    SetMoveRelativeDistance = Sig('in', 'in')
    GetMoveRelativeDistance = Sig('in')
    MoveRelativeDistance = Sig('in')
    GetHomingParamsBlock = Sig('in', 'in')
    SetHomingParamsBlock = Sig('in', 'in')
    GetJogParamsBlock = Sig('in', 'in')
    SetJogParamsBlock = Sig('in', 'in')
    GetLimitSwitchParamsBlock = Sig('in', 'in')
    SetLimitSwitchParamsBlock = Sig('in', 'in')
    RequestPowerParams = Sig('in')
    GetPowerParams = Sig('in', 'in')
    SetPowerParams = Sig('in', 'in')
    RequestBowIndex = Sig('in')
    GetBowIndex = Sig('in')
    SetBowIndex = Sig('in', 'in')
    UsesPIDLoopEncoding = Sig('in')
    SetPIDLoopEncoderParams = Sig('in', 'in')
    SetPIDLoopEncoderCoeff = Sig('in', 'in')
    RequestPIDLoopEncoderParams = Sig('in')
    GetPIDLoopEncoderParams = Sig('in', 'in')
    GetPIDLoopEncoderCoeff = Sig('in')
    SuspendMoveMessages = Sig('in')
    ResumeMoveMessages = Sig('in')
    RequestPosition = Sig('in')
    RequestStatusBits = Sig('in')
    GetStatusBits = Sig('in')
    StartPolling = Sig('in', 'in')
    PollingDuration = Sig('in')
    StopPolling = Sig('in', ret=ret_ignore)
    TimeSinceLastMsgReceived = Sig('in', 'in')
    EnableLastMsgTimer = Sig('in', 'in', 'in', ret=ret_ignore)
    HasLastMsgTimerOverrun = Sig('in')
    RequestSettings = Sig('in')
    GetStageAxisMinPos = Sig('in')
    GetStageAxisMaxPos = Sig('in')
    SetStageAxisLimits = Sig('in', 'in', 'in')
    SetMotorTravelMode = Sig('in', 'in')
    GetMotorTravelMode = Sig('in')
    SetMotorParams = Sig('in', 'in', 'in', 'in')
    GetMotorParams = Sig('in', 'in', 'in', 'in')
    SetMotorParamsExt = Sig('in', 'in', 'in', 'in')
    GetMotorParamsExt = Sig('in', 'in', 'in', 'in')
    SetMotorVelocityLimits = Sig('in', 'in', 'in')
    GetMotorVelocityLimits = Sig('in', 'in', 'in')
    ResetRotationModes = Sig('in')
    SetRotationModes = Sig('in', 'in', 'in')
    SetMotorTravelLimits = Sig('in', 'in', 'in')
    GetMotorTravelLimits = Sig('in', 'in', 'in')
    RequestDigitalOutputs = Sig('in')
    GetDigitalOutputs = Sig('in')
    SetDigitalOutputs = Sig('in', 'in')
    GetRealValueFromDeviceUnit = Sig('in', 'in', 'in', 'in')
    GetDeviceUnitFromRealValue = Sig('in', 'in', 'in', 'in')

# similarly, a TLI_KSG class can be defined for TLI Strain gauges



############## This should be the end of the Level 2 Wrapper ##################
###############################################################################
#%%
################################################################################ 
############### Level 3 Wrapper (final stage) begin ############################

# %%
class Thorlabs: # Wrapper class for TLI methods
    TYPE_KST101 = 26
    @staticmethod
    def ListDevicesAny(ser_buf_len: int = 512) -> list:
        """List all available (unopened) Thorlabs devices.

        Args:
            ser_buf_len (int, optional): String buffer length where the serials are read into. Defaults to 512.

        Raises:
            RuntimeError: Failed to build device list
            RuntimeError: Failed to get device list
            RuntimeWarning: Incomplete serial found

        Returns:
            list: Serial numbers of available devices
        """
        ret = TLIBase.BuildDeviceList()
        if ret:
            raise RuntimeError('Could not create device list: (%d) %s'%(ret, err_codes[ret]))
        num_devices = TLIBase.GetDeviceListSize()
        if not num_devices:
            return []
        ffi = FFI()
        ser_buf = ffi.new('char[]', ser_buf_len)
        ret = TLIBase.GetDeviceListExt(ser_buf, ser_buf_len)
        if ret:
            raise RuntimeError('Thorlabs:%s(): %s (%d).'%(__funcname__(), err_codes[ret], ret))
        ser_buf = ffi.string(ser_buf).decode('utf-8')
        ser_vals = ser_buf.split(',')
        for idx in range(len(ser_vals)):
            if ser_vals[idx] == '':
                del ser_vals[idx]
            elif len(ser_vals[idx]) != 8:
                del ser_vals[idx]
                warnings.warn('Serial %s invalid, please check, consider increasing buffer length.')
            else:
                ser_vals[idx] = int(ser_vals[idx])
        ser_vals.sort() # sort by ID
        return ser_vals

    @staticmethod
    def ListDevices(typeID: int) -> list:
        """List all available (unopened) Thorlabs devices of given type ID.

        Args:
            typeID (int): Thorlabs Device Type ID Number

        Raises:
            RuntimeError: Failed to build device list
            RuntimeError: Failed to get device list

        Returns:
            list: Serial numbers of available devices
        """
        ret = TLIBase.BuildDeviceList()
        if ret:
            raise RuntimeError('Could not create device list: (%d) %s'%(ret, err_codes[ret]))
        num_devices = TLIBase.GetDeviceListSize()
        if not num_devices:
            return []
        ffi = FFI()
        ser_buf_len = 100
        ser_buf = ffi.new('char[]', ser_buf_len)
        ret = TLIBase.GetDeviceListByTypeExt(ser_buf, ser_buf_len, typeID)
        if ret:
            raise RuntimeError('Could not get device list by type ID %d: (%d) %s'%(typeID, ret, err_codes[ret]))
        ser_buf = ffi.string(ser_buf).decode('utf-8')
        ser_vals = ser_buf.split(',')
        for idx in range(len(ser_vals)):
            if ser_vals[idx] == '':
                del ser_vals[idx]
            else:
                ser_vals[idx] = int(ser_vals[idx])
        ser_vals.sort()
        return ser_vals

    @staticmethod
    def GetDeviceInfo(serialNumber: int | list) -> dict:
        """Get Device Info for a given Thorlabs Serial Number.

            Args:
                typeID (int | list(int)): Thorlabs Serial Numbers.

            Raises:
                RuntimeError: Could not get device info for serial.

            Returns:
                dict: Device Information (struct TLI_DeviceInfo)
        """

        if isinstance(serialNumber, int):
            ffi = FFI()
            ffi.cdef("""
            struct TLI_DeviceInfo
            {
                DWORD typeID;
                char description[65];
                char serialNo[16];
                DWORD PID;
                bool isKnownType;
                int motorType;
                bool isPiezoDevice;
                bool isLaser;
                bool isCustomType;
                bool isRack;
                short maxChannels;
            };
            """, packed=True) # define struct, with packing
            ser_buf = ffi.new('struct TLI_DeviceInfo *') # create memory for struct
            ret = TLIBase.GetDeviceInfo(str(serialNumber), ser_buf) # pass pointer
            if not ret:
                raise RuntimeError('Could not get device info for serial %d'%(serialNumber))
            return cdata_dict(ser_buf, ffi)

        elif isinstance(serialNumber, list):
            out = []
            for s in serialNumber:
                out.append(Thorlabs.GetDeviceInfo(s))
            return out

        else:
            raise TypeError('Serial number can be an int or a list of int.')

    class KST101: # Subclass for KST101 devices
        # TLI List method
        open_devices = [] # List of opened devices
        @staticmethod
        def ListDevices() -> list:
            """List all available KST101 devices (including opened devices).

            Returns:
                list: Serial numbers of available KST101 devices.

            NOTE: Backport listing opened devices feature to Thorlabs.ListDevicesAny() and Thorlabs.ListDevices()
            """
            devs = Thorlabs.ListDevices(Thorlabs.TYPE_KST101) # special case
            _devs = Thorlabs.KST101.open_devices + devs
            _devs = list(set(_devs))
            return _devs
        
        # Init stores the serial number
        def __init__(self, serialNumber: int):
            """Create an instance of Thorlabs KST101 Stepper Motor Controller

            Args:
                serialNumber (int): Serial number of the KST101 Controller

            Raises:
                ValueError: Invalid serial number
                RuntimeError: Instance of KST101 exists with this serial number
                RuntimeError: Serial number not in device list
            """
            if str(serialNumber)[:2] != str(Thorlabs.TYPE_KST101):
                raise ValueError('Invalid serial %d: KST101 Serial starts with %s'%(serialNumber, str(Thorlabs.TYPE_KST101)))
            elif serialNumber in Thorlabs.KST101.open_devices:
                raise RuntimeError('Serial %d already in use.'%(serialNumber))
            elif serialNumber not in Thorlabs.KST101.ListDevices():
                raise RuntimeError('Serial %d not in device list.'%(serialNumber))
            self.serial = str(serialNumber)
            self.open = False

        # SCC Methods
        def Open(self) -> bool:
            """Open connection to the KST101 Controller.

            Raises:
                RuntimeError: Connection to device is already open.
                RuntimeError: Error opening connection to device.

            Returns:
                bool: _description_
            """
            if int(self.serial) in Thorlabs.KST101.open_devices:
                raise RuntimeError('Device %d is already open.'%(int(self.serial)))
            ret = TLI_KST.Open(self.serial)
            if ret:
                raise RuntimeError('KST101:%s(): %d (%s)'%(__funcname__(), ret, err_codes[ret]))
            Thorlabs.KST101.open_devices.append(int(self.serial))
            self.open = True
            return True

        def __del__(self):
            if self.open:
                self.Close()
        
        def Close(self) -> None:
            """Close connection to the KST101 Controller.
            """
            TLI_KST.Close(self.serial)
            Thorlabs.KST101.open_devices.remove(int(self.serial))
            self.serial = None
            self.open = False

        def CheckConnection(self) -> bool:
            """Check connection to the device.

            Returns:
                bool: True if the device is connected.
            """
            return TLI_KST.CheckConnection(self.serial)
        
        def Identify(self) -> None:
            """Identify the device by blinking the display backlight.

            Raises:
                RuntimeWarning: Connection to device is not open.

            Returns:
                None
            """
            if not self.open:
                self.Open()
            TLI_KST.Identify(self.serial)
            return
        
        # Should this be automatically performed on open?
        def GetHardwareInfo(self) -> dict:
            """Get the hardware information for this device.

            Raises:
                RuntimeError: GetHardwareInfoBlock call fails
            
            Returns:
                dict: Information in struct TLI_HardwareInformation
            """
            if not self.open:
                self.Open()
            ffi = FFI()
            ffi.cdef("""
            struct TLI_HardwareInformation
            {
                DWORD serialNumber;
                char modelNumber[8];
                WORD type;
                DWORD firmwareVersion;
                char notes[48];
                unsigned char deviceDependantData[12];
                WORD hardwareVersion;
                WORD modificationState;
                short numChannels;};
            """)
            ser_buf = ffi.new('struct TLI_HardwareInformation *')
            ret = TLI_KST.GetHardwareInfoBlock(self.serial, ser_buf)
            if ret:
                raise RuntimeError('KST101:%s(): %d (%s)'%(__funcname__(), ret, err_codes[ret]))
            return cdata_dict(ser_buf, ffi)

    
# %%
if __name__ == '__main__':
    from pprint import pprint
    serials = Thorlabs.ListDevicesAny()
    print('Serial number(s): ', end = '')
    pprint(serials)
    devinfo = Thorlabs.GetDeviceInfo(serials[0])
    print('Device Info: ', end = '')
    pprint(devinfo)
    print('Device Info(s): ', end = '')
    pprint(Thorlabs.GetDeviceInfo(serials))
    motor = Thorlabs.KST101(serials[0])
    print(motor.Open())
    print(motor.GetHardwareInfo())
    motor.Identify()
    del motor
# %%
