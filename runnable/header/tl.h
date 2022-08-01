#pragma once
#define KCUBESTEPPER_API
#include <OaIdl.h>
extern "C"
{
	typedef enum FT_Status : short
	{
		FT_OK = 0x00, /// <OK - no error.
		FT_InvalidHandle = 0x01, ///<Invalid handle.
		FT_DeviceNotFound = 0x02, ///<Device not found.
		FT_DeviceNotOpened = 0x03, ///<Device not opened.
		FT_IOError = 0x04, ///<I/O error.
		FT_InsufficientResources = 0x05, ///<Insufficient resources.
		FT_InvalidParameter = 0x06, ///<Invalid parameter.
		FT_DeviceNotPresent = 0x07, ///<Device not present.
		FT_IncorrectDevice = 0x08 ///<Incorrect device.
	 } FT_Status;
	typedef enum MOT_MotorTypes
	{
		MOT_NotMotor = 0,
		MOT_DCMotor = 1,
		MOT_StepperMotor = 2,
		MOT_BrushlessMotor = 3,
		MOT_CustomMotor = 100,
	} MOT_MotorTypes;
	typedef enum MOT_TravelModes : int
	{
		MOT_TravelModeUndefined,///<Undefined
		MOT_Linear = 0x01,///<Linear travel, default units are millimeters
		MOT_Rotational = 0x02,///<Rotational travel, default units are degrees
	} MOT_TravelModes;
	typedef enum MOT_TravelDirection : short
	{
		MOT_TravelDirectionUndefined,///<Undefined
		MOT_Forwards = 0x01,///<Move in a Forward direction
		MOT_Backwards = 0x02,///<Move in a Backward / Reverse direction
	} MOT_TravelDirection;
	typedef enum KST_Stages : short
	{
		ZST6 = 0x20, ///< ZST6.
		ZST13 = 0x21, ///< ZST13.
		ZST25 = 0x22, ///< ZST25.
		ZST206 = 0x30, ///< ZST206.
		ZST213 = 0x31, ///< ZST213.
		ZST225 = 0x32, ///< ZST225.
		ZFS206 = 0x40, ///< ZFS206.
		ZFS213 = 0x41, ///< ZFS213.
		ZFS225 = 0x42, ///< ZFS225.
        DRV013_25MM   = 0x50, ///< DRV013 13mm.
        DRV014_50MM   = 0x51, ///< DRV014 25mm.
		NR360 = 0x70, ///< NR360.
		PLS_X25MM = 0x72, ///< PLS_X.
		PLS_X25MM_HiRes = 0x73, ///< PLS_X HiRes.
		FW103 = 0x75, ///< FW103.
	} KST_Stages;
	typedef enum MOT_HomeLimitSwitchDirection : short
	{
		MOT_LimitSwitchDirectionUndefined,///<Undefined
		MOT_ReverseLimitSwitch = 0x01,///<Limit switch in forward direction
		MOT_ForwardLimitSwitch = 0x04,///<Limit switch in reverse direction
	} MOT_HomeLimitSwitchDirection;
	typedef enum MOT_DirectionSense : short
	{
		MOT_Normal = 0x00,///<Move / Jog direction is normal (clockwise).
		MOT_Reverse = 0x01,///<Move / Jog direction is reversed (anti clockwise).
	} MOT_DirectionSense;
	typedef enum MOT_JogModes : short
	{
		MOT_JogModeUndefined = 0x00,///<Undefined
		MOT_Continuous = 0x01,///<Continuous jogging
		MOT_SingleStep = 0x02,///<Jog 1 step at a time
	} MOT_JogModes;
	typedef enum MOT_StopModes : short
	{
		MOT_StopModeUndefined = 0x00,///<Undefined
		MOT_Immediate = 0x01,///<Stops immediate
		MOT_Profiled = 0x02,///<Stops using a velocity profile
	} MOT_StopModes;
	typedef enum MOT_LimitSwitchModes : WORD
	{
		MOT_LimitSwitchModeUndefined = 0x00,///<Undefined
		MOT_LimitSwitchIgnoreSwitch=0x01,///<Ignore limit switch
		MOT_LimitSwitchMakeOnContact=0x02,///<Switch makes on contact
		MOT_LimitSwitchBreakOnContact=0x03,///<Switch breaks on contact
		MOT_LimitSwitchMakeOnHome=0x04,///<Switch makes on contact when homing
		MOT_LimitSwitchBreakOnHome=0x05,///<Switch breaks on contact when homing
		MOT_PMD_Reserved=0x06,///<Reserved for PMD brushless servo controllers
		MOT_LimitSwitchIgnoreSwitchSwapped = 0x81,///<Ignore limit switch (swapped)
		MOT_LimitSwitchMakeOnContactSwapped = 0x82,///<Switch makes on contact (swapped)
		MOT_LimitSwitchBreakOnContactSwapped = 0x83,///<Switch breaks on contact (swapped)
		MOT_LimitSwitchMakeOnHomeSwapped = 0x84,///<Switch makes on contact when homing (swapped)
		MOT_LimitSwitchBreakOnHomeSwapped = 0x85,///<Switch breaks on contact when homing (swapped)
	} MOT_LimitSwitchModes;
	typedef enum MOT_LimitSwitchSWModes : WORD
	{
		MOT_LimitSwitchSWModeUndefined = 0x00,///<Undefined
		MOT_LimitSwitchIgnored=0x01,///<Ignore limit switch
		MOT_LimitSwitchStopImmediate=0x02,///<Stop immediately when hitting limit switch
		MOT_LimitSwitchStopProfiled=0x03,///<Stop profiled when hitting limit switch
		MOT_LimitSwitchIgnored_Rotational=0x81,///<Ignore limit switch (rotational stage)
		MOT_LimitSwitchStopImmediate_Rotational=0x82,///<Stop immediately when hitting limit switch (rotational stage)
		MOT_LimitSwitchStopProfiled_Rotational=0x83,///<Stop profiled when hitting limit switch (rotational stage)
	} MOT_LimitSwitchSWModes;
	typedef enum MOT_LimitsSoftwareApproachPolicy : __int16
	{
		DisallowIllegalMoves = 0,///<Disable any move outside of the current travel range of the stage
		AllowPartialMoves,///<Truncate moves to within the current travel range of the stage.
		AllowAllMoves,///<Allow all moves, regardless of whether they are within the current travel range of the stage.
	} MOT_LimitsSoftwareApproachPolicy;
	typedef enum KMOT_WheelDirectionSense : __int16
	{
		KMOT_WM_Positive = 0x01,///< Move at constant velocity
		KMOT_WM_Negative = 0x02,///< Phase B
	} KMOT_WheelDirectionSense;
	typedef enum KMOT_WheelMode : __int16
	{
		KMOT_WM_Velocity = 0x01,///< Move at constant velocity
		KMOT_WM_Jog = 0x02,///< Phase B
		KMOT_WM_MoveAbsolute = 0x03,///< Phase A and B
	} KMOT_WheelMode;
	typedef enum KMOT_TriggerPortMode : __int16
	{
		KMOT_TrigDisabled = 0x00,///< Trigger Disabled
		KMOT_TrigIn_GPI = 0x01,///< General purpose logic input (<see cref="SCC_GetStatusBits(const char * serialNo)"> GetStatusBits</see>)
		KMOT_TrigIn_RelativeMove = 0x02,///< Move relative using relative move parameters
		KMOT_TrigIn_AbsoluteMove = 0x03,///< Move absolute using absolute move parameters
		KMOT_TrigIn_Home = 0x04,///< Perform a Home action
		KMOT_TrigIn_Stop = 0x05,///< Perform a Stop Immediate action
		KMOT_TrigOut_GPO = 0x0A,///< General purpose output (<see cref="SCC_SetDigitalOutputs(const char * serialNo, byte outputBits)"> SetDigitalOutputs</see>)
		KMOT_TrigOut_InMotion = 0x0B,///< Set when device moving
		KMOT_TrigOut_AtMaxVelocity = 0x0C,///< Set when at max velocity
		KMOT_TrigOut_AtPositionSteps = 0x0D,///< Set when at predefine position steps,<br />Set using wTrigStartPos, wTrigInterval, wTrigNumPulses,wTrigPulseWidth
		KMOT_TrigOut_Synch = 0x0E,///< TBD ?
	} KMOT_TriggerPortMode;
	typedef enum KMOT_TriggerPortPolarity : __int16
	{
		KMOT_TrigPolarityHigh = 0x01,///< Trigger Polarity high
		KMOT_TrigPolarityLow = 0x02,///< Trigger Polarity Low
	} KMOT_TriggerPortPolarity;
	typedef enum MOT_PID_LoopMode : WORD
	{
		MOT_PIDLoopModeDisabled = 0x00,///<Disabled or Undefined
		MOT_PIDOpenLoopMode = 0x01,///<Encoder is in open loop mode
		MOT_PIDClosedLoopMode = 0x02,///<Encoder is in closed loop mode
	} MOT_PID_LoopMode;
	typedef enum MOT_MovementModes
	{
		LinearRange = 0x00,///< Fixed Angular Range defined by MinPosition and MaxPosition
		RotationalUnlimited = 0x01,///< Unlimited angle
		RotationalWrapping = 0x02,///< Angular Range 0 to 360 with wrap around
	} MOT_MovementModes;
	typedef enum MOT_MovementDirections
	{
		Quickest = 0x00,///< Uses the shortest travel between two angles
		Forwards = 0x01,///< Only rotate in a forward direction
		Reverse = 0x02,///< Only rotate in a backward direction
	} MOT_MovementDirections;
	#pragma pack(1)
	typedef struct TLI_DeviceInfo
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
	} TLI_DeviceInfo;
	typedef struct TLI_HardwareInformation
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
	} TLI_HardwareInformation;
	typedef struct MOT_VelocityParameters
	{
		int minVelocity;
		int acceleration;
		int maxVelocity;
	} MOT_VelocityParameters;
	typedef struct MOT_JogParameters
	{
		MOT_JogModes mode;
		unsigned int stepSize;
		MOT_VelocityParameters velParams;
		MOT_StopModes stopMode;
	} MOT_JogParameters;
	typedef struct MOT_HomingParameters
	{
		MOT_TravelDirection direction;
		MOT_HomeLimitSwitchDirection limitSwitch;
		unsigned int velocity;
		unsigned int offsetDistance;
	} MOT_HomingParameters;
	typedef struct MOT_LimitSwitchParameters
	{
		MOT_LimitSwitchModes clockwiseHardwareLimit;
		MOT_LimitSwitchModes anticlockwiseHardwareLimit;
		DWORD clockwisePosition;
		DWORD anticlockwisePosition;
		MOT_LimitSwitchSWModes softLimitMode;
	} MOT_LimitSwitchParameters;
	typedef struct MOT_PowerParameters
	{
		WORD restPercentage;
		WORD movePercentage;
	} MOT_PowerParameters;
	typedef struct KMOT_MMIParams
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
	} KMOT_MMIParams;
	typedef struct KMOT_TriggerConfig
	{
		KMOT_TriggerPortMode Trigger1Mode;
		KMOT_TriggerPortPolarity Trigger1Polarity;
		KMOT_TriggerPortMode Trigger2Mode;
		KMOT_TriggerPortPolarity Trigger2Polarity;
		__int16 reserved[6];
	} KMOT_TriggerConfig;
	typedef struct KMOT_TriggerParams
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
	} KMOT_TriggerParams;
	typedef struct MOT_PIDLoopEncoderParams
	{
		MOT_PID_LoopMode loopMode;
		int proportionalGain;
		int integralGain;
		int differentialGain;
		int PIDOutputLimit;
		int PIDTolerance;
	} MOT_PIDLoopEncoderParams;

typedef unsigned long ULONG;
typedef long LONG;
typedef unsigned short USHORT;
typedef void* PVOID;
typedef unsigned char byte;

	typedef struct tagSAFEARRAYBOUND
    {
    ULONG cElements;
    LONG lLbound;
    } 	SAFEARRAYBOUND;

	typedef struct tagSAFEARRAY
    {
    USHORT cDims;
    USHORT fFeatures;
    ULONG cbElements;
    ULONG cLocks;
    PVOID pvData;
    SAFEARRAYBOUND rgsabound[ 1 ];
    } 	SAFEARRAY;


	#pragma pack()
	KCUBESTEPPER_API short   TLI_BuildDeviceList(void);
	KCUBESTEPPER_API short   TLI_GetDeviceListSize();
	KCUBESTEPPER_API short   TLI_GetDeviceList(SAFEARRAY** stringsReceiver);
	KCUBESTEPPER_API short   TLI_GetDeviceListByType(SAFEARRAY** stringsReceiver, int typeID);
	KCUBESTEPPER_API short   TLI_GetDeviceListByTypes(SAFEARRAY** stringsReceiver, int * typeIDs, int length);
	KCUBESTEPPER_API short   TLI_GetDeviceListExt(char *receiveBuffer, DWORD sizeOfBuffer);
	KCUBESTEPPER_API short   TLI_GetDeviceListByTypeExt(char *receiveBuffer, DWORD sizeOfBuffer, int typeID);
	KCUBESTEPPER_API short   TLI_GetDeviceListByTypesExt(char *receiveBuffer, DWORD sizeOfBuffer, int * typeIDs, int length);
	KCUBESTEPPER_API short   TLI_GetDeviceInfo(char const * serialNo, TLI_DeviceInfo *info);
	KCUBESTEPPER_API void   TLI_InitializeSimulations();
	KCUBESTEPPER_API void   TLI_UninitializeSimulations();
	KCUBESTEPPER_API short   SCC_Open(char const * serialNo);
	KCUBESTEPPER_API void   SCC_Close(char const * serialNo);
	KCUBESTEPPER_API bool   SCC_CheckConnection(char const * serialNo);
	KCUBESTEPPER_API void   SCC_Identify(char const * serialNo);
	KCUBESTEPPER_API short   SCC_SetStageType(char const * serialNo, KST_Stages stageId);
	KCUBESTEPPER_API short   SCC_GetHardwareInfo(char const * serialNo, char * modelNo, DWORD sizeOfModelNo, WORD * type, WORD * numChannels,
		char * notes, DWORD sizeOfNotes, DWORD * firmwareVersion, WORD * hardwareVersion, WORD * modificationState);
	KCUBESTEPPER_API short   SCC_GetHardwareInfoBlock(char const * serialNo, TLI_HardwareInformation *hardwareInfo);
	KCUBESTEPPER_API DWORD   SCC_GetSoftwareVersion(char const * serialNo);
	KCUBESTEPPER_API void   SCC_SetCalibrationFile(char const * serialNo, char const *filename, bool enabled);
	KCUBESTEPPER_API bool   SCC_IsCalibrationActive(char const * serialNo);
	KCUBESTEPPER_API bool   SCC_GetCalibrationFile(char const * serialNo, char * filename, short sizeOfBuffer);
	KCUBESTEPPER_API char   SCC_GetHubBay(char const * serialNo);
	KCUBESTEPPER_API bool   SCC_LoadSettings(char const * serialNo);
	KCUBESTEPPER_API bool   SCC_LoadNamedSettings(char const * serialNo, char const *settingsName);
	KCUBESTEPPER_API bool   SCC_PersistSettings(char const * serialNo);
	KCUBESTEPPER_API short   SCC_DisableChannel(char const * serialNo);
	KCUBESTEPPER_API short   SCC_EnableChannel(char const * serialNo);
	KCUBESTEPPER_API bool   SCC_CanDeviceLockFrontPanel(char const * serialNo);
	KCUBESTEPPER_API bool    SCC_GetFrontPanelLocked(char const * serialNo);
	KCUBESTEPPER_API short    SCC_RequestFrontPanelLocked(char const * serialNo);
	KCUBESTEPPER_API short    SCC_SetFrontPanelLock(char const * serialNo, bool locked);
	KCUBESTEPPER_API int   SCC_GetNumberPositions(char const * serialNo);
	KCUBESTEPPER_API short   SCC_MoveToPosition(char const * serialNo, int index);
	KCUBESTEPPER_API int   SCC_GetPosition(char const * serialNo);
	KCUBESTEPPER_API bool   SCC_CanHome(char const * serialNo);
	KCUBESTEPPER_API bool   SCC_NeedsHoming(char const * serialNo);
	KCUBESTEPPER_API bool   SCC_CanMoveWithoutHomingFirst(char const * serialNo);
	KCUBESTEPPER_API short   SCC_Home(char const * serialNo);
	KCUBESTEPPER_API void   SCC_ClearMessageQueue(char const * serialNo);
	KCUBESTEPPER_API void   SCC_RegisterMessageCallback(char const * serialNo, void(*functionPointer)());
	KCUBESTEPPER_API int   SCC_MessageQueueSize(char const * serialNo);
	KCUBESTEPPER_API bool   SCC_GetNextMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData);
	KCUBESTEPPER_API bool   SCC_WaitForMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData);
	KCUBESTEPPER_API short   SCC_RequestHomingParams(char const * serialNo);
	KCUBESTEPPER_API unsigned int   SCC_GetHomingVelocity(char const * serialNo);
	KCUBESTEPPER_API short   SCC_SetHomingVelocity(char const * serialNo, unsigned int velocity);
	KCUBESTEPPER_API short   SCC_MoveRelative(char const * serialNo, int displacement);
	KCUBESTEPPER_API short   SCC_RequestJogParams(const char * serialNo);
	KCUBESTEPPER_API short   SCC_GetJogMode(char const * serialNo, MOT_JogModes * mode, MOT_StopModes * stopMode);
	KCUBESTEPPER_API short   SCC_SetJogMode(char const * serialNo, MOT_JogModes mode, MOT_StopModes stopMode);
	KCUBESTEPPER_API unsigned int   SCC_GetJogStepSize(char const * serialNo);
	KCUBESTEPPER_API short   SCC_SetJogStepSize(char const * serialNo, unsigned int stepSize);
	KCUBESTEPPER_API short   SCC_GetJogVelParams(char const * serialNo, int * acceleration, int * maxVelocity);
	KCUBESTEPPER_API short   SCC_SetJogVelParams(char const * serialNo, int acceleration, int maxVelocity);
	KCUBESTEPPER_API short   SCC_MoveJog(char const * serialNo, MOT_TravelDirection jogDirection);
	KCUBESTEPPER_API short   SCC_RequestVelParams(char const * serialNo);
	KCUBESTEPPER_API short   SCC_GetVelParams(char const * serialNo, int * acceleration, int * maxVelocity);
	KCUBESTEPPER_API short   SCC_SetVelParams(char const * serialNo, int acceleration, int maxVelocity);
	KCUBESTEPPER_API short   SCC_MoveAtVelocity(char const * serialNo, MOT_TravelDirection direction);
	KCUBESTEPPER_API void   SCC_SetDirection(char const * serialNo, bool reverse);
	KCUBESTEPPER_API short   SCC_StopImmediate(char const * serialNo);
	KCUBESTEPPER_API short   SCC_StopProfiled(char const * serialNo);
	KCUBESTEPPER_API short   SCC_RequestBacklash(char const * serialNo);
	KCUBESTEPPER_API long   SCC_GetBacklash(char const * serialNo);
	KCUBESTEPPER_API short   SCC_SetBacklash(char const * serialNo, long distance);
	KCUBESTEPPER_API long   SCC_GetPositionCounter(char const * serialNo);
	KCUBESTEPPER_API short   SCC_SetPositionCounter(char const * serialNo, long count);
	KCUBESTEPPER_API short   SCC_RequestEncoderCounter(char const * serialNo);
	KCUBESTEPPER_API long   SCC_GetEncoderCounter(char const * serialNo);
	KCUBESTEPPER_API short   SCC_SetEncoderCounter(char const * serialNo, long count);
	KCUBESTEPPER_API short   SCC_RequestLimitSwitchParams(char const * serialNo);
	KCUBESTEPPER_API short   SCC_GetLimitSwitchParams(char const * serialNo, MOT_LimitSwitchModes * clockwiseHardwareLimit, MOT_LimitSwitchModes * anticlockwiseHardwareLimit, unsigned int * clockwisePosition, unsigned int * anticlockwisePosition, MOT_LimitSwitchSWModes * softLimitMode);
	KCUBESTEPPER_API short   SCC_SetLimitSwitchParams(char const * serialNo, MOT_LimitSwitchModes clockwiseHardwareLimit, MOT_LimitSwitchModes anticlockwiseHardwareLimit, unsigned int clockwisePosition, unsigned int anticlockwisePosition, MOT_LimitSwitchSWModes softLimitMode);
	KCUBESTEPPER_API MOT_LimitsSoftwareApproachPolicy   SCC_GetSoftLimitMode(char const * serialNo);
	KCUBESTEPPER_API void   SCC_SetLimitsSoftwareApproachPolicy(char const * serialNo, MOT_LimitsSoftwareApproachPolicy limitsSoftwareApproachPolicy);
	KCUBESTEPPER_API short   SCC_RequestMMIParams(char const * serialNo);
	KCUBESTEPPER_API  short   SCC_GetMMIParamsExt(char const * serialNo, KMOT_WheelMode *wheelMode, __int32 *wheelMaxVelocity, __int32 *wheelAcceleration, KMOT_WheelDirectionSense *directionSense,
		__int32 *presetPosition1, __int32 *presetPosition2, __int16 *displayIntensity, __int16 *displayTimeout, __int16 *displayDimIntensity);
	KCUBESTEPPER_API  short   SCC_GetMMIParams(char const * serialNo, KMOT_WheelMode *wheelMode, __int32 *wheelMaxVelocity, __int32 *wheelAcceleration, KMOT_WheelDirectionSense *directionSense,
		__int32 *presetPosition1, __int32 *presetPosition2, __int16 *displayIntensity);
	KCUBESTEPPER_API short   SCC_SetMMIParamsExt(char const * serialNo, KMOT_WheelMode wheelMode, __int32 wheelMaxVelocity, __int32 wheelAcceleration, KMOT_WheelDirectionSense directionSense,
		__int32 presetPosition1, __int32 presetPosition2, __int16 displayIntensity, __int16 displayTimeout, __int16 displayDimIntensity);
	KCUBESTEPPER_API short   SCC_SetMMIParams(char const * serialNo, KMOT_WheelMode wheelMode, __int32 wheelMaxVelocity, __int32 wheelAcceleration, KMOT_WheelDirectionSense directionSense,
		__int32 presetPosition1, __int32 presetPosition2, __int16 displayIntensity);
	KCUBESTEPPER_API short   SCC_RequestTriggerConfigParams(char const * serialNo);
	KCUBESTEPPER_API  short   SCC_GetTriggerConfigParams(char const * serialNo, KMOT_TriggerPortMode *trigger1Mode, KMOT_TriggerPortPolarity *trigger1Polarity, KMOT_TriggerPortMode *trigger2Mode, KMOT_TriggerPortPolarity *trigger2Polarity);
	KCUBESTEPPER_API short   SCC_SetTriggerConfigParams(char const * serialNo, KMOT_TriggerPortMode trigger1Mode, KMOT_TriggerPortPolarity trigger1Polarity, KMOT_TriggerPortMode trigger2Mode, KMOT_TriggerPortPolarity trigger2Polarity);
	KCUBESTEPPER_API  short   SCC_RequestPosTriggerParams(char const * serialNo);
	KCUBESTEPPER_API  short   SCC_GetTriggerParamsParams(char const * serialNo, __int32 *triggerStartPositionFwd, __int32 *triggerIntervalFwd, __int32 *triggerPulseCountFwd,
		__int32 *triggerStartPositionRev, __int32 *triggerIntervalRev, __int32 *triggerPulseCountRev,
		__int32 *triggerPulseWidth, __int32 *cycleCount);
	KCUBESTEPPER_API short   SCC_SetTriggerParamsParams(char const * serialNo, __int32 triggerStartPositionFwd, __int32 triggerIntervalFwd, __int32 triggerPulseCountFwd,
		__int32 triggerStartPositionRev, __int32 triggerIntervalRev, __int32 triggerPulseCountRev,
		__int32 triggerPulseWidth, __int32 cycleCount);
	KCUBESTEPPER_API short   SCC_GetMMIParamsBlock(char const * serialNo, KMOT_MMIParams *mmiParams);
	KCUBESTEPPER_API short   SCC_SetMMIParamsBlock(char const * serialNo, KMOT_MMIParams *mmiParams);
	KCUBESTEPPER_API short   SCC_GetTriggerConfigParamsBlock(char const * serialNo, KMOT_TriggerConfig *triggerConfigParams);
	KCUBESTEPPER_API short   SCC_SetTriggerConfigParamsBlock(char const * serialNo, KMOT_TriggerConfig *triggerConfigParams);
	KCUBESTEPPER_API short   SCC_GetTriggerParamsParamsBlock(char const * serialNo, KMOT_TriggerParams *triggerParamsParams);
	KCUBESTEPPER_API short   SCC_SetTriggerParamsParamsBlock(char const * serialNo, KMOT_TriggerParams *triggerParamsParams);
	KCUBESTEPPER_API short   SCC_GetVelParamsBlock(const char * serialNo, MOT_VelocityParameters  *velocityParams);
	KCUBESTEPPER_API short   SCC_SetVelParamsBlock(const char * serialNo, MOT_VelocityParameters *velocityParams);
	KCUBESTEPPER_API short   SCC_RequestMoveAbsolutePosition(char const * serialNo);
	KCUBESTEPPER_API short   SCC_SetMoveAbsolutePosition(const char * serialNo, int position);
	KCUBESTEPPER_API int   SCC_GetMoveAbsolutePosition(const char * serialNo);
	KCUBESTEPPER_API short   SCC_MoveAbsolute(const char * serialNo);
	KCUBESTEPPER_API short   SCC_RequestMoveRelativeDistance(char const * serialNo);
	KCUBESTEPPER_API short   SCC_SetMoveRelativeDistance(const char * serialNo, int distance);
	KCUBESTEPPER_API int   SCC_GetMoveRelativeDistance(const char * serialNo);
	KCUBESTEPPER_API short   SCC_MoveRelativeDistance(const char * serialNo);
	KCUBESTEPPER_API short   SCC_GetHomingParamsBlock(const char * serialNo, MOT_HomingParameters *homingParams);
	KCUBESTEPPER_API short   SCC_SetHomingParamsBlock(const char * serialNo, MOT_HomingParameters *homingParams);
	KCUBESTEPPER_API short   SCC_GetJogParamsBlock(const char * serialNo, MOT_JogParameters *jogParams);
	KCUBESTEPPER_API short   SCC_SetJogParamsBlock(const char * serialNo, MOT_JogParameters *jogParams);
	KCUBESTEPPER_API short   SCC_GetLimitSwitchParamsBlock(const char * serialNo, MOT_LimitSwitchParameters *limitSwitchParams);
	KCUBESTEPPER_API short   SCC_SetLimitSwitchParamsBlock(const char * serialNo, MOT_LimitSwitchParameters *limitSwitchParams);
	KCUBESTEPPER_API short   SCC_RequestPowerParams(char const * serialNo);
	KCUBESTEPPER_API short   SCC_GetPowerParams(const char * serialNo, MOT_PowerParameters *powerParams);
	KCUBESTEPPER_API short   SCC_SetPowerParams(const char * serialNo, MOT_PowerParameters *powerParams);
	KCUBESTEPPER_API short   SCC_RequestBowIndex(const char * serialNo);
	KCUBESTEPPER_API short   SCC_GetBowIndex(const char * serialNo);
	KCUBESTEPPER_API short   SCC_SetBowIndex(const char * serialNo, short bowIndex);
	KCUBESTEPPER_API bool   SCC_UsesPIDLoopEncoding(const char * serialNo);
	KCUBESTEPPER_API short   SCC_SetPIDLoopEncoderParams(const char * serialNo, MOT_PIDLoopEncoderParams * params);
	KCUBESTEPPER_API short   SCC_SetPIDLoopEncoderCoeff(const char * serialNo, double coeff);
	KCUBESTEPPER_API short   SCC_RequestPIDLoopEncoderParams(const char * serialNo);
	KCUBESTEPPER_API short   SCC_GetPIDLoopEncoderParams(const char * serialNo, MOT_PIDLoopEncoderParams * params);
	KCUBESTEPPER_API double   SCC_GetPIDLoopEncoderCoeff(const char * serialNo);
	KCUBESTEPPER_API short   SCC_SuspendMoveMessages(char const * serialNo);
	KCUBESTEPPER_API short   SCC_ResumeMoveMessages(char const * serialNo);
	KCUBESTEPPER_API short   SCC_RequestPosition(char const * serialNo);
	KCUBESTEPPER_API short   SCC_RequestStatusBits(char const * serialNo);
	KCUBESTEPPER_API DWORD   SCC_GetStatusBits(char const * serialNo);
	KCUBESTEPPER_API bool   SCC_StartPolling(char const * serialNo, int milliseconds);
	KCUBESTEPPER_API long   SCC_PollingDuration(char const * serialNo);
	KCUBESTEPPER_API void   SCC_StopPolling(char const * serialNo);
	KCUBESTEPPER_API bool   SCC_TimeSinceLastMsgReceived(char const * serialNo, __int64 &lastUpdateTimeMS);
	KCUBESTEPPER_API void   SCC_EnableLastMsgTimer(char const * serialNo, bool enable, __int32 lastMsgTimeout);
	KCUBESTEPPER_API bool   SCC_HasLastMsgTimerOverrun(char const * serialNo);
	KCUBESTEPPER_API short   SCC_RequestSettings(char const * serialNo);
	KCUBESTEPPER_API int   SCC_GetStageAxisMinPos(char const * serialNo);
	KCUBESTEPPER_API int   SCC_GetStageAxisMaxPos(char const * serialNo);
	KCUBESTEPPER_API short   SCC_SetStageAxisLimits(char const * serialNo, int minPosition, int maxPosition);
	KCUBESTEPPER_API short   SCC_SetMotorTravelMode(char const * serialNo, MOT_TravelModes travelMode);
	KCUBESTEPPER_API MOT_TravelModes   SCC_GetMotorTravelMode(char const * serialNo);
	KCUBESTEPPER_API short   SCC_SetMotorParams(char const * serialNo, long stepsPerRev, long gearBoxRatio, float pitch);
	KCUBESTEPPER_API short   SCC_GetMotorParams(char const * serialNo, long *stepsPerRev, long *gearBoxRatio, float *pitch);
	KCUBESTEPPER_API short   SCC_SetMotorParamsExt(char const * serialNo, double stepsPerRev, double gearBoxRatio, double pitch);
	KCUBESTEPPER_API short   SCC_GetMotorParamsExt(char const * serialNo, double *stepsPerRev, double *gearBoxRatio, double *pitch);
	KCUBESTEPPER_API short   SCC_SetMotorVelocityLimits(char const * serialNo, double maxVelocity, double maxAcceleration);
	KCUBESTEPPER_API short   SCC_GetMotorVelocityLimits(char const * serialNo, double *maxVelocity, double *maxAcceleration);
	KCUBESTEPPER_API short   SCC_ResetRotationModes(char const * serialNo);
	KCUBESTEPPER_API short   SCC_SetRotationModes(char const * serialNo, MOT_MovementModes mode, MOT_MovementDirections direction);
	KCUBESTEPPER_API short   SCC_SetMotorTravelLimits(char const * serialNo, double minPosition, double maxPosition);
	KCUBESTEPPER_API short   SCC_GetMotorTravelLimits(char const * serialNo, double *minPosition, double *maxPosition);
	KCUBESTEPPER_API short   SCC_RequestDigitalOutputs(char const * serialNo);
	KCUBESTEPPER_API byte   SCC_GetDigitalOutputs(char const * serialNo);
	KCUBESTEPPER_API short   SCC_SetDigitalOutputs(char const * serialNo, byte outputsBits);
	KCUBESTEPPER_API short   SCC_GetRealValueFromDeviceUnit(char const * serialNo, int device_unit, double *real_unit, int unitType);
	KCUBESTEPPER_API short   SCC_GetDeviceUnitFromRealValue(char const * serialNo, double real_unit, int *device_unit, int unitType);
}
