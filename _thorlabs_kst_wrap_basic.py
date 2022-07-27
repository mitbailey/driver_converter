
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
# %% TLI Functions
class TLIBase(NiceLib): # this would be the base of the Thorlabs namespace, which would provide static methods common across all Thorlabs sub APIs
    _info_ = load_lib('thorlabs_kst_')
    _ret_ = ret_errcode
    _prefix_ = 'TLI_'

    BuildDeviceList = Sig()
    GetDeviceListSize = Sig()
    # GetErrorString = Sig('in', 'buf', 'len', ret=ret_ignore) # ignore this example function
    GetDeviceListExt = Sig('in', 'in')
    GetDeviceInfo = Sig('in', 'in')
    GetDeviceListByTypeExt = Sig('in', 'in', 'in')
# %%
class TLI_KST(NiceLib): # This would be KST101, a subclass of Thorlabs
    _info_ = load_lib('thorlabs_kst_')
    _prefix_ = 'SCC_'

    Open = Sig('in')
    Close = Sig('in', ret = ret_ignore)
    CheckConnection = Sig('in')
    Identify = Sig('in', ret = ret_ignore)
    SetStageType = Sig('in', 'in')
    GetHardwareInfoBlock = Sig('in', 'in') # ser, TLI_HardwareInformation

# similarly, a TLI_KSG class can be defined for TLI Strain gauges

############## This should be the end of the Level 2 Wrapper ##################

############## Level 3 Wrapper (final stage) begin ############################

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
