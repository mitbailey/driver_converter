# %%
from nicelib import build_lib, process
import os

header_info = {
    'win*': {
        'path': (
            r"{PROGRAMFILES}\Thorlabs\Kinesis", # path to thorlabs kinesis libs
            r"{PROGRAMFILES(X86)}\Thorlabs\Kinesis", # path to thorlabs kinesis libs
            os.getcwd() + '/header' # local path, this is where the sanitized header lives
        ),
        'header': 'tl.h' # sanitized header file. No need to strip cdecls and such, can be cleared using hooks.
        # However, any non trivial typedefs (such as structs) used in function I/O had to be explicitly defined here.
    },
}
lib_names = {'win*': 'Thorlabs.MotionControl.KCube.StepperMotor', # for the .lib, not sure if necessary
             'win*': 'Thorlabs.MotionControl.KCube.StepperMotor.dll'} # the actual DLL

def build():
    # this function builds the CFFI symbol table.
    # header_info: header_info dictionary containing paths etc
    # lib_names: .lib, .dll, .so...
    # '_thorlabs_kst_lib': Output name. Has to end with lib.
    # os.getcwd(): Current working directory of _build_kst.py, a fine place to save the CFFI symbol table.
    # ignore_system_headers: windows.h and such
    # token_hooks: deal with extern "C" {}, __declspec(), __cdecl etc
    # ast_hooks: deal with basic typedefs
    build_lib(header_info, lib_names, '_thorlabs_kst_lib', os.getcwd(), ignore_system_headers=True, token_hooks=(process.extern_c_hook, process.declspec_hook, process.cdecl_hook), ast_hooks=[process.add_typedef_hook])
    # this function will generate the _thorlabs_kst_lib.py file, which contains the CFFI symbol table.

if __name__ == '__main__':
    from instrumental.log import log_to_screen
    log_to_screen(fmt='%(message)s')
    build()