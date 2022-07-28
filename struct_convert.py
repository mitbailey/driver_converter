# Parses a low-level driver C-header file looking for struct declarations.
# Converts only the struct declarations to mid-level driver compatible declarations.

# Startup arguments.
IN_FILENAME = 'input/tl.h'
OUT_FILENAME = 'tl_h_structs_out.py'

f = open(IN_FILENAME, 'r')
o = open('output/' + OUT_FILENAME, 'w')

o.write('# %% Imports\n')
o.write('from __future__ import annotations\n')
o.write('from nicelib import load_lib, NiceLib, Sig, NiceObject, RetHandler, ret_ignore\n')
o.write('from cffi import FFI\n')
o.write('from inspect import getmembers\n')
o.write('import warnings\n\n')

o.write('# %%\n')
o.write('package_ffi = FFI()\n\n')

o.write('# %% Converting struct to Dictionary\n')
o.write('def cdata_dict(cd, package_ffi: FFI):\n')
o.write('    if isinstance(cd, package_ffi.CData):\n')
o.write('        try:\n')
o.write('            return package_ffi.string(cd).decode(\'utf-8\')\n')
o.write('        except TypeError:\n')
o.write('            try:\n')
o.write('                return [cdata_dict(x, package_ffi) for x in cd]\n')
o.write('            except TypeError:\n')
o.write('                return {k: cdata_dict(v, package_ffi) for k, v in getmembers(cd)}\n')
o.write('    else:\n')
o.write('        return cd\n')

o.write('# %% Struct definitions.\n')

i = -1
pack = 0
within_struct = False
struct_name = ''
while True:
    i += 1
    line = f.readline()
    if not line:
        break

    # First, get rid of extraneous symbols.
    line = line.replace('\n', '')
    line = line.replace(';', '')
    line = line.strip()
    # print(line)

    if within_struct:
        # Detects some permutation of: } [SPACES] structName
        # Has trouble when the declared name differs from that on the closure. tagStructName is the only instance of this.
        if ((line.find('}') != -1) and ((line.find(struct_name) != -1) or line.find('tag' + struct_name))):
            # End of the struct.
            within_struct = False
            o.write('};\n')
            o.write('""", packed=' + str(pack) + ') # Defines struct, with packing.\n')
            # o.write('ser_buf = package_ffi.new(\'struct ' + struct_name + ' *\') # Creates memory for struct.\n')
            # o.write(struct_name + '_DICT = cdata_dict(ser_buf, package_ffi)\n\n')
            o.write('\n')
            print('- (END)')
        else:
            if (line == '{'):
                continue
            o.write('   ' + line + ';\n')
            print('- ' + line)
    else:
        # pack = 0
        # Skip until we get a struct.
        if (line.find('#pragma pack') != -1):
            # We found a #pragma pack(n) statement.
            pack_str = line[line.find('(') + 1 : line.find(')')]
            if pack_str == '':
                pack = 0
            else:
                pack = int(pack_str)
            print('Packing: ' + str(pack))

        # Indicates start of a struct.
        struct_begin = line.find('typedef struct')
        if (struct_begin != -1):
            within_struct = True
            struct_name = line[struct_begin + len('typedef struct') + 1 : len(line)]
            o.write('package_ffi.cdef("""\n')
            o.write('struct ' + struct_name + '\n')
            o.write('{\n')
            print(struct_name)

o.close()