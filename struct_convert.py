# Parses a low-level driver C-header file looking for struct declarations.
# Converts only the struct declarations to mid-level driver compatible declarations.

# Startup arguments.
IN_FILENAME = 'tl.h'
OUT_FILENAME = 'tl_h_structs_out.txt'

f = open(IN_FILENAME, 'r')
o = open(OUT_FILENAME, 'w')

i = -1
while True:
    i += 1
    line = f.readline()
    if not line:
        break

    # First, get rid of extraneous symbols.
    line = line.replace('\n', '')
    line = line.replace(';', '')
    # print(line)

    pack = 0
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
    # if (line.find('typedef struct'))