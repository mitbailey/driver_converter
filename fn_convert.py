# Parses a low-level driver C-header file looking for function declarations.
# Converts only the function declarations to mid-level driver compatible declarations.

import re
# Create a function-matching regex.
fn_regex = re.compile(r'.+?\s+?.+?\s+?.+?\(.+?\);')
# fn_regex = re.compile(r'.(int|float|double|char).;')

# Startup arguments.
IN_FILENAME = 'tl.h'
OUT_FILENAME = 'tl_h_fn_out.py'

f = open(IN_FILENAME, 'r')
o = open(OUT_FILENAME, 'w')

i = 0
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
    # line = line.replace(';', '')
    line = line.strip()
    # print(line)

    if fn_regex.match(line) is not None:
        print(str(i) + ': ' + line)