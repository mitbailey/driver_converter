# Parses a low-level driver C-header file looking for function declarations.
# Converts only the function declarations to mid-level driver compatible declarations.

import re
# Create a function-matching regex.
fn_regex = re.compile(r'.+?\s+?.+?\s+?.+?\(.+?\);')
# fn_regex = re.compile(r'.(int|float|double|char).;')

# Startup arguments.
IN_FILENAME = 'input/tl.h'
OUT_FILENAME = 'tl_h_fn_out.py'
AUTO_DETECT_PREFIX = True

f = open(IN_FILENAME, 'r')
outfiles = []
if AUTO_DETECT_PREFIX == False:
    o = open('output/' + OUT_FILENAME, 'w')
else:
    o = None

prefixes = []
opened_output_files = []

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
        # print(str(i) + ': ' + line)
        print("LINE " + str(i) + ': ')

        # Figure out the function's name.
        fn_split = line.split(' ')
        fn_split = [section.strip() for section in fn_split]
        # print(fn_split)
        j = 0
        while True:
            if j >= len(fn_split):
                j = -1
                break
            retval = fn_split[j].find('(')
            # print("RETVAL: " + str(retval))
            if retval == -1:
                # No '(' found, keep looking.
                j += 1
                continue
            elif retval == 0:
                # '(' found, but its the first char in the string. Function format must be: name (args), so use previous
                # string entry in the list.
                j -= 1
                break
            else:
                break

        if j < 0:
            print("Failed to parse function on line " + str(i) + "!")
            exit()

        if fn_split[j].find('(') != -1:
            fn_name = fn_split[j][0 : fn_split[j].find('(')]
        else:
            fn_name = fn_split[j]
        fn_ret = ''
        k = 0
        while True:
            if k == j:
                break
            fn_ret = fn_ret + fn_split[k] + ' '
            k += 1
        print(fn_ret)

        print("Function name:")
        print(fn_name)

        fn_prefix = fn_name.split('_')[0]
        print(fn_prefix)

        if AUTO_DETECT_PREFIX:
            fn_prefixed_outfilename = fn_prefix + '_' + OUT_FILENAME
            print(fn_prefixed_outfilename)
            if fn_prefix not in prefixes:
                # File not yet opened.
                outfiles.append((fn_prefixed_outfilename, open('output/' + fn_prefixed_outfilename, 'w')))
                prefixes.append(fn_prefix)
            o = [file for (name, file) in outfiles if name == fn_prefixed_outfilename][0]
            print(o)

        # Remove prefix from function's name.
        fn_name = fn_name[len(fn_prefix) + 1: len(fn_name)]
        
        # Find the function's arguments.
        fn_args = line[line.find('(') + 1 : line.find(')')].split(',')
        fn_args = [arg.strip() for arg in fn_args]
        if (len(fn_args) == 0):
            print("Function args: NO ARGUMENTS")
        else:
            print("Function args:")
            print(fn_args)

        # Write data for this function to file.
        o.write(fn_name + ' = Sig(')

        # Determine args as 'in' or 'out.'
        # If there are more than 0 arguments.
        if fn_args[0] != '':
            first = True
            for arg in fn_args:
                print(arg)
                if (arg.find('**') != -1):
                    print('out')
                    if first:
                        first = False
                    else:
                        o.write(', ')
                    o.write('\'out\'')
                else:
                    if first:
                        first = False
                    else:
                        o.write(', ')
                    print('in')
                    o.write('\'in\'')
            if (fn_ret.find(' void') != -1):
                o.write(', ')
        if (fn_ret.find(' void') != -1):
            o.write('ret=ret_ignore')

        o.write(')\n')


        print('\n')

for (name, file) in outfiles:
    file.close()
