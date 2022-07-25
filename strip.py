# Startup arguments.
PREFIX = 'SCC_'
IN_FILENAME = 'scc_header.txt'
IGNORE_N_CHARS = 17
OUT_FILENAME = 'scc_out.txt'

f = open(IN_FILENAME, 'r')
o = open(OUT_FILENAME, 'w')

o.write('_prefix_ = \'' + PREFIX + '\'\n\n')

i = 0
while True:
    i += 1
    line = f.readline()
    if not line:
        break

    # First, get rid of extraneous symbols.
    line = line.replace('\n', '')
    line = line.replace(';', '')
    print(line)

    # Next, ignore 'KCUBESTEPPER_API' tag.
    line = line[IGNORE_N_CHARS : len(line)]
    # line = line[line.find(PREFIX) : len(line)]
    print(line)

    # Next, find the return type.
    fn_ret = line[0 : line.find(' ')]
    print("Return type:   " + fn_ret)
    line = line[line.find(' ') : len(line)]
    line = line.strip()
    print(line)

    # Find the function name.
    fn_name = line[0 : line.find('(')]
    print("Function name: " + fn_name)

    # Remove prefix.
    fn_name = fn_name[len(PREFIX) : len(fn_name)]

    # Find the function's arguments.
    fn_args = line[line.find('(') + 1 : line.find(')')].split(',')
    if (len(fn_args) == 0):
        print("Function args: NO ARGUMENTS")
    else:
        print("Function args:")
        print(fn_args)

    # Write function data to file.
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
        if (fn_ret == 'void'):
            o.write(', ')
    if (fn_ret == 'void'):
        o.write('ret=ret_ignore')

    o.write(')\n')


    # star_i = 0
    # while True:
    #     star_i = fn_args.find('*')
    #     if (star_i == -1):
    #         break
    #     if (fn_args[star_i + 1] == '*'):
    #         print('out (' + str(star_i) + ')')
    #     else:
    #         print('in (' + str(star_i) + ')')
    #     fn_args = fn_args[star_i + 2 : len(fn_args)]


    print("\n")
print("Task completed successfully.")