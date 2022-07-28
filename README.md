# C to Python Drivers Converter

Converts a header, such as `tl.h`, to the format seen in `_thorlabs_kst_wrap_basic.py`.

- `struct_convert.py` will parse the original header, looking for structs. It has two startup arguments that can be set in the file: `IN_FILENAME` and `OUT_FILENAME`. The script will parse `IN_FILENAME` and produce a file called `OUT_FILENAME`.

- `fn_convert.py` will parse the original header, looking for functions. It has three startup arguments that can be set in the file: `IN_FILENAME`, `OUT_FILENAME`, and `AUTO_DETECT_PREFIX`. The script will parse `IN_FILENAME` and produce a file called `OUT_FILENAME`. The script, if `AUTO_DETECT_PREFIX` is set to `True`, will assume that everything ahead of the first underscore in a function name is a prefix, and will treat them appropriately. This script automatically corrects for function names formatted as either `fn_name(args)` or `fn_name (args)`.

- [DEPRECATED] `strip.py` converts a list of functions to their `Sig(...)` versions. This script requires a list of functions in a file such as `tli_header.txt`.