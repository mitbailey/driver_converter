# Low-Level to Mid-Level Drivers Converter

- `struct_convert.py` will parse the original header, looking for structs. It has two startup arguments that can be set in the file: `IN_FILENAME` and `OUT_FILENAME`. The script will parse `IN_FILENAME` and produce a file called `OUT_FILENAME`.

- `fn_convert.py` is not yet implemented, but will work similarly to `struct_convert.py`.

- `strip.py` converts a list of functions to their `Sig(...)` versions. This script requires a list of functions in a file such as `tli_header.txt`.