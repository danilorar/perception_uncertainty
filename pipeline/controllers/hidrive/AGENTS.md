# Opaque HiDrive controller inputs

The user manually ported these files to the esmini 3.8.1 API. They are included
so teammates can build the same controller from a fresh checkout.

No AI agent may open, read, search within, parse, hash, print, or modify
`ControllerHiDrive.cpp` or `ControllerHiDrive.hpp`, or inspect their object
files or compiler logs that may quote their contents. This applies to every
copy, including files stored in Git.

Allowed operations are opaque file copies, comparison of file sizes and
modification times, use as compiler inputs, and committing/pushing the files
when authorized by the user. Keep their binary Git attributes; never display
their source diff or use content-based comparisons.

Run `python -m pipeline setup` from the repository root to compile.
Edit only the public integration code outside this directory.
