#!/bin/zsh
# Author: Zhuo Ma
# Launch the original-trajectory replay workflow; paths are resolved by paths.py.
set -e
cd -- "${0:A:h}"
/usr/bin/python3 run_scenarios.py --view 1549178
