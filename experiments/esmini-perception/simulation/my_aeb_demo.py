#!/usr/bin/env python3
"""Run the ideal object sensor with optional external AEB.

Author: Zhuo Ma
Entry point only: aeb_runner.py contains the annotated simulation loop,
esmini_api.py the native sensor calls, sensor_model.py the dropout model,
and aeb_controller.py the AEB decision and ego trajectory integration.
This entry always uses dropout probability 0.0. AEB defaults to off.
"""
from aeb_runner import main


if __name__ == "__main__":
    main(ideal=True)
