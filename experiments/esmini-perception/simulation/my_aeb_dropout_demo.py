#!/usr/bin/env python3
"""Run object-detection dropout with optional external AEB.

Author: Zhuo Ma
Entry point only: aeb_runner.py contains the annotated simulation loop,
esmini_api.py the native sensor calls, sensor_model.py the dropout model,
and aeb_controller.py the AEB decision and ego trajectory integration.
Dropout defaults to 0.1; use --drop-prob 0 for the no-dropout reference.
"""
from aeb_runner import main


if __name__ == "__main__":
    main(ideal=False)
