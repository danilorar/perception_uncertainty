@echo off
setlocal
cd /d "%~dp0"
set "ESMINI_CONFIG_FILE="
"runtime\esmini-demo\bin\replayer.exe" --file "runs\20260913T153722_252575Z_ncap\simulation_1_of_1.dat" --res_path "runtime\esmini-demo\resources" --path "sources\vectorgrp-OSC-NCAP-scenarios-15365d1\OpenDRIVE\NCAP" --window 60 60 1280 720 --repeat --logfile_path "evidence\replay-ncap.log"
if errorlevel 1 pause
endlocal
