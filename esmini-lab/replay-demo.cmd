@echo off
setlocal
cd /d "%~dp0"
set "ESMINI_CONFIG_FILE="
"runtime\esmini-demo\bin\replayer.exe" --file "runs\20260913T153713_529656Z_demo\simulation.dat" --res_path "runtime\esmini-demo\resources" --window 60 60 1280 720 --repeat --logfile_path "evidence\replay-demo.log"
if errorlevel 1 pause
endlocal
