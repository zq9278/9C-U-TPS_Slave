@echo off
setlocal
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" goto install
py -3 -m venv .venv
if not errorlevel 1 goto install
python -m venv .venv
if errorlevel 1 (
    echo Install Python 3.10 or newer, then run setup.bat again.
    pause
    exit /b 1
)
:install
".venv\Scripts\python.exe" -m pip install -r requirements.txt
if errorlevel 1 (
    echo Dependency installation failed. Check the network and retry.
    pause
    exit /b 1
)
echo Ready. Run run_jlink_pid_tuner.bat.
pause
