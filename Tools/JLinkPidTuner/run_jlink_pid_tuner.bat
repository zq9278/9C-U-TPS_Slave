@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo First run: execute setup.bat to create the Python environment.
    pause
    exit /b 1
)
".venv\Scripts\python.exe" jlink_pid_tuner.py
if errorlevel 1 pause
