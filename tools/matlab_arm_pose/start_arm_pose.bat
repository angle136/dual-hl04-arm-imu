@echo off
setlocal

set "TOOL_DIR=%~dp0"
set "MATLAB_EXE=D:\Matlab_itself\bin\matlab.exe"

if not exist "%MATLAB_EXE%" set "MATLAB_EXE="
if not defined MATLAB_EXE (
    for /f "delims=" %%I in ('where matlab.exe 2^>nul') do if not defined MATLAB_EXE set "MATLAB_EXE=%%I"
)

if not defined MATLAB_EXE (
    echo MATLAB was not found. Add matlab.exe to PATH or edit this launcher.
    pause
    exit /b 1
)

start "" "%MATLAB_EXE%" -nosplash -softwareopengl -sd "%TOOL_DIR%" -r "try, arm_imu_live_pose; catch exception, disp(getReport(exception,'extended')); end"
