@echo off
setlocal enabledelayedexpansion

echo.
echo Select board:
echo   1. damiao_h7
echo   2. dji_c
echo.
set /p BOARD_CHOICE="Enter choice (1-2): "

if "%BOARD_CHOICE%"=="1" (
    set "BOARD=damiao_h7"
) else if "%BOARD_CHOICE%"=="2" (
    set "BOARD=dji_c"
) else (
    echo [ERROR] Invalid choice
    exit /b 1
)

echo.
echo Select probe:
echo   1. stlink
echo   2. daplink
echo   3. jlink
echo.
set /p PROBE_CHOICE="Enter choice (1-3): "

if "%PROBE_CHOICE%"=="1" (
    set "PROBE=stlink"
) else if "%PROBE_CHOICE%"=="2" (
    set "PROBE=daplink"
) else if "%PROBE_CHOICE%"=="3" (
    set "PROBE=jlink"
) else (
    echo [ERROR] Invalid choice
    exit /b 1
)

echo.
echo Flashing: [%PROBE%] %BOARD%
echo.

:: Call flash_interactive.bat
call "%~dp0flash_interactive.bat" "%BOARD%" "%PROBE%"
