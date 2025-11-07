@echo off
REM Quick build and flash script for jig firmware

cd tools\test_jig_firmware

echo ========================================
echo Building Jig Firmware...
echo ========================================
idf.py build

if %ERRORLEVEL% NEQ 0 (
    echo Build FAILED!
    pause
    exit /b 1
)

echo.
echo ========================================
echo Build SUCCESS! Ready to flash.
echo ========================================
echo.
echo Flash command: idf.py -p COM24 flash monitor
echo.
pause
