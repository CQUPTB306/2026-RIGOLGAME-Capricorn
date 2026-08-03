@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "HEX_FILE=%SCRIPT_DIR%Objects\empty_LP_MSPM0G3507_nortos_keil.hex"
set "JLINK_SCRIPT=%SCRIPT_DIR%flash_mspm0g3507.jlink"

if defined JLINK_EXE (
    set "JLINK_CMD=%JLINK_EXE%"
) else if exist "D:\Program Files\SEGGER\JLink\JLink.exe" (
    set "JLINK_CMD=D:\Program Files\SEGGER\JLink\JLink.exe"
) else if exist "C:\Program Files\SEGGER\JLink\JLink.exe" (
    set "JLINK_CMD=C:\Program Files\SEGGER\JLink\JLink.exe"
) else if exist "C:\Program Files (x86)\SEGGER\JLink\JLink.exe" (
    set "JLINK_CMD=C:\Program Files (x86)\SEGGER\JLink\JLink.exe"
) else (
    echo [ERROR] JLink.exe not found.
    echo         Set JLINK_EXE or install SEGGER J-Link software first.
    exit /b 1
)

if not exist "%HEX_FILE%" (
    echo [ERROR] HEX file not found:
    echo         %HEX_FILE%
    exit /b 1
)

if not exist "%JLINK_SCRIPT%" (
    echo [ERROR] J-Link script not found:
    echo         %JLINK_SCRIPT%
    exit /b 1
)

pushd "%SCRIPT_DIR%"
"%JLINK_CMD%" -NoGui 1 -ExitOnError 1 -AutoConnect 1 -Device MSPM0G3507 -If SWD -Speed 4000 -CommanderScript "%JLINK_SCRIPT%"
set "RET=%ERRORLEVEL%"
popd

exit /b %RET%
