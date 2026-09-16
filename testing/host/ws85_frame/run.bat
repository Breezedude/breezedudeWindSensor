@echo off
REM Host test for src/ws85uart.h - how a WS85 frame is put together from what
REM the receiver delivers after a standby wakeup.
REM
REM It compiles the real firmware header natively against the stub in stubs\
REM (Arduino.h) and feeds it bursts recorded on the rig with the WS85_TRACE
REM build. Needs Visual Studio's cl.exe.
REM
REM Usage:  run.bat            (from a normal cmd prompt, finds vcvars itself)
REM         run.bat nosetup    (from a Developer Command Prompt)

setlocal
set HERE=%~dp0
set WORK=%HERE%build

if /I "%~1"=="nosetup" goto :build
for %%V in (
  "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
  "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat"
  "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
  "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
) do if exist %%V ( call %%V >nul 2>&1 & goto :build )
echo Could not find vcvars64.bat - run this from a Developer Command Prompt
echo and pass "nosetup", or adjust the paths above.
exit /b 1

:build
if not exist "%WORK%" mkdir "%WORK%"

REM The header includes <Arduino.h>, which /I. resolves to the stub.
copy /Y "%HERE%..\..\..\src\ws85uart.h" "%WORK%\ws85uart.h" >nul
copy /Y "%HERE%stubs\*.h"               "%WORK%\"           >nul
copy /Y "%HERE%test_ws85_frame.cpp"     "%WORK%\"           >nul

pushd "%WORK%"
cl /nologo /EHsc /W3 /std:c++17 /utf-8 /I. test_ws85_frame.cpp /Fe:test_ws85_frame.exe >nul
if errorlevel 1 (
  echo COMPILE FAILED
  popd
  exit /b 1
)
REM Once more with the trace fields compiled in.
cl /nologo /EHsc /W3 /std:c++17 /utf-8 /I. /DWS85_TRACE test_ws85_frame.cpp /Fe:test_ws85_frame_trace.exe >nul
if errorlevel 1 (
  echo COMPILE FAILED with WS85_TRACE
  popd
  exit /b 1
)
REM .\ prefix: some shells set NoDefaultCurrentDirectoryInExePath
echo release build
.\test_ws85_frame.exe
set RC=%ERRORLEVEL%
echo.
echo WS85_TRACE build
.\test_ws85_frame_trace.exe
if errorlevel 1 set RC=1
popd
exit /b %RC%
