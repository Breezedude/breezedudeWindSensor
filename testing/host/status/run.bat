@echo off
REM Host test for src/status.cpp - the CDC settings editor.
REM
REM It compiles the real firmware source natively against the stubs in stubs\
REM (Arduino.h, tools.h, logging.h, ota_ab.h, ota_lora.h) and exercises the
REM command parsing, staging and flash-write path. Needs Visual Studio's cl.exe.
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

REM The firmware source uses quoted includes ("tools.h" etc), which resolve
REM relative to the including file first - so it is copied next to the stubs.
copy /Y "%HERE%..\..\..\src\status.cpp" "%WORK%\status.cpp" >nul
copy /Y "%HERE%..\..\..\src\status.h"   "%WORK%\status.h"   >nul
copy /Y "%HERE%stubs\*.h"                      "%WORK%\"                  >nul
copy /Y "%HERE%test_status.cpp"         "%WORK%\"                  >nul

pushd "%WORK%"
cl /nologo /EHsc /W3 /std:c++17 /I. /DVERSION=\"1.2.0\" test_status.cpp /Fe:test_status.exe >nul
if errorlevel 1 (
  echo COMPILE FAILED
  popd
  exit /b 1
)
REM .\ prefix: some shells set NoDefaultCurrentDirectoryInExePath
.\test_status.exe
set RC=%ERRORLEVEL%
popd
exit /b %RC%
