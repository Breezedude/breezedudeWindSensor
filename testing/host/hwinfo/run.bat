@echo off
REM Host test for pack_hwinfo() in src/fanet.cpp - the HWInfo byte layout.
REM
REM It compiles the real firmware source natively against the stubs in stubs\
REM (Arduino.h, defines.h, logging.h) and checks every byte of the three debug
REM payloads and the repeater report. Needs Visual Studio's cl.exe.
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

REM The firmware source uses quoted includes ("fanet.h" etc), which resolve
REM relative to the including file first - so it is copied next to the stubs.
copy /Y "%HERE%..\..\..\src\fanet.cpp" "%WORK%\fanet.cpp" >nul
copy /Y "%HERE%..\..\..\src\fanet.h"   "%WORK%\fanet.h"   >nul
copy /Y "%HERE%stubs\*.h"              "%WORK%\"          >nul
copy /Y "%HERE%test_hwinfo.cpp"        "%WORK%\"          >nul

pushd "%WORK%"
cl /nologo /EHsc /W3 /std:c++17 /I. test_hwinfo.cpp /Fe:test_hwinfo.exe >nul
if errorlevel 1 (
  echo COMPILE FAILED
  popd
  exit /b 1
)
REM .\ prefix: some shells set NoDefaultCurrentDirectoryInExePath
.\test_hwinfo.exe
set RC=%ERRORLEVEL%
popd
exit /b %RC%
