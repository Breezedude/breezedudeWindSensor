@echo off
REM Host test for src/fwd_queue.cpp - the packets waiting to be forwarded: a second
REM station during the delay of the first, a full queue, a channel that stays busy,
REM and the counter wrap.
REM
REM It compiles the real firmware source natively. The module has no
REM dependencies beyond stdint, so there are no stubs. Needs Visual Studio's
REM cl.exe.
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

copy /Y "%HERE%..\..\..\src\fwd_queue.cpp" "%WORK%\fwd_queue.cpp" >nul
copy /Y "%HERE%..\..\..\src\fwd_queue.h"   "%WORK%\fwd_queue.h"   >nul
copy /Y "%HERE%test_fwd_queue.cpp"         "%WORK%\"            >nul

pushd "%WORK%"
cl /nologo /EHsc /W3 /std:c++17 /utf-8 /I. test_fwd_queue.cpp fwd_queue.cpp /Fe:test_fwd_queue.exe >nul
if errorlevel 1 (
  echo COMPILE FAILED
  popd
  exit /b 1
)
REM .\ prefix: some shells set NoDefaultCurrentDirectoryInExePath
.\test_fwd_queue.exe
set RC=%ERRORLEVEL%
popd
exit /b %RC%
