@echo off
REM Host test for src/fanet_sched.cpp - the receive plan of the scheduled
REM repeater: learning a sender's rhythm, opening windows around it, giving up
REM and starting over.
REM
REM It compiles the real firmware source natively against the stub in stubs\
REM (logging.h). Needs Visual Studio's cl.exe.
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

REM The module includes "logging.h", which /I. resolves to the stub.
copy /Y "%HERE%..\..\..\src\fanet_sched.cpp" "%WORK%\fanet_sched.cpp" >nul
copy /Y "%HERE%..\..\..\src\fanet_sched.h"   "%WORK%\fanet_sched.h"   >nul
copy /Y "%HERE%stubs\*.h"                    "%WORK%\"                >nul
copy /Y "%HERE%test_fanet_sched.cpp"         "%WORK%\"                >nul

pushd "%WORK%"
cl /nologo /EHsc /W3 /std:c++17 /utf-8 /I. test_fanet_sched.cpp fanet_sched.cpp /Fe:test_fanet_sched.exe >nul
if errorlevel 1 (
  echo COMPILE FAILED
  popd
  exit /b 1
)
REM .\ prefix: some shells set NoDefaultCurrentDirectoryInExePath
.\test_fanet_sched.exe
set RC=%ERRORLEVEL%
popd
exit /b %RC%
