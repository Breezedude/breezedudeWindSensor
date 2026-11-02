@echo off
REM Host test for src/fanet_id.cpp - the FANET address derived from the MCU UUID:
REM the old way up to hardware 2.4, the checksum from 2.5 on, and the values an
REM address must not have.
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

copy /Y "%HERE%..\..\..\src\fanet_id.cpp" "%WORK%\fanet_id.cpp" >nul
copy /Y "%HERE%..\..\..\src\fanet_id.h"   "%WORK%\fanet_id.h"   >nul
copy /Y "%HERE%test_fanet_id.cpp"         "%WORK%\"            >nul

pushd "%WORK%"
cl /nologo /EHsc /W3 /std:c++17 /utf-8 /I. test_fanet_id.cpp fanet_id.cpp /Fe:test_fanet_id.exe >nul
if errorlevel 1 (
  echo COMPILE FAILED
  popd
  exit /b 1
)
REM .\ prefix: some shells set NoDefaultCurrentDirectoryInExePath
.\test_fanet_id.exe
set RC=%ERRORLEVEL%
popd
exit /b %RC%
