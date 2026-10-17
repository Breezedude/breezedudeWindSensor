@echo off
REM Runs every host test in this directory and sums up. Each test's run.bat
REM finds the compiler itself; pass "nosetup" through from a Developer Command
REM Prompt to skip that.
REM
REM Usage:  run_all.bat [nosetup]

setlocal EnableDelayedExpansion
set HERE=%~dp0
set FAILED=
set COUNT=0

for /D %%T in ("%HERE%*") do (
  if exist "%%T\run.bat" (
    set /A COUNT+=1
    echo.
    echo ===== %%~nxT =====
    call "%%T\run.bat" %1
    if errorlevel 1 set FAILED=!FAILED! %%~nxT
  )
)

echo.
if "%FAILED%"=="" (
  echo ALL %COUNT% HOST TESTS PASSED
  exit /b 0
) else (
  echo FAILED:%FAILED%
  exit /b 1
)
