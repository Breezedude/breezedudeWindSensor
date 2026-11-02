@echo off
rem Series flashing of the 16 KiB Breezedude bootloader over SWD.
rem Connect SWD first, then power; the script finds the board, flashes it,
rem shows OK or FAILED and waits for the next one. Ctrl+C ends it.
rem The work is done in burn.ps1 next to this file.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0burn.ps1"
