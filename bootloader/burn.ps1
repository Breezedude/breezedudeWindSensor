# Series flashing of the Breezedude bootloader over SWD (OpenOCD + CMSIS-DAP).
#
# Started by burn.bat. Runs until Ctrl+C:
#   1. probe for a board until one answers twice in a row (a board being
#      plugged in makes contact in stages),
#   2. clear the screen, flash with a progress bar, show OK or FAILED,
#   3. wait until that board is gone, then look for the next one.
#
# The flash sequence is the one burn.bat always used: halt, chip erase,
# BOOTPROT off, program + verify at 0x0, BOOTPROT back to 16 KiB, run. A
# failed attempt is retried once at a lower SWD clock.
#
# The full OpenOCD output of a failed board goes to burn-last-error.log next to
# this script. Keep this file ASCII: Windows PowerShell 5.1 reads scripts
# without a BOM in the ANSI code page.

$ErrorActionPreference = 'Continue'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$bootBin = Join-Path $here 'bootloader-breezedude-16k.bin'
$errorLog = Join-Path $here 'burn-last-error.log'

$BLOCK_FULL = [string][char]0x2588
$BLOCK_EMPTY = [string][char]0x2591
try { [Console]::OutputEncoding = [System.Text.Encoding]::UTF8 } catch {}

function Write-Status([string]$text, [ConsoleColor]$color = 'White') {
  # One line, rewritten in place.
  $width = [Math]::Max(20, [Console]::WindowWidth - 1)
  if ($text.Length -gt $width) { $text = $text.Substring(0, $width) }
  Write-Host ("`r" + $text.PadRight($width)) -NoNewline -ForegroundColor $color
}

function Get-BootloaderVersion([string]$path) {
  $text = [System.Text.Encoding]::ASCII.GetString([System.IO.File]::ReadAllBytes($path))
  $m = [regex]::Match($text, 'UF2 Bootloader (v[0-9][0-9A-Za-z.\-]*)')
  if ($m.Success) { return $m.Groups[1].Value } else { return 'unknown version' }
}

if (-not (Get-Command openocd -ErrorAction SilentlyContinue)) {
  Write-Host 'openocd not found on PATH.' -ForegroundColor Red
  exit 1
}
if (-not (Test-Path $bootBin)) {
  Write-Host "Bootloader binary not found: $bootBin" -ForegroundColor Red
  exit 1
}
$bootVersion = Get-BootloaderVersion $bootBin
$bootBinOcd = $bootBin -replace '\\', '/'
$baseArgs = '-f interface/cmsis-dap.cfg -f target/at91samdXX.cfg -c "transport select swd"'

# Runs OpenOCD and hands every output line to $onLine. Returns the exit code.
function Invoke-OpenOcd([string]$ocdArgs, [scriptblock]$onLine) {
  $psi = New-Object System.Diagnostics.ProcessStartInfo
  $psi.FileName = 'cmd.exe'
  # OpenOCD writes everything to stderr; 2>&1 gives one stream to read.
  $psi.Arguments = "/c openocd $ocdArgs 2>&1"
  $psi.UseShellExecute = $false
  $psi.RedirectStandardOutput = $true
  $psi.CreateNoWindow = $true
  $p = [System.Diagnostics.Process]::Start($psi)
  while (($line = $p.StandardOutput.ReadLine()) -ne $null) {
    if ($onLine) { & $onLine $line }
  }
  $p.WaitForExit()
  return $p.ExitCode
}

# 'board', 'none' (adapter there, no target answering) or 'noadapter'.
function Test-Board {
  $script:probeLines = New-Object System.Collections.Generic.List[string]
  $code = Invoke-OpenOcd "$baseArgs -c `"adapter speed 400`" -c init -c shutdown" { param($l) $script:probeLines.Add($l) }
  $all = $script:probeLines -join "`n"
  if ($all -match 'unable to find a matching CMSIS-DAP device|unable to open CMSIS-DAP') { return 'noadapter' }
  if ($code -eq 0 -and $all -match 'SWD DPIDR') { return 'board' }
  return 'none'
}

# Flash stages as OpenOCD reports them, with the share of the bar each one reaches.
$stages = @(
  @{ Pattern = 'SWD DPIDR';                Percent = 10; Label = 'connected' },
  @{ Pattern = 'chip erase';               Percent = 20; Label = 'erasing' },
  @{ Pattern = 'chip erased|bootloader size'; Percent = 35; Label = 'erased' },
  @{ Pattern = 'Programming Started';      Percent = 40; Label = 'programming' },
  @{ Pattern = 'Programming Finished';     Percent = 70; Label = 'programmed' },
  @{ Pattern = 'Verify Started';           Percent = 75; Label = 'verifying' },
  @{ Pattern = 'Verified OK';              Percent = 90; Label = 'verified' },
  @{ Pattern = 'shutdown command invoked'; Percent = 100; Label = 'done' }
)

function Show-Bar([int]$percent, [string]$label) {
  $width = 30
  $filled = [int][Math]::Round($width * $percent / 100)
  $bar = ($BLOCK_FULL * $filled) + ($BLOCK_EMPTY * ($width - $filled))
  Write-Status ("  {0} {1,3} %  {2}" -f $bar, $percent, $label) 'White'
}

function Invoke-Flash([int]$speedKhz) {
  $script:flashLog = New-Object System.Collections.Generic.List[string]
  $script:flashPercent = 0
  $ocdArgs = "$baseArgs -c `"adapter speed $speedKhz`" -c init -c `"reset halt`" -c `"at91samd chip-erase`" " +
             "-c `"reset halt`" -c `"at91samd bootloader 0`" -c `"reset halt`" " +
             "-c `"program {$bootBinOcd} 0x00000000 verify`" -c `"reset halt`" " +
             "-c `"at91samd bootloader 16384`" -c `"reset halt`" -c `"reset run`" -c shutdown"
  Show-Bar 0 'starting'
  $code = Invoke-OpenOcd $ocdArgs {
    param($l)
    $script:flashLog.Add($l)
    foreach ($s in $stages) {
      if ($s.Percent -gt $script:flashPercent -and $l -match $s.Pattern) {
        $script:flashPercent = $s.Percent
        Show-Bar $s.Percent $s.Label
      }
    }
  }
  $ok = ($code -eq 0) -and (($script:flashLog -join "`n") -match 'Verified OK')
  return $ok
}

function Get-FailureReason {
  $err = $script:flashLog | Where-Object { $_ -match '^Error' } | Select-Object -Last 1
  if ($err) { return ($err -replace '^Error:\s*', '') }
  return 'see burn-last-error.log'
}

$flashed = 0
$failed = 0
Clear-Host

while ($true) {
  # --- look for the next board ---
  $attempt = 0
  $seen = 0
  while ($seen -lt 2) {
    $attempt++
    $state = Test-Board
    switch ($state) {
      'board'     { $seen++ }
      'noadapter' { $seen = 0; Write-Status ("SWD adapter not found - connect the CMSIS-DAP probe ({0})" -f $attempt) 'Red' }
      default     { $seen = 0; Write-Status ("Connecting ... attempt {0}" -f $attempt) 'White' }
    }
    if ($seen -lt 2) { Start-Sleep -Milliseconds 300 }
  }

  # --- flash it ---
  Clear-Host
  $boardNo = $flashed + $failed + 1
  Write-Host ("Breezedude bootloader {0}   flashed: {1}   failed: {2}" -f $bootVersion, $flashed, $failed) -ForegroundColor DarkGray
  Write-Host ("Board {0}" -f $boardNo) -ForegroundColor White

  $ok = Invoke-Flash 400
  if (-not $ok) {
    Show-Bar 0 'retrying at 100 kHz'
    $ok = Invoke-Flash 100
  }
  Write-Host ''
  if ($ok) {
    $flashed++
    Write-Host '  OK' -ForegroundColor Green
  } else {
    $failed++
    $script:flashLog | Set-Content -Path $errorLog -Encoding UTF8
    Write-Host ('  FAILED: ' + (Get-FailureReason)) -ForegroundColor Red
  }

  # --- wait until this board is gone, so it is not flashed twice ---
  while ((Test-Board) -eq 'board') {
    Write-Status 'Disconnect the board ...' 'White'
    Start-Sleep -Milliseconds 300
  }
}
