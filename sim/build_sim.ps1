# Host simulation build -- NOT the firmware build.
#
# Compiles the root main.cpp control loop against the shim headers in sim/
# (esp_log.h, driver/i2c.h, freertos/*) so the gait/transform math can be run
# and inspected on a PC. The firmware entry point is main/main.cpp, which is
# built by ESP-IDF/PlatformIO and is unaffected by this script.
#
#   .\sim\build_sim.ps1           # build sim\tpup_sim.exe
#   .\sim\build_sim.ps1 -Run      # build, then run it
#   .\sim\build_sim.ps1 -RealTime # delays actually sleep instead of being virtual
#   .\sim\build_sim.ps1 -DebugBuild # -g -O0 instead of -O2

[CmdletBinding()]
param(
    [switch]$Run,
    [switch]$RealTime,
    [switch]$DebugBuild
)

$ErrorActionPreference = 'Stop'

$SimDir  = $PSScriptRoot
$RootDir = Split-Path -Parent $SimDir
$Output  = Join-Path $SimDir 'tpup_sim.exe'

# Sources on the control path.
#
# Excluded on purpose:
#   Test_Session.cpp, Self_Test.cpp  hardware test code, firmware build only
#   Filter.cpp                       does not compile yet (its two static
#                                    helpers are called above their
#                                    definitions, so they are not in scope at
#                                    the call site). main.cpp includes
#                                    Filter.hpp but calls nothing from it, so
#                                    it is left out until that is fixed.
$Sources = @(
    'main.cpp'
    'Common.cpp'
    'Gait_Controller.cpp'
    'IK.cpp'
    'IMU.cpp'
    'Servo.cpp'
    'Spot_Model.cpp'
    'Transformer.cpp'
    'TrapezoidPath.cpp'
) | ForEach-Object { Join-Path $RootDir $_ }

$Compiler = (Get-Command g++ -ErrorAction SilentlyContinue).Source
if (-not $Compiler) {
    $Fallback = 'C:\msys64\mingw64\bin\g++.exe'
    if (Test-Path $Fallback) {
        $Compiler = $Fallback
    } else {
        throw "g++ not found on PATH and not at $Fallback. Install MSYS2 mingw-w64 g++ or add it to PATH."
    }
}

# gnu++23 matches what ESP-IDF builds the firmware with. It is not optional:
# Spot_Model.cpp uses aggregate class template argument deduction ('vec pos =
# {...}'), which is C++20, and M_PI is only visible outside strict-ANSI mode.
$Flags = @('-std=gnu++23', '-Wall', '-Wextra')
if ($DebugBuild) { $Flags += @('-g', '-O0') } else { $Flags += '-O2' }
if ($RealTime) { $Flags += '-DSIM_REAL_TIME_DELAYS' }

# sim/ comes second so the shim headers are only picked up for includes the
# repo root does not satisfy itself.
$Includes = @("-I$RootDir", "-I$SimDir")

Write-Host "Building $Output"
& $Compiler @Flags @Includes @Sources '-o' $Output
if ($LASTEXITCODE -ne 0) { throw "Build failed (exit $LASTEXITCODE)." }
Write-Host "Built $Output"

if ($Run) {
    Write-Host "--- running ---"
    & $Output
    exit $LASTEXITCODE
}
