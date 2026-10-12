# Host simulation build -- NOT the firmware build.
#
# Compiles the main/walk_main.cpp control loop against the shim headers in
# sim/ (esp_log.h, driver/i2c.h, freertos/*) so the gait/transform math can be
# run and inspected on a PC. The test-session entry point, main/test_main.cpp,
# is built only by ESP-IDF/PlatformIO and is unaffected by this script.
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
#                                    the call site). walk_main.cpp includes
#                                    Filter.hpp but calls nothing from it, so
#                                    it is left out until that is fixed.
$TpupDir = Join-Path $RootDir 'components\tpup'
$Sources = @(Join-Path $RootDir 'main\walk_main.cpp')
$Sources += @(
    'hardware\Board.cpp'
    'core\Common.cpp'
    'gait\Gait_Controller.cpp'
    'kinematics\IK.cpp'
    'hardware\IMU.cpp'
    'hardware\Servo.cpp'
    'kinematics\Spot_Model.cpp'
    'kinematics\Transformer.cpp'
    'gait\TrapezoidPath.cpp'
) | ForEach-Object { Join-Path $TpupDir $_ }

# walk_main.cpp defines app_main(), the signature ESP-IDF calls. A PC executable
# needs main(), so this shim supplies one and calls straight through.
$Sources += (Join-Path $SimDir 'sim_main.cpp')

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
# TPUP_CONTROL_LOOP picks the control-loop app_main in walk_main.cpp. Without it
# that function is preprocessed away and the link fails on a missing app_main.
$Flags = @('-std=gnu++23', '-Wall', '-Wextra', '-DTPUP_CONTROL_LOOP')
if ($DebugBuild) { $Flags += @('-g', '-O0') } else { $Flags += '-O2' }
if ($RealTime) { $Flags += '-DSIM_REAL_TIME_DELAYS' }

# sim/ comes last so the shim headers are only picked up for includes the
# tpup folders do not satisfy themselves.
$Includes = @('core', 'hardware', 'kinematics', 'gait', 'estimation') |
    ForEach-Object { "-I$(Join-Path $TpupDir $_)" }
$Includes += "-I$SimDir"

Write-Host "Building $Output"
& $Compiler @Flags @Includes @Sources '-o' $Output
if ($LASTEXITCODE -ne 0) { throw "Build failed (exit $LASTEXITCODE)." }
Write-Host "Built $Output"

if ($Run) {
    Write-Host "--- running ---"
    & $Output
    exit $LASTEXITCODE
}
