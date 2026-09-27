# zeliboba - build helper.
#
#   powershell -ExecutionPolicy Bypass -File build.ps1
#   powershell -ExecutionPolicy Bypass -File build.ps1 -Clean -Tests
#
# Locates the Visual Studio toolchain itself, so it works from a plain shell.
param(
    [switch]$Clean,
    [switch]$Tests,
    [string]$BuildDir = "build",
    [ValidateSet("Debug", "Release", "RelWithDebInfo")] [string]$Config = "Release",
    [string]$Target = ""
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path

$vswhere = "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $vswhere)) { throw "vswhere.exe not found - Visual Studio is required" }

$vsPath = & $vswhere -latest -products * -property installationPath
if (-not $vsPath) { throw "no Visual Studio installation found" }

$cmake = Join-Path $vsPath "Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
$ninja = Join-Path $vsPath "Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"
$vcvars = Join-Path $vsPath "VC\Auxiliary\Build\vcvars64.bat"
foreach ($tool in @($cmake, $vcvars)) {
    if (-not (Test-Path $tool)) { throw "missing tool: $tool" }
}

$buildDir = Join-Path $root $BuildDir
if ($Clean -and (Test-Path $buildDir)) { Remove-Item $buildDir -Recurse -Force }

# A generated .cmd keeps the quoting sane (paths contain spaces).
$scriptPath = Join-Path $env:TEMP "zlb_build.cmd"
$lines = @()
$lines += "@echo off"
$lines += "call `"$vcvars`" >nul"
if (Test-Path $ninja) {
    $lines += "`"$cmake`" -S `"$root`" -B `"$buildDir`" -G Ninja -DCMAKE_MAKE_PROGRAM=`"$ninja`" -DCMAKE_BUILD_TYPE=$Config"
} else {
    $lines += "`"$cmake`" -S `"$root`" -B `"$buildDir`" -G `"Visual Studio 18 2026`" -A x64"
}
$lines += "if errorlevel 1 exit /b 1"
$buildCmd = "`"$cmake`" --build `"$buildDir`""
if (-not (Test-Path $ninja)) { $buildCmd += " --config $Config" }
if ($Target) { $buildCmd += " --target $Target" }
$lines += $buildCmd
$lines | Set-Content -Path $scriptPath -Encoding ASCII

Write-Host "== configure + build ==" -ForegroundColor Cyan
& cmd.exe /c $scriptPath
if ($LASTEXITCODE -ne 0) { throw "build failed with exit code $LASTEXITCODE" }

if ($Tests) {
    Write-Host "== tests ==" -ForegroundColor Cyan
    $exe = Join-Path $buildDir "bin\zlb_tests.exe"
    if (-not (Test-Path $exe)) { $exe = Join-Path $buildDir "bin\$Config\zlb_tests.exe" }
    & $exe
    exit $LASTEXITCODE
}

Write-Host "build finished: $buildDir\bin" -ForegroundColor Green
