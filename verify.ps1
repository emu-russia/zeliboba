# zeliboba - end to end verification.
#
#   powershell -ExecutionPolicy Bypass -File verify.ps1
#   powershell -ExecutionPolicy Bypass -File verify.ps1 -Steps 2000000
#
# Builds everything, runs the self tests, rebuilds the eMMC image, walks the boot
# chain stage by stage and prints a summary. Exit code is non-zero when a step
# that is expected to work fails.
param(
    [int]$Steps = 400000,
    [switch]$SkipBuild
)

$ErrorActionPreference = "Continue"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$bin = Join-Path $root "build\bin"
$results = New-Object System.Collections.ArrayList

function Step($name, [scriptblock]$body) {
    Write-Host "== $name" -ForegroundColor Cyan
    $output = & $body 2>&1 | Out-String
    $code = $LASTEXITCODE
    [void]$results.Add([pscustomobject]@{ Step = $name; Exit = $code; Output = $output })
    Write-Host $output
    return $code
}

if (-not $SkipBuild) {
    Step "build" { powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $root "build.ps1") -BuildDir build } | Out-Null
}

$zeliboba = Join-Path $bin "zeliboba.exe"
if (-not (Test-Path $zeliboba)) { throw "zeliboba.exe was not built" }

Step "self tests" { & (Join-Path $bin "zlb_tests.exe") } | Out-Null

Step "machine layout" { & $zeliboba --info -q } | Out-Null

Step "boot plan" { & $zeliboba --boot -q } | Out-Null

Step "first loader: $Steps steps" {
    & $zeliboba -q -ex "core mep" -ex "run $Steps" -ex "boot" -ex "regs" -ex "quit"
} | Out-Null

Step "second loader (direct)" {
    & $zeliboba -q --stage second -ex "run $Steps" -ex "boot" -ex "quit"
} | Out-Null

Step "arm kernel boot loader" {
    & $zeliboba -q --stage kbl -ex "run $Steps" -ex "boot" -ex "quit"
} | Out-Null

Step "kernel stage" {
    & $zeliboba -q --stage kernel -ex "run $Steps" -ex "boot" -ex "quit"
} | Out-Null

$emmc = Join-Path $bin "emmc_rebuild.exe"
if (Test-Path $emmc) {
    Step "eMMC image" { & $emmc --firmware (Join-Path $root "..\Vita_104_Firmware\Out") --out (Join-Path $root "build\emmc.img") --verify } | Out-Null
}

Write-Host ""
Write-Host "================ summary ================" -ForegroundColor Yellow
foreach ($r in $results) {
    $status = if ($r.Exit -eq 0) { "ok  " } else { "FAIL" }
    "{0}  {1}" -f $status, $r.Step
}
Write-Host ""
Write-Host "milestones seen in the last first-loader run:"
$first = $results | Where-Object { $_.Step -like "first loader*" } | Select-Object -First 1
if ($first) {
    ($first.Output -split "`n") | Where-Object { $_ -match '^\s+\*' -or $_ -match 'milestone' } | ForEach-Object { $_.Trim() }
}

$failed = ($results | Where-Object { $_.Exit -ne 0 }).Count
if ($failed -gt 0) { Write-Host "$failed step(s) failed" -ForegroundColor Red; exit 1 }
Write-Host "all steps completed" -ForegroundColor Green
