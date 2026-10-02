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
    [switch]$SkipBuild,
    [int]$BaselineFailures = 0
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

# The self tests are expected to be fully green (round 92 finished the last
# documented failures: the ARM decoder/disassembler, the CMeP device register
# names, the Bigmac vector/stream tests and the machine wiring tests).  A
# non-zero baseline is still accepted so an older tree can be checked, but the
# default is zero: any failing case is a regression.  -BaselineFailures <n>
# overrides the number.
Step "self tests" {
    $output = & (Join-Path $bin "zlb_tests.exe") 2>&1 | Out-String
    Write-Host $output
    $match = [regex]::Match($output, '(\d+) case\(s\) failed')
    if (-not $match.Success) { $global:LASTEXITCODE = 1; return 1 }
    $failedCases = [int]$match.Groups[1].Value
    if ($failedCases -gt $BaselineFailures) {
        Write-Host ("self tests: {0} failing case(s), baseline is {1}" -f $failedCases, $BaselineFailures) -ForegroundColor Red
        $global:LASTEXITCODE = 1
        return 1
    }
    if ($failedCases -lt $BaselineFailures) {
        Write-Host ("self tests: {0} failing case(s) - better than the baseline of {1}; update docs/STATUS.md" -f $failedCases, $BaselineFailures) -ForegroundColor Yellow
    }
    # Step() reads $LASTEXITCODE, and the test binary just set it to 1 on its way out.
    $global:LASTEXITCODE = 0
    return 0
} | Out-Null

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

# Save states: a machine that is saved mid-boot, continued, then reloaded and
# continued the same distance must end up bit-identical. Any state the
# serialiser forgets shows up as a different continuation.
Step "save state determinism" {
    $work = Join-Path $root "build\state-check"
    New-Item -ItemType Directory -Force -Path $work | Out-Null
    $checkpoint = Join-Path $work "checkpoint.state"
    $direct = Join-Path $work "continued-direct.state"
    $resumed = Join-Path $work "continued-resumed.state"
    Remove-Item $checkpoint, $direct, $resumed -ErrorAction SilentlyContinue
    & $zeliboba -q -ex "runm 300000" -ex "savestate $checkpoint" -ex "runm 50000" -ex "savestate $direct" `
                -ex "loadstate $checkpoint" -ex "runm 50000" -ex "savestate $resumed" -ex "quit" | Out-Null
    if (-not (Test-Path $checkpoint) -or -not (Test-Path $direct) -or -not (Test-Path $resumed)) {
        Write-Host "save state: the emulator did not write all three states" -ForegroundColor Red
        $global:LASTEXITCODE = 1
        return 1
    }
    $a = (Get-FileHash $direct -Algorithm SHA256).Hash
    $b = (Get-FileHash $resumed -Algorithm SHA256).Hash
    Write-Host ("state checkpoint {0:N0} bytes, continuation {1:N0} bytes" -f (Get-Item $checkpoint).Length, (Get-Item $direct).Length)
    if ($a -ne $b) {
        Write-Host "save state: the resumed continuation differs from the direct one" -ForegroundColor Red
        $global:LASTEXITCODE = 1
        return 1
    }
    Write-Host "save state: resumed continuation is identical"
    $global:LASTEXITCODE = 0
    return 0
} | Out-Null

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
