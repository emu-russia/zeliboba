# zeliboba - standalone compile check for a set of source globs.
#
#   powershell -ExecutionPolicy Bypass -File check.ps1 "src\common\*.cpp" "src\bus\*.cpp"
#
# Compiles the listed files with cl /c into build-check\ so a workstream can
# validate its own code without depending on the rest of the tree compiling.
param(
    [Parameter(Mandatory = $true, ValueFromRemainingArguments = $true)]
    [string[]]$Patterns
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$out = Join-Path $root "build-check"
New-Item -ItemType Directory -Force $out | Out-Null

$vcvars = "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
if (-not (Test-Path $vcvars)) { throw "vcvars64.bat not found" }

$files = @()
foreach ($pattern in $Patterns) {
    $files += Get-Item (Join-Path $root $pattern) -ErrorAction SilentlyContinue | ForEach-Object { $_.FullName }
}
if ($files.Count -eq 0) { throw "no files matched" }

$defines = '/DZLB_ROOT_DIR=\"' + $root + '\" /DZLB_WORKSPACE_DIR=\"' + $root + '\..\"'
$fileArgs = ($files | ForEach-Object { '"' + $_ + '"' }) -join ' '

$scriptPath = Join-Path $env:TEMP "zlb_check.cmd"
$lines = @()
$lines += "@echo off"
$lines += 'call "' + $vcvars + '" >nul'
$lines += 'cd /d "' + $root + '"'
$lines += 'cl /nologo /c /std:c++20 /EHsc /W3 /I src ' + $defines + ' /Fo"' + $out + '\\" ' + $fileArgs
$lines | Set-Content -Path $scriptPath -Encoding ASCII

& cmd.exe /c $scriptPath
exit $LASTEXITCODE
