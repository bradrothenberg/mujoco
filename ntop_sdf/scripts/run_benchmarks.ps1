param(
    [string] $BuildDir = "",
    [string] $OutputDir = "",
    [int[]] $Bodies = @(10, 100, 1000, 10000),
    [int[]] $Rates = @(60, 120, 240),
    [int] $Warmup = 10,
    [int] $Steps = 20,
    [switch] $IncludeStockSdf
)

$ErrorActionPreference = 'Stop'
$projectDir = Split-Path -Parent $PSScriptRoot
$repoDir = Split-Path -Parent $projectDir
if (-not $BuildDir) { $BuildDir = Join-Path $repoDir 'build-ntop-sdf' }
if (-not $OutputDir) {
    $OutputDir = Join-Path $projectDir (Join-Path 'benchmark-output' (Get-Date -Format 'yyyyMMdd-HHmmss'))
}
if (Test-Path -LiteralPath $OutputDir) { throw "Output directory already exists: $OutputDir" }
New-Item -ItemType Directory -Path $OutputDir | Out-Null

function Save-Runs([string] $Name, [string] $Exe, [string] $Mode = '') {
    if (-not (Test-Path -LiteralPath $Exe)) { throw "Executable not found: $Exe" }
    $first = $true
    $outputFile = Join-Path $OutputDir $Name
    foreach ($rate in $Rates) {
        foreach ($count in $Bodies) {
            if ($Mode) { $lines = & $Exe $count $Warmup $Steps $rate $Mode 40 20 0.005 }
            else { $lines = & $Exe $count $Warmup $Steps $rate }
            if ($LASTEXITCODE -ne 0 -or $lines.Count -ne 2) { throw "Benchmark failed: $Exe, $Mode, $count, $rate" }
            if ($first) { Set-Content -LiteralPath $outputFile -Value $lines -Encoding utf8; $first = $false }
            else { Add-Content -LiteralPath $outputFile -Value $lines[1] -Encoding utf8 }
            Write-Output $lines[1]
        }
    }
}

Save-Runs 'standalone.csv' (Join-Path $BuildDir 'ntop_sdf_bench.exe')
Save-Runs 'mujoco-native.csv' (Join-Path $BuildDir 'mujoco_bench.exe') 'native'
Save-Runs 'mujoco-cached.csv' (Join-Path $BuildDir 'mujoco_bench.exe') 'cached'
if ($IncludeStockSdf) { Save-Runs 'mujoco-stock-sdf.csv' (Join-Path $BuildDir 'mujoco_bench.exe') 'field' }

$accuracy = & (Join-Path $BuildDir 'ntop_sdf_accuracy.exe')
if ($LASTEXITCODE -ne 0) { throw 'Contact accuracy benchmark failed' }
Set-Content -LiteralPath (Join-Path $OutputDir 'contact-accuracy.csv') -Value $accuracy -Encoding utf8
Write-Output "Results: $OutputDir"
