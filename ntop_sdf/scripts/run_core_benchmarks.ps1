param(
    [Parameter(Mandatory = $true)] [string] $SphereImplicit,
    [int[]] $Counts = @(10, 100, 1000),
    [int[]] $Rates = @(60, 120, 240),
    [int] $Warmup = 10,
    [int] $Steps = 20,
    [string] $Output = "ntop_sdf/results/ntop_core_sphere_benchmarks.csv"
)

$ErrorActionPreference = "Stop"
$repository = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$exe = Join-Path $repository "build-ntop-sdf/mujoco_bench.exe"
if (!(Test-Path -LiteralPath $exe) -or !(Test-Path -LiteralPath $SphereImplicit)) {
    throw "Build mujoco_bench with nTop Core and provide the SDK sphere fixture."
}
$implicitPath = (Resolve-Path -LiteralPath $SphereImplicit).Path
$csv = [System.Collections.Generic.List[string]]::new()
foreach ($count in $Counts) {
    foreach ($rate in $Rates) {
        foreach ($mode in @("native", "cached", "core")) {
            Write-Host "Running $mode, $count bodies, $rate Hz"
            $arguments = @($count, $Warmup, $Steps, $rate, $mode, 40, 20, 0.005)
            if ($mode -eq "core") { $arguments += $implicitPath }
            $lines = @(& $exe @arguments)
            if ($LASTEXITCODE -ne 0 -or $lines.Count -ne 2) {
                throw "Benchmark failed: $mode, $count bodies, $rate Hz."
            }
            if ($csv.Count -eq 0) { $csv.Add($lines[0]) }
            $csv.Add($lines[1])
        }
    }
}
$outputPath = if ([System.IO.Path]::IsPathRooted($Output)) { $Output } else { Join-Path $repository $Output }
$csv | Set-Content -LiteralPath $outputPath -Encoding utf8
Write-Host "Wrote $($csv.Count - 1) measurements to $outputPath"
