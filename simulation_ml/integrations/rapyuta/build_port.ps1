param([string]$Engine = 'C:/Program Files/Epic Games/UE_5.8', [switch]$SkipInstall)
$ErrorActionPreference = 'Stop'
$workspace = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$plugin = Join-Path $workspace 'Plugins/RapyutaSimulationPlugins/RapyutaSimulationPlugins.uplugin'
$buildName = [DateTime]::UtcNow.ToString('HHmmssfff')
# Keep UHT/MSVC generated paths below Windows' 260-character tool limit.
$package = Join-Path $workspace "_tb/$buildName"
& "$Engine/Engine/Build/BatchFiles/RunUAT.bat" BuildPlugin "-Plugin=$plugin" "-Package=$package" -TargetPlatforms=Win64 -NoP4 -NoDeleteHostProject
if ($LASTEXITCODE -ne 0) { throw "Plugin build failed: $LASTEXITCODE" }
# Install compiled output only after a successful build. Original assets stay intact.
if (-not $SkipInstall) { Copy-Item -LiteralPath "$package/Binaries" -Destination (Split-Path $plugin) -Recurse -Force }
Write-Output "Built package: $package"
