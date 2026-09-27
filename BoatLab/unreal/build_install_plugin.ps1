<#
Builds the BoatLab BoatPhysics plugin v2 (adds the forward RGB + depth camera, reset_pose and
payload markers) and installs it into <project>/Plugins/BoatPhysics.

    powershell -ExecutionPolicy Bypass -File BoatLab\unreal\build_install_plugin.ps1
    ... -SkipInstall       build only (inspect %TEMP%\boatlab_build\blp<stamp>\ first)
    ... -Engine "D:/UE_5.8" override settings.json engine_root

Close the Unreal editor before installing. The previous plugin (Source + Binaries) is backed up to
BoatLab\unreal\backups\BoatPhysics_<stamp>; restore by copying it back with the editor closed.
Requires Visual Studio 2022 C++ tools + Windows SDK (the same setup that built the plugin before).
#>
param([string]$Engine = '', [switch]$SkipInstall)
$ErrorActionPreference = 'Stop'
$lab = Split-Path $PSScriptRoot -Parent
$root = Split-Path $lab -Parent
$settings = Get-Content (Join-Path $lab 'settings.json') -Raw | ConvertFrom-Json
if (-not $Engine) { $Engine = if ($env:UE_58_ROOT) { $env:UE_58_ROOT } else { $settings.engine_root } }
$uat = Join-Path $Engine 'Engine/Build/BatchFiles/RunUAT.bat'
if (-not (Test-Path -LiteralPath $uat)) { throw "RunUAT.bat not found under $Engine - set engine_root in BoatLab/settings.json" }

$stamp = [DateTime]::Now.ToString('HHmmss')
# Build on a local disk with short paths: UHT/MSVC generated paths must stay under the 260-character
# Windows limit, and UBA cannot memory-map the shared PCH on Google Drive ("Incorrect function").
$buildRoot = Join-Path ([IO.Path]::GetTempPath()) 'boatlab_build'
$stage = Join-Path $buildRoot "bls$stamp"
$package = Join-Path $buildRoot "blp$stamp"
New-Item -ItemType Directory -Force -Path $stage | Out-Null
Copy-Item -Path (Join-Path $PSScriptRoot 'BoatPhysics/*') -Destination $stage -Recurse -Force
Write-Host "Building BoatPhysics v2 -> $package"
& $uat BuildPlugin "-Plugin=$stage/BoatPhysics.uplugin" "-Package=$package" -TargetPlatforms=Win64 -NoP4 -NoDeleteHostProject
if ($LASTEXITCODE -ne 0) { throw "Plugin build failed ($LASTEXITCODE). Nothing was installed. See the log above." }
if ($SkipInstall) { Write-Host "Build OK (not installed): $package"; exit 0 }

$editor = Get-Process -Name UnrealEditor -ErrorAction SilentlyContinue
if ($editor) { throw "Unreal Editor is running. Save your work, close it, then run this script again (build output kept in $package)." }

$dest = Join-Path $root 'Plugins/BoatPhysics'
$backup = Join-Path $PSScriptRoot ("backups/BoatPhysics_" + [DateTime]::Now.ToString('yyyyMMdd_HHmmss'))
New-Item -ItemType Directory -Force -Path $backup | Out-Null
foreach ($item in 'BoatPhysics.uplugin', 'Source', 'Binaries') {
    $p = Join-Path $dest $item
    if (Test-Path -LiteralPath $p) { Copy-Item -LiteralPath $p -Destination $backup -Recurse -Force }
}
Write-Host "Backed up current plugin to $backup"
Copy-Item -LiteralPath (Join-Path $package 'BoatPhysics.uplugin') -Destination $dest -Force
Remove-Item -LiteralPath (Join-Path $dest 'Source') -Recurse -Force -ErrorAction SilentlyContinue
Copy-Item -LiteralPath (Join-Path $package 'Source') -Destination $dest -Recurse -Force
Copy-Item -LiteralPath (Join-Path $package 'Binaries') -Destination $dest -Recurse -Force
# Keep the plugin marked as prebuilt so the editor loads the binaries instead of asking to rebuild.
$up = Get-Content (Join-Path $dest 'BoatPhysics.uplugin') -Raw | ConvertFrom-Json
$up | Add-Member -NotePropertyName Installed -NotePropertyValue $true -Force
$up | ConvertTo-Json -Depth 5 | Set-Content (Join-Path $dest 'BoatPhysics.uplugin') -Encoding UTF8
Get-FileHash -LiteralPath (Join-Path $dest 'Binaries/Win64/UnrealEditor-BoatPhysics.dll') | Format-List
Write-Host "Installed BoatPhysics v2. Open the editor, press Play, then run: BoatLab\START.cmd -> check-unreal"
