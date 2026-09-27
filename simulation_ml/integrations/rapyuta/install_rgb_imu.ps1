param([int]$EditorProcessId = 21124)
$ErrorActionPreference = 'Stop'
$workspace = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$logPath = Join-Path $PSScriptRoot 'validation/rgb_imu_install.txt'
Start-Transcript -Path $logPath -Force
try {
    $engineExe = 'C:/Program Files/Epic Games/UE_5.8/Engine/Binaries/Win64/UnrealEditor.exe'
    $source = Join-Path $workspace '_tb/144011025/Binaries/Win64'
    $destination = Join-Path $workspace 'Plugins/RapyutaSimulationPlugins/Binaries/Win64'
    if (-not (Test-Path -LiteralPath (Join-Path $source 'UnrealEditor-RapyutaSimulationPlugins.dll'))) { throw 'Built DLL missing' }
    $editorProcess = Get-Process -Id $EditorProcessId -ErrorAction SilentlyContinue
    if ($editorProcess -and $editorProcess.Path -ne $engineExe.Replace('/','\')) { throw 'Unexpected editor process path' }
    $backupDir = Join-Path $PSScriptRoot ('validation/binary_backup_rgb_' + [DateTime]::UtcNow.ToString('yyyyMMdd_HHmmss'))
    New-Item -ItemType Directory -Path $backupDir | Out-Null
    Get-ChildItem -LiteralPath $destination -File | Copy-Item -Destination $backupDir
    # Windows permits renaming a loaded DLL. Preserve it, then stage the next
    # startup binary even if the editor takes a long time to close.
    $installedDll = Join-Path $destination 'UnrealEditor-RapyutaSimulationPlugins.dll'
    $preservedDll = Join-Path $destination ('UnrealEditor-RapyutaSimulationPlugins.preRGB_' + [DateTime]::UtcNow.ToString('HHmmss') + '.dll')
    Move-Item -LiteralPath $installedDll -Destination $preservedDll
    Copy-Item -LiteralPath (Join-Path $source 'UnrealEditor-RapyutaSimulationPlugins.dll') -Destination $installedDll
    Copy-Item -LiteralPath (Join-Path $source 'UnrealEditor.modules') -Destination $destination -Force
    Get-FileHash -LiteralPath (Join-Path $destination 'UnrealEditor-RapyutaSimulationPlugins.dll') | Format-List
    if ($editorProcess) {
        if (-not $editorProcess.CloseMainWindow()) { throw 'New binary staged; could not request graceful editor close' }
        if (-not $editorProcess.WaitForExit(60000)) { throw 'New binary staged; editor remains open. Finish any save dialog and restart.' }
    }
    Copy-Item -LiteralPath (Join-Path $source 'UnrealEditor-RapyutaSimulationPlugins.pdb') -Destination $destination -Force
    $projectPath = Join-Path $workspace 'mcp_gpt.uproject'
    # Unreal is an interactive application the user needs to see and control.
    $newEditor = Start-Process -FilePath $engineExe -ArgumentList ('"' + $projectPath + '"') -PassThru
    Write-Output ('Restarted editor PID: ' + $newEditor.Id)
} finally { Stop-Transcript }
