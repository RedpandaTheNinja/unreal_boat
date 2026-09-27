param([switch]$NoBrowser)
$ErrorActionPreference = 'Stop'
$workspace = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$url = 'http://127.0.0.1:8765'
$existing = $null
try { $existing = Invoke-RestMethod "$url/api/state" -TimeoutSec 2 } catch {}
if ($existing -and $existing.bridge -ne 'mini_f1_dashboard_v1') { throw 'Port 8765 belongs to another service.' }
if (-not $existing) {
    $pythonExe = 'C:/Program Files/Epic Games/UE_5.8/Engine/Binaries/ThirdParty/Python3/Win64/python.exe'
    $serverFile = Join-Path $PSScriptRoot 'dashboard/server.py'
    $logDir = Join-Path $PSScriptRoot 'validation'
    New-Item -ItemType Directory -Path $logDir -Force | Out-Null
    $process = Start-Process -FilePath $pythonExe -ArgumentList ('"' + $serverFile + '"') -WorkingDirectory $workspace -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logDir 'dashboard.log') -RedirectStandardError (Join-Path $logDir 'dashboard_errors.log')
    $process.Id | Set-Content (Join-Path $logDir 'dashboard.pid')
    for ($attempt=0; $attempt -lt 30; $attempt++) {
        Start-Sleep -Milliseconds 200
        try { $existing = Invoke-RestMethod "$url/api/state" -TimeoutSec 1; break } catch {}
    }
    if (-not $existing) { throw 'Dashboard failed to start. See validation/dashboard_errors.log.' }
}
if (-not $NoBrowser) { Start-Process $url }
Write-Output "Dashboard ready: $url"
