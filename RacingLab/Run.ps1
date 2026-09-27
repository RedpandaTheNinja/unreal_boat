param([ValidateSet('menu','editor','dashboard','settings','train','evaluate','baseline','diagnose','stop','check','manual','configure')][string]$Action='menu')
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$cfg=Get-Content (Join-Path $PSScriptRoot 'settings.json') -Raw | ConvertFrom-Json
$engine=$cfg.engine_root
if ($env:UE_58_ROOT) { $engine=$env:UE_58_ROOT }
$python=Join-Path $engine 'Engine/Binaries/ThirdParty/Python3/Win64/python.exe'
$editor=Join-Path $engine 'Engine/Binaries/Win64/UnrealEditor.exe'
if ($Action -eq 'menu') {
    Write-Host 'RacingLab: editor | dashboard | settings | configure | train | evaluate | baseline | diagnose | stop | check | manual'
    $Action=Read-Host 'Action'
    if ($Action -notin @('editor','dashboard','settings','train','evaluate','baseline','diagnose','stop','check','manual','configure')) { throw 'Unknown action' }
}
if ($Action -eq 'settings') { Start-Process notepad.exe -ArgumentList ('"'+(Join-Path $PSScriptRoot 'settings.json')+'"'); exit }
if ($Action -eq 'manual') { Start-Process notepad.exe -ArgumentList ('"'+(Join-Path $PSScriptRoot 'START_HERE.md')+'"'); exit }
if (-not (Test-Path -LiteralPath $python)) { throw 'UE Python missing. Edit engine_root in settings.json or set UE_58_ROOT.' }
switch ($Action) {
    { $_ -in @('train','evaluate') } { & (Join-Path $PSScriptRoot 'DQN.ps1') -Action $Action; exit $LASTEXITCODE }
    editor { Start-Process -FilePath $editor -ArgumentList ('"'+(Join-Path $root 'mcp_gpt.uproject')+'"') }
    dashboard {
        $url="http://127.0.0.1:$($cfg.dashboard_port)"
        $state=$null
        try { $state=Invoke-RestMethod "$url/api/state" -TimeoutSec 2 } catch {}
        if ($state -and $state.bridge -ne 'mini_f1_dashboard_v1') { throw 'Dashboard port belongs to another service' }
        if (-not $state) {
            $logs=Join-Path $PSScriptRoot 'runs';New-Item -ItemType Directory -Force $logs | Out-Null
            $server=Join-Path $root 'simulation_ml/integrations/rapyuta/dashboard/server.py'
            Start-Process -FilePath $python -ArgumentList @(('"'+$server+'"'),'--port',$cfg.dashboard_port,'--udp-port',$cfg.udp_port) -WindowStyle Hidden -RedirectStandardOutput (Join-Path $logs 'dashboard.log') -RedirectStandardError (Join-Path $logs 'dashboard_errors.log') | Out-Null
            for ($i=0;$i -lt 25;$i++) {
                Start-Sleep -Milliseconds 200
                try { $state=Invoke-RestMethod "$url/api/state" -TimeoutSec 1;break } catch {}
            }
            if (-not $state) { throw 'Dashboard did not start; see runs/dashboard_errors.log' }
        }
        Start-Process $url
    }
    check { & $python (Join-Path $PSScriptRoot 'check_install.py'); if ($LASTEXITCODE) { exit $LASTEXITCODE } }
    default { & $python (Join-Path $PSScriptRoot 'rl_demo.py') $Action; if ($LASTEXITCODE) { exit $LASTEXITCODE } }
}
