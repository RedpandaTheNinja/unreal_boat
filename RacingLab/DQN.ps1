param([ValidateSet('train','evaluate','settings','setup')][string]$Action='train',[string]$Python312='')
$ErrorActionPreference='Stop'
$python=Join-Path $PSScriptRoot '.venv/Scripts/python.exe'
if ($Action -eq 'settings') { Start-Process notepad.exe -ArgumentList ('"'+(Join-Path $PSScriptRoot 'dqn_settings.json')+'"');exit }
if ($Action -eq 'setup') {
    $basePython=$Python312
    if (-not $basePython) {
        $existing=Join-Path (Split-Path $PSScriptRoot -Parent) 'simulation_ml/.venv/Scripts/python.exe'
        if (Test-Path -LiteralPath $existing) { $basePython=$existing }
        elseif (Get-Command py -ErrorAction SilentlyContinue) { $basePython= & py -3.12 -c 'import sys; print(sys.executable)' }
    }
    if (-not $basePython -or -not (Test-Path -LiteralPath $basePython)) { throw 'Install Python 3.12 with the Windows py launcher, or pass -Python312 C:/path/to/python.exe to DQN.ps1 -Action setup.' }
    & $basePython -c 'import sys; sys.exit(sys.version_info[:2] != (3,12))'
    if ($LASTEXITCODE) { throw 'DQN setup requires Python 3.12; the UE bundled Python is 3.11.' }
    & $basePython -m venv (Join-Path $PSScriptRoot '.venv')
    if ($LASTEXITCODE) {throw 'Cannot create venv; use installed Python 3.12: py -3.12 -m venv RacingLab/.venv'}
    & $python -m pip install -r (Join-Path $PSScriptRoot 'dqn_requirements.txt') --index-url https://download.pytorch.org/whl/cpu
    exit $LASTEXITCODE
}
if (-not (Test-Path -LiteralPath $python)) {throw 'Run 14_Setup_DQN.cmd first on this computer.'}
& $python (Join-Path $PSScriptRoot 'dqn_train.py') $Action
exit $LASTEXITCODE
