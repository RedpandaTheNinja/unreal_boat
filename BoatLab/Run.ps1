<#
BoatLab launcher (Windows PowerShell 5.1+). Double-click START.cmd for the menu, or:
    .\Run.ps1 <action>
Actions:
  setup      create BoatLab\.venv and install Python packages (once per computer)
  check      offline install check          check-unreal  also query Unreal (Play must be running)
  editor     open the Unreal project        plugin        build + install BoatPhysics v2 (camera) - editor closed
  course     open Unreal and build the AIMM course into the lake level (widens lake to 120 ft)
  dashboard  start/open the dashboard (http://127.0.0.1:8770)
  twin       dashboard + mission runner on the Python twin (no Unreal needed)
  unreal     dashboard + mission runner on Unreal (open the lake level and press Play first)
  fast       full mission on the twin as fast as possible, print the score, plot the run
  tests      run the regression tests        plot     plot the latest run
  tune       sample buoy colours from Unreal and propose HSV thresholds
  stop       stop the mission runner (motors neutral)
  fake       fake Unreal plugin server (test the Unreal path without Unreal)
  guide      open GUIDE.md
#>
param([string]$Action = 'menu', [string]$Mission = '', [string]$Color = '', [string]$Nav = '')
$ErrorActionPreference = 'Stop'
$lab = $PSScriptRoot
$root = Split-Path $lab -Parent
$cfg = Get-Content (Join-Path $lab 'settings.json') -Raw | ConvertFrom-Json
$engine = $cfg.engine_root
if ($env:UE_58_ROOT) { $engine = $env:UE_58_ROOT }
$editor = Join-Path $engine 'Engine/Binaries/Win64/UnrealEditor.exe'
$project = Join-Path $root $cfg.project_file
if (-not $Mission) { $Mission = $cfg.default_mission }
$actions = @('setup', 'check', 'check-unreal', 'editor', 'plugin', 'course', 'dashboard', 'twin', 'unreal', 'fast', 'tests', 'plot', 'tune', 'stop', 'fake', 'guide')

function Get-Python {
    if ($cfg.python -and (Test-Path -LiteralPath $cfg.python)) { return $cfg.python }
    $venv = Join-Path $lab '.venv/Scripts/python.exe'
    if (Test-Path -LiteralPath $venv) { return $venv }
    throw 'BoatLab Python environment missing. Run 01_Setup.cmd first (or set "python" in BoatLab/settings.json).'
}

function Find-Python([string]$exe, [string[]]$opts = @()) {
    # Full path of a working Python 3.11/3.12 behind $exe (e.g. py -3.12), or $null. It actually runs the
    # interpreter, so the Microsoft Store "python" alias on a PC without Python (prints a hint, exits 9009) is rejected.
    try { $out = & $exe @opts -c 'import sys; print(sys.executable); sys.exit(sys.version_info[:2] not in ((3, 11), (3, 12)))' 2>$null } catch { return $null }
    if ($LASTEXITCODE -eq 0 -and $out) { return "$out".Trim() }
    return $null
}

function Start-Dashboard {
    $py = Get-Python
    $url = "http://127.0.0.1:$($cfg.dashboard_port)"
    $up = $false
    try { Invoke-RestMethod "$url/api/course" -TimeoutSec 2 | Out-Null; $up = $true } catch {}
    if (-not $up) {
        $logs = Join-Path $lab 'runs'; New-Item -ItemType Directory -Force $logs | Out-Null
        Start-Process -FilePath $py -ArgumentList @('dashboard/server.py', '--port', $cfg.dashboard_port, '--runner', $cfg.runner_api_port, '--ue-port', $cfg.ue_udp_port) `
            -WorkingDirectory $lab -WindowStyle Hidden -RedirectStandardOutput (Join-Path $logs 'dashboard.log') -RedirectStandardError (Join-Path $logs 'dashboard_errors.log') | Out-Null
        for ($i = 0; $i -lt 30; $i++) { Start-Sleep -Milliseconds 200; try { Invoke-RestMethod "$url/api/course" -TimeoutSec 1 | Out-Null; $up = $true; break } catch {} }
        if (-not $up) { throw 'Dashboard did not start; see BoatLab/runs/dashboard_errors.log' }
    }
    Start-Process $url
}

function Start-UnrealPlay {
    # Start Simulate in the open editor via its MCP server (the plugin only listens on UDP during Play).
    # Not fatal: the runner waits up to 2 minutes for Play if this cannot start it.
    & (Get-Python) tools/unreal_play.py
}

function Start-Runner([string]$backend) {
    $py = Get-Python
    $url = "http://127.0.0.1:$($cfg.runner_api_port)/api/state"
    try { Invoke-RestMethod $url -TimeoutSec 1 | Out-Null; Write-Host 'A mission runner is already active. Use "stop" first.'; return } catch {}
    $runArgs = @('-m', 'boatnav.runner', '--backend', $backend, '--mission', $Mission, '--api-port', $cfg.runner_api_port, '--stay', '--start-paused')
    if ($backend -eq 'twin') { $runArgs += '--realtime' }
    if ($backend -eq 'unreal' -and (Test-Path -LiteralPath (Join-Path $lab 'config/vision_colors_unreal.json'))) { $runArgs += @('--vision', 'vision_colors_unreal.json') }
    # Vision colours are not tuned for Unreal yet (GUIDE 3.4): default Unreal runs to prior positions
    # (camera shown, not fused). Override with -Nav fused once vision_colors_unreal.json is tuned.
    if (-not $Nav -and $backend -eq 'unreal') { $Nav = 'prior' }
    if ($Nav) { $runArgs += @('--nav', $Nav) }
    if ($Color) { $runArgs += @('--color', $Color) }
    Write-Host "Starting runner: $py $($runArgs -join ' ')"
    Write-Host 'The mission starts PAUSED - press "Start / resume" on the dashboard. Close this window or run 09_Stop.cmd to stop.'
    & $py @runArgs
}

if ($Action -eq 'menu') {
    Write-Host ('BoatLab actions: ' + ($actions -join ' | '))
    $Action = Read-Host 'Action'
}
if ($Action -notin $actions) { throw "Unknown action '$Action'. Choose: $($actions -join ', ')" }

Push-Location $lab
try {
    switch ($Action) {
        'setup' {
            $base = Find-Python py @('-3.12')
            if (-not $base) { $base = Find-Python py @('-3.11') }
            if (-not $base) { $base = Find-Python python }
            if (-not $base) {
                # No Python installed: fall back to the Python 3.11 that ships with Unreal Engine.
                $base = Find-Python (Join-Path $engine 'Engine/Binaries/ThirdParty/Python3/Win64/python.exe')
                if ($base) { Write-Host 'No Python 3.11/3.12 installed; using the Python bundled with Unreal Engine.' }
            }
            if (-not $base) {
                throw ('No Python 3.11/3.12 found, and no Unreal Python under engine_root "' + $engine + '". Install Python 3.12 ' +
                    '(python.org, tick "py launcher", or run: winget install -e --id Python.Python.3.12), or fix engine_root in settings.json, then run setup again.')
            }
            Write-Host "Creating BoatLab/.venv from $base"
            & $base -m venv (Join-Path $lab '.venv')
            $py = Join-Path $lab '.venv/Scripts/python.exe'
            if ($LASTEXITCODE -or -not (Test-Path -LiteralPath $py)) { throw "Could not create BoatLab/.venv with $base" }
            & $py -m pip install --upgrade pip
            & $py -m pip install -r (Join-Path $lab 'requirements.txt')
            if ($LASTEXITCODE) { throw 'Installing the Python packages failed (see above). Check the internet connection and run setup again.' }
            & $py tools/check_install.py
        }
        'check' { & (Get-Python) tools/check_install.py }
        'check-unreal' { & (Get-Python) tools/check_install.py --unreal }
        'editor' { Start-Process -FilePath $editor -ArgumentList ('"' + $project + '"') }
        'plugin' { & (Join-Path $lab 'unreal/build_install_plugin.ps1') }
        'course' {
            $script = Join-Path $lab 'unreal/run_build_course.py'
            Write-Host 'Opening Unreal and building the course (watch the Output Log for [boatlab]). Save prompts: choose Save.'
            Start-Process -FilePath $editor -ArgumentList @(('"' + $project + '"'), ('-ExecutePythonScript="' + $script + '"'))
        }
        'dashboard' { Start-Dashboard }
        'twin' { Start-Dashboard; Start-Runner 'twin' }
        'unreal' { Start-Dashboard; Start-UnrealPlay; Start-Runner 'unreal' }
        'fast' {
            $py = Get-Python
            & $py -m boatnav.runner --backend twin --mission $Mission --api-port 0 --camera-hz 2
            & $py tools/plot_run.py
            $png = Get-ChildItem (Join-Path $lab 'runs') -Filter '*.full.png' | Sort-Object LastWriteTime | Select-Object -Last 1
            if ($png) { Start-Process $png.FullName }
        }
        'tests' { & (Get-Python) -m pytest tests -q }
        'plot' {
            & (Get-Python) tools/plot_run.py
            $png = Get-ChildItem (Join-Path $lab 'runs') -Filter '*.full.png' | Sort-Object LastWriteTime | Select-Object -Last 1
            if ($png) { Start-Process $png.FullName }
        }
        'tune' { Start-UnrealPlay; & (Get-Python) tools/tune_colors.py --backend unreal --frames 25 --save runs/frames_unreal --vision vision_colors_unreal.json }
        'stop' {
            $body = '{"cmd":"quit"}'
            try { Invoke-RestMethod "http://127.0.0.1:$($cfg.runner_api_port)/api/command" -Method Post -Body $body -ContentType 'application/json' -TimeoutSec 2 | Out-Null; Write-Host 'Runner asked to stop (motors neutral).' }
            catch { Write-Host 'No runner answered. Unreal neutralises the motors 0.5 s after the last command anyway.' }
        }
        'fake' { & (Get-Python) tools/fake_unreal.py }
        'guide' { Start-Process notepad.exe -ArgumentList ('"' + (Join-Path $lab 'GUIDE.md') + '"') }
    }
}
finally { Pop-Location }
