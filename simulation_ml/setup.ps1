param(
    [string]$Python = "python",
    [string]$EditorPython = "",
    [switch]$UseActiveEnvironment,
    [switch]$InstallConfig
)
$ErrorActionPreference = "Stop"
$workspaceRoot = $PSScriptRoot
$runtimePython = Join-Path $workspaceRoot '.venv/Scripts/python.exe'
$condaArgs = @()
if ($UseActiveEnvironment) {
    $runtimeOutput = & $Python (Join-Path $workspaceRoot 'scripts/python_environment.py')
    if ($LASTEXITCODE -ne 0) { throw 'Could not resolve the active Python 3.11+ environment.' }
    if (-not $runtimeOutput) { throw 'Python returned no environment information. Use -Python with the full path to your environment python.exe.' }
    $runtimeInfo = ($runtimeOutput -join [Environment]::NewLine) | ConvertFrom-Json
    if ([string]::IsNullOrWhiteSpace($runtimeInfo.executable) -or [string]::IsNullOrWhiteSpace($runtimeInfo.prefix)) {
        throw 'Python returned incomplete environment information; executable and prefix are required.'
    }
    $runtimePython = $runtimeInfo.executable
    if ($env:CONDA_PREFIX) {
        $condaPrefix = $env:CONDA_PREFIX.Trim().Trim('"')
        if ([IO.Path]::GetFullPath($runtimeInfo.prefix).TrimEnd('\') -ne [IO.Path]::GetFullPath($condaPrefix).TrimEnd('\')) {
            throw 'The selected Python is not in the activated conda environment.'
        }
        if (-not $env:CONDA_EXE -or -not (Test-Path -LiteralPath $env:CONDA_EXE)) {
            throw 'CONDA_EXE is unavailable. Activate conda in this PowerShell window first.'
        }
        $condaArgs = @('--conda-exe', $env:CONDA_EXE, '--conda-prefix', $condaPrefix)
    }
} elseif (-not (Test-Path -LiteralPath $runtimePython)) {
    & $Python -m venv (Join-Path $workspaceRoot '.venv')
    if ($LASTEXITCODE -ne 0) { throw 'Could not create the Python environment. Use -Python with a Python 3.11+ executable.' }
}
& $runtimePython -m pip install --disable-pip-version-check -e "${workspaceRoot}/unreal_fable/python[dev,mcp]"
if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed.' }
$configArgs = @((Join-Path $workspaceRoot 'scripts/configure_codex.py'), '--python', $runtimePython) + $condaArgs
if ($EditorPython) { $configArgs += @('--editor-python', $EditorPython) }
& $runtimePython @configArgs
if ($LASTEXITCODE -ne 0) { throw 'Configuration generation failed.' }
& $runtimePython (Join-Path $workspaceRoot 'scripts/smoke_mcp.py') --config (Join-Path $workspaceRoot 'codex-config.toml')
if ($LASTEXITCODE -ne 0) { throw 'MCP connection test failed; configuration was not installed.' }
if ($InstallConfig) {
    & $runtimePython @configArgs --install
    if ($LASTEXITCODE -ne 0) { throw 'Configuration installation failed.' }
}
Write-Host 'Codex MCP is ready. Reload this Codex project and select GPT-5.6 Sol for an existing task.'
