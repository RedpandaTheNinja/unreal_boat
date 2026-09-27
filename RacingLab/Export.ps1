param([Parameter(Mandatory=$true)][string]$Destination,[switch]$CheckOnly)
$ErrorActionPreference='Stop'
$source=(Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$target=[IO.Path]::GetFullPath($Destination)
if ($target.StartsWith($source.TrimEnd('\')+'\',[StringComparison]::OrdinalIgnoreCase) -or $target -eq $source) { throw 'Choose a destination outside the source project.' }
if (Test-Path -LiteralPath $target) { throw 'Destination must be new; exporter never overwrites a handover.' }
$folders=@('Content','Config','Plugins/RapyutaSimulationPlugins','RacingLab','track','car_model','simulation_ml/integrations/rapyuta')
$manifest=@()
foreach ($folder in $folders) {
    foreach ($file in Get-ChildItem -LiteralPath (Join-Path $source $folder) -File -Recurse) {
        $relative=$file.FullName.Substring($source.Length+1)
        if ($relative -match '(^|\\)(Intermediate|DerivedDataCache|__pycache__|\.venv)(\\|$)') { continue }
        $manifest+=[pscustomobject]@{path=$relative;bytes=$file.Length}
    }
}
Write-Host "$($manifest.Count) files, $([math]::Round(($manifest | Measure-Object bytes -Sum).Sum/1GB,2)) GB -> $target"
if ($CheckOnly) { return }
New-Item -ItemType Directory -Path $target | Out-Null
foreach ($entry in $manifest) {
    $destinationFile=Join-Path $target $entry.path
    New-Item -ItemType Directory -Force -Path (Split-Path $destinationFile -Parent) | Out-Null
    Copy-Item -LiteralPath (Join-Path $source $entry.path) -Destination $destinationFile
}
$project=Get-Content (Join-Path $source 'mcp_gpt.uproject') -Raw | ConvertFrom-Json
$optional=@('ModelContextProtocol','Terminal','EditorToolset','MCPClientToolset')
$project.Plugins=@($project.Plugins | Where-Object { $_.Name -notin $optional })
$project | ConvertTo-Json -Depth 20 | Set-Content -Encoding UTF8 (Join-Path $target 'mcp_gpt.uproject')
$manifest | ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 (Join-Path $target 'HANDOVER_MANIFEST.json')
Write-Host 'Transfer prepared. Open RacingLab/START_HERE.md on the recipient computer.'
