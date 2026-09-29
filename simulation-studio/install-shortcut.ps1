$studioRoot = $PSScriptRoot
$studioPython = Join-Path $studioRoot '.venv\Scripts\pythonw.exe'
if (-not (Test-Path -LiteralPath $studioPython)) { throw 'The project Python environment is missing.' }
$desktopPath = [Environment]::GetFolderPath('DesktopDirectory')
$shortcutPath = Join-Path $desktopPath 'TME180 Simulation Studio.lnk'
$shellLink = New-Object -ComObject WScript.Shell
if (Test-Path -LiteralPath $shortcutPath) {
    $existingLink = $shellLink.CreateShortcut($shortcutPath)
    if ($existingLink.TargetPath -ne $studioPython) { throw 'A different shortcut already uses this name.' }
}
$shortcut = $shellLink.CreateShortcut($shortcutPath)
$shortcut.TargetPath = $studioPython
$shortcut.Arguments = '"' + (Join-Path $studioRoot 'launcher.pyw') + '"'
$shortcut.WorkingDirectory = $studioRoot
$shortcut.Description = 'TME180 local scenario, uncertainty and AEB experiments'
$shortcut.IconLocation = (Join-Path $studioRoot 'studio.ico') + ',0'
$shortcut.WindowStyle = 7
$shortcut.Save()
[PSCustomObject]@{ Shortcut = $shortcutPath; Target = $shortcut.TargetPath; Arguments = $shortcut.Arguments; WorkingDirectory = $shortcut.WorkingDirectory } | ConvertTo-Json
