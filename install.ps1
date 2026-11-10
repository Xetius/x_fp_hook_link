# Copies the mod into BeamNG's unpacked mods folder. Re-run after changing anything in mod\.
# Install with the game closed: input actions and mod scripts are only read when the game starts.
$running = Get-Process -Name 'BeamNG.drive.x64' -ErrorAction SilentlyContinue
if ($running) {
  Write-Host 'BeamNG is running. Close the game first, then run this again.'
  exit 1
}
$src = Join-Path $PSScriptRoot 'mod'
$mods = Join-Path $env:LOCALAPPDATA 'BeamNG\BeamNG.drive\current\mods\unpacked'
$dst = Join-Path $mods 'x_fp_hook_link'
robocopy $src $dst /MIR /NJH /NJS /NP | Out-Null
if ($LASTEXITCODE -ge 8) { throw "robocopy failed with exit code $LASTEXITCODE" }
Write-Host "Installed to $dst"
exit 0 # robocopy returns 1 for "files copied", which is success
