# Share the selected ESP32 with Ubuntu. Close Arduino Serial Monitor first.
param(
    [string]$Port = 'COM7',
    [string]$Distribution = 'Ubuntu-22.04'
)
$ErrorActionPreference = 'Stop'
$usbTool = Join-Path $env:ProgramFiles 'usbipd-win\usbipd.exe'
if (-not (Test-Path -LiteralPath $usbTool)) {
    throw 'Install usbipd-win first: winget install --interactive --exact dorssel.usbipd-win'
}
if ($Port -notmatch '^COM[0-9]+$') { throw 'Use a Windows serial port name such as COM7.' }
$devices = & $usbTool list
$matched = @($devices | Where-Object { $_ -match ('\(' + [regex]::Escape($Port) + '\)') })
if ($matched.Count -ne 1 -or $matched[0] -notmatch '^\s*(\d+-\d+)\s') {
    $devices | Write-Output
    throw "Cannot uniquely identify $Port. Reconnect the ESP32 or check its current COM number."
}
$bus = [regex]::Match($matched[0], '^\s*(\d+-\d+)\s').Groups[1].Value
if ($matched[0] -match '\bAttached\b') {
    Write-Output "$Port is already attached to WSL. Run bash run.sh doctor in Ubuntu."
    return
}
if ($matched[0] -match 'Not shared') {
    # Windows requires elevation once per shared device.
    $process = Start-Process -FilePath $usbTool -ArgumentList @('bind', '--busid', $bus) -Verb RunAs -WindowStyle Hidden -Wait -PassThru
    if ($process.ExitCode -ne 0) { throw 'USB sharing was not approved or failed.' }
}
& wsl.exe -d $Distribution -- true
if ($LASTEXITCODE -ne 0) { throw 'Ubuntu could not start.' }
& $usbTool attach --wsl --busid $bus
if ($LASTEXITCODE -ne 0) { throw 'USB attachment failed; check the message above.' }
Write-Output "Attached $Port to WSL. In Ubuntu run bash run.sh doctor, then bash run.sh hardware io_only:=true."
Write-Output "Before uploading from Arduino again: usbipd detach --busid $bus"
