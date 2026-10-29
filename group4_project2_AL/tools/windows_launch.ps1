# Run the Linux scripts in WSL; no Windows execution policy is changed permanently.
param(
    [ValidateSet('install', 'run')][string]$Action = 'run',
    [Parameter(Mandatory=$true)][string]$ProjectPath,
    [Parameter(ValueFromRemainingArguments=$true)][string[]]$ScriptArguments
)
$ErrorActionPreference = 'Stop'
try {
    $sourcePath = (Get-Item -LiteralPath $ProjectPath).FullName
    $distribution = 'Ubuntu-22.04'
    if ($env:PROJECT2_WSL_DISTRO) { $distribution = $env:PROJECT2_WSL_DISTRO }
    # Explorer may open either a Linux UNC folder or an extracted Windows folder.
    if ($sourcePath -match '^\\\\(?:wsl[.]localhost|wsl[$])\\([^\\]+)\\(.*)$') {
        $distribution = $Matches[1]
        $linuxPath = '/' + $Matches[2].Replace('\', '/')
    } else {
        $linuxPath = (& wsl.exe -d $distribution -- wslpath -a -u $sourcePath | Out-String).Trim()
        if ($LASTEXITCODE -ne 0) { throw 'Cannot open Ubuntu-22.04. Install/start it before running this file.' }
    }
    $script = if ($Action -eq 'install') { 'install_build.sh' } else { 'run.sh' }
    Write-Host "Ubuntu: $distribution"
    Write-Host "Project: $linuxPath"
    # Positional arguments preserve spaces without constructing shell command text.
    & wsl.exe -d $distribution -- bash "$linuxPath/$script" @ScriptArguments
    exit $LASTEXITCODE
} catch {
    Write-Host $_ -ForegroundColor Red
    Write-Host 'Open Ubuntu and run: bash install_build.sh (or bash run.sh sim 1) from the package folder.'
    exit 1
}
