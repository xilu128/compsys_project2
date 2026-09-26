param(
    [ValidateSet('Debug','Release')][string]$Configuration='Debug',
    [string]$IdeDirectory=$env:STM32CUBEIDE_HOME,
    [string]$Workspace
)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'find_cubeide.ps1')
$IdeDirectory=Find-CubeIde $IdeDirectory
$repo=Split-Path $PSScriptRoot -Parent
$project=Join-Path $repo 'Projects\STM32L476JG-SensorTile\Applications\ALLMEMS1\STM32CubeIDE'
if (-not $Workspace) { $Workspace=Join-Path (Split-Path $repo -Parent) 'STM32CubeIDE_Workspace_Build' }
$ide=Join-Path $IdeDirectory 'stm32cubeidec.exe'
if (-not (Test-Path -LiteralPath $ide)) { throw "CubeIDE not found: $ide. Supply -IdeDirectory." }
$state=Join-Path $Workspace 'ide-configuration'
New-Item -ItemType Directory -Path $Workspace,$state -Force | Out-Null
# A source-only import has no generated makefile yet. Bootstrap before asking
# Eclipse to clean; otherwise make clean reports a false missing-target error.
if (-not (Test-Path -LiteralPath (Join-Path $project "$Configuration\makefile"))) {
    & $ide -nosplash -configuration $state -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data $Workspace -import $project -build "COMSYS704/$Configuration" 2>&1 | Tee-Object -FilePath (Join-Path $Workspace "bootstrap-$Configuration.log")
    if ($LASTEXITCODE -ne 0) { throw 'Initial makefile generation failed' }
}
$started=Get-Date
$log=Join-Path $Workspace "build-$Configuration.log"
& $ide -nosplash -configuration $state -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data $Workspace -import $project -cleanBuild "COMSYS704/$Configuration" 2>&1 | Tee-Object -FilePath $log
if ($LASTEXITCODE -ne 0) { throw "Build failed: $log" }
if (Select-String -LiteralPath $log -Pattern 'Build Failed|terminated with exit code [1-9]' -Quiet) {
    throw "Clean/build reported a failure: $log"
}
if (-not (Select-String -LiteralPath $log -Pattern 'Build Finished\. 0 errors' -Quiet)) {
    throw "Missing successful build summary: $log"
}
$elf=Join-Path $project "$Configuration\STM32L476JG-SensorTile_ALLMEMS1.elf"
if (-not (Test-Path -LiteralPath $elf) -or (Get-Item -LiteralPath $elf).LastWriteTime -lt $started) {
    throw "No fresh ELF produced: $elf"
}
Write-Output "Built $elf"
Get-FileHash -LiteralPath $elf -Algorithm SHA256
