param([string]$Compiler, [string]$Python='python')
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$app=Join-Path $root 'Projects\STM32L476JG-SensorTile\Applications\ALLMEMS1'
if (-not $Compiler) {
    foreach ($name in @('clang','gcc','zig')) {
        $found=Get-Command $name -ErrorAction SilentlyContinue
        if ($found) { $Compiler=$found.Source; break }
    }
}
if (-not $Compiler) { throw 'Supply -Compiler with a desktop clang/gcc/zig executable (not ARM GCC).' }
$out=Join-Path ([IO.Path]::GetTempPath()) ('p2-tests-'+[guid]::NewGuid())
New-Item -ItemType Directory -Path $out | Out-Null
$exe=Join-Path $out 'test_p2.exe'
$ccArgs=@()
if ([IO.Path]::GetFileNameWithoutExtension($Compiler) -eq 'zig') { $ccArgs += 'cc' }
$ccArgs += @('-std=c99','-Wall','-Wextra','-Werror','-pedantic',('-I'+(Join-Path $app 'Inc')),
    (Join-Path $PSScriptRoot 'test_p2.c'),(Join-Path $app 'Src\p2_motion.c'),
    (Join-Path $app 'Src\p2_sensor.c'),'-o',$exe)
try {
    & $Compiler @ccArgs
    if ($LASTEXITCODE -ne 0) { throw 'C compilation failed' }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw 'C tests failed' }
    & $Python (Join-Path $PSScriptRoot 'test_calibration.py')
    if ($LASTEXITCODE -ne 0) { throw 'Calibration tests failed' }
    & $Python (Join-Path $PSScriptRoot 'test_packet.py')
    if ($LASTEXITCODE -ne 0) { throw 'Packet tests failed' }
    & $Python (Join-Path $PSScriptRoot 'test_logging.py')
    if ($LASTEXITCODE -ne 0) { throw 'Logging tests failed' }
} finally {
    # Only the uniquely created test directory; never a repository/workspace root.
    $resolved=(Resolve-Path -LiteralPath $out).Path
    $expected=[IO.Path]::GetFullPath($out)
    if ($resolved -eq $expected -and (Split-Path $resolved -Leaf) -like 'p2-tests-*') {
        Remove-Item -LiteralPath $resolved -Recurse -Force
    }
}
