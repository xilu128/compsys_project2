# Shared discovery for build/debug tooling. No installation-specific path required.
function Find-CubeIde([string]$Preferred) {
    if ($Preferred) {
        if (Test-Path -LiteralPath (Join-Path $Preferred 'stm32cubeidec.exe')) { return $Preferred }
        throw "Invalid CubeIDE directory: $Preferred"
    }
    $candidates=@()
    $cmd=Get-Command stm32cubeidec.exe -ErrorAction SilentlyContinue
    if ($cmd) { $candidates += Split-Path $cmd.Source -Parent }
    $keys=@('HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*',
            'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\*',
            'HKCU:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*')
    foreach ($entry in (Get-ItemProperty $keys -ErrorAction SilentlyContinue |
                        Where-Object DisplayName -Match 'STM32CubeIDE')) {
        if ($entry.InstallLocation) {
            $base=$entry.InstallLocation.Trim('"')
            $candidates += $base
            $candidates += Join-Path $base 'STM32CubeIDE'
        }
    }
    foreach ($drive in (Get-PSDrive -PSProvider FileSystem | Where-Object Root -Match '^[A-Z]:\\$')) {
        $roots=@((Join-Path $drive.Root 'ST'),(Join-Path $drive.Root 'Program Files\STMicroelectronics'))
        $roots += Get-ChildItem -LiteralPath $drive.Root -Directory -Filter '*STM32CubeIDE*' -ErrorAction SilentlyContinue |
                  Select-Object -ExpandProperty FullName
        foreach ($root in $roots) {
            if (Test-Path -LiteralPath $root) {
                $candidates += Get-ChildItem -LiteralPath $root -Filter stm32cubeidec.exe -File -Recurse -Depth 3 -ErrorAction SilentlyContinue |
                               ForEach-Object { $_.DirectoryName }
            }
        }
    }
    $found=@($candidates | Sort-Object -Unique | Where-Object {
        Test-Path -LiteralPath (Join-Path $_ 'stm32cubeidec.exe')
    })
    if ($found.Count -eq 1) { return $found[0] }
    if ($found.Count -gt 1) { throw "Multiple IDEs found; set -IdeDirectory: $($found -join ', ')" }
    throw 'STM32CubeIDE not found; set STM32CUBEIDE_HOME or -IdeDirectory.'
}
