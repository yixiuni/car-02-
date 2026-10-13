param([string]$Compiler = 'C:\MinGW\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectPath = Split-Path -Parent $PSScriptRoot
Push-Location -LiteralPath $projectPath
try {
    New-Item -ItemType Directory -Path 'build/host-tests' -Force | Out-Null
    foreach ($laps in @(1, 3, 0)) {
        $outputPath = "build/host-tests/control-$laps.exe"
        & $Compiler -std=c11 -Wall -Wextra -Werror "-DCAR_TARGET_LAPS=$laps" -ICore/Inc Core/Src/car_control.c tests/test_car_control.c -o $outputPath
        if ($LASTEXITCODE -ne 0) { throw "Control test compilation failed: $laps" }
        & ".\$outputPath"
        if ($LASTEXITCODE -ne 0) { throw "Control tests failed: $laps" }
    }
    foreach ($polarity in @(0, 1)) {
        foreach ($reverse in @(0, 1)) {
            $outputPath = "build/host-tests/gray-$polarity-$reverse.exe"
            & $Compiler -std=c11 -Wall -Wextra -Werror "-DCAR_GRAY_TARGET_BIT_IS_ONE=$polarity" "-DCAR_GRAY_REVERSE_ORDER=$reverse" -ICore/Inc Core/Src/gray_protocol.c tests/test_gray_protocol.c -o $outputPath
            if ($LASTEXITCODE -ne 0) { throw 'Gray protocol test compilation failed' }
            & ".\$outputPath"
            if ($LASTEXITCODE -ne 0) { throw 'Gray protocol tests failed' }
        }
    }
    python tests/check_interfaces.py
    if ($LASTEXITCODE -ne 0) { throw 'Interface audit failed' }
} finally {
    Pop-Location
}
