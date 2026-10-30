$ErrorActionPreference = "Stop"

$firmwareRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$hostDir = $PSScriptRoot
$buildDir = Join-Path $hostDir "build"
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

$vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path -LiteralPath $vswhere)) {
    throw "vswhere.exe not found. Install Visual Studio with the C++ toolchain."
}

$vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsPath) {
    throw "Visual Studio C++ toolchain not found."
}

$vcvars = Join-Path $vsPath "VC\Auxiliary\Build\vcvars64.bat"
if (-not (Test-Path -LiteralPath $vcvars)) {
    throw "vcvars64.bat not found: $vcvars"
}

$exe = Join-Path $buildDir "test_motor.exe"
$cmd = "call `"$vcvars`" >nul && cl /nologo /TC /W4 /utf-8 /I`"$hostDir\fake_hal`" /I`"$firmwareRoot\Core\Inc`" `"$hostDir\test_motor.c`" `"$firmwareRoot\Core\Src\motor.c`" /Fe:`"$exe`""

Push-Location $buildDir
try {
    & cmd.exe /d /s /c $cmd
    if ($LASTEXITCODE -ne 0) {
        throw "Host test compile/link failed with exit code $LASTEXITCODE"
    }

    & $exe
    if ($LASTEXITCODE -ne 0) {
        throw "Host test run failed with exit code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
}