param(
    [string]$SourceRoot = (Resolve-Path "$PSScriptRoot\..").Path,
    [string]$BuildDir = "",
    [string]$Config = "Release",
    [string]$QtDir = "C:\Qt\6.10.1\msvc2022_64",
    [string]$OnnxDir = "C:\onnxruntime-gpu",
    [string]$VcpkgBin = "C:\vcpkg\installed\x64-windows\bin",
    [string]$ModelsDir = "",
    [string]$OutputDir = "",
    [string]$NiInstaller = "",
    [string]$VcRedist = "",
    [string]$InnoSetup = "C:\Program Files (x86)\Inno Setup 6\ISCC.exe",
    [switch]$CopyNidaq = $true
)

if (-not $BuildDir) { $BuildDir = Join-Path $SourceRoot "build" }
if (-not $ModelsDir) { $ModelsDir = Join-Path $SourceRoot "models" }
if (-not $OutputDir) { $OutputDir = Join-Path $SourceRoot "installer_output" }

if (-not $NiInstaller) {
    $NiInstaller = Join-Path $env:USERPROFILE "Downloads\ni-daqmx_25.8_online.exe"
}
if (-not (Test-Path $NiInstaller)) {
    throw "NI-DAQmx installer not found: $NiInstaller"
}

if (-not $VcRedist) {
    $downloads = Join-Path $env:USERPROFILE "Downloads"
    $candidate = Get-ChildItem -Path $downloads -Filter "*vcredist*_x64*.exe" -ErrorAction SilentlyContinue | Select-Object -First 1
    if (-not $candidate) {
        $candidate = Get-ChildItem -Path $downloads -Filter "VC_redist.x64.exe" -ErrorAction SilentlyContinue | Select-Object -First 1
    }
    if ($candidate) {
        $VcRedist = $candidate.FullName
    }
}
if (-not $VcRedist -or -not (Test-Path $VcRedist)) {
    throw "VC++ Redistributable not found. Pass -VcRedist <path to vcredist_x64.exe>."
}

if (-not (Test-Path $InnoSetup)) {
    throw "Inno Setup compiler not found: $InnoSetup"
}

$packageScript = Join-Path $SourceRoot "scripts\package_portable.ps1"
if (-not (Test-Path $packageScript)) {
    throw "Portable packaging script not found: $packageScript"
}

$packageDir = & $packageScript `
    -SourceRoot $SourceRoot `
    -BuildDir $BuildDir `
    -Config $Config `
    -QtDir $QtDir `
    -OnnxDir $OnnxDir `
    -VcpkgBin $VcpkgBin `
    -ModelsDir $ModelsDir `
    -OutputDir (Join-Path $SourceRoot "dist") `
    -CopyNidaq:$CopyNidaq

$packageDir = ($packageDir | Select-Object -Last 1)
if (-not $packageDir) {
    throw "Portable packaging did not return an output directory."
}
$packageDir = (Resolve-Path $packageDir).Path

New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null

$issPath = Join-Path $SourceRoot "installer\installer.iss"
if (-not (Test-Path $issPath)) {
    throw "Installer script not found: $issPath"
}

$defineSource = "/DSourceDir=`"$packageDir`""
$defineNi = "/DNiInstaller=`"$NiInstaller`""
$defineVc = "/DVcRedist=`"$VcRedist`""
$defineOut = "/DOutputDir=`"$OutputDir`""

& $InnoSetup $defineSource $defineNi $defineVc $defineOut $issPath
if ($LASTEXITCODE -ne 0) {
    throw "Inno Setup failed with exit code $LASTEXITCODE"
}

Write-Host "Installer created in: $OutputDir"
