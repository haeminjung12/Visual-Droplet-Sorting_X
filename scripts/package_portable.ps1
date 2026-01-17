param(
    [string]$SourceRoot = (Resolve-Path "$PSScriptRoot\..").Path,
    [string]$BuildDir = "",
    [string]$Config = "Release",
    [string]$QtDir = "C:\Qt\6.10.1\msvc2022_64",
    [string]$OnnxDir = "C:\onnxruntime-gpu",
    [string]$VcpkgBin = "C:\vcpkg\installed\x64-windows\bin",
    [string]$ModelsDir = "",
    [string]$OutputDir = "",
    [switch]$CopyNidaq = $true,
    [string]$NidaqBin = "C:\Program Files (x86)\National Instruments\Shared\ExternalCompilerSupport\C\lib64\msvc"
)

if (-not $BuildDir) { $BuildDir = Join-Path $SourceRoot "build" }
if (-not $ModelsDir) { $ModelsDir = Join-Path $SourceRoot "models" }
if (-not $OutputDir) { $OutputDir = Join-Path $SourceRoot "dist" }

$exePath = Join-Path $BuildDir ("qt_hama_gui\" + $Config + "\droplet_pipeline.exe")
if (-not (Test-Path $exePath)) {
    throw "Executable not found: $exePath"
}

$windeployqt = Join-Path $QtDir "bin\windeployqt.exe"
if (-not (Test-Path $windeployqt)) {
    throw "windeployqt.exe not found: $windeployqt"
}

$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$packageDir = Join-Path $OutputDir ("droplet_pipeline_" + $stamp)
New-Item -ItemType Directory -Path $packageDir -Force | Out-Null

Copy-Item -Path $exePath -Destination $packageDir -Force

# Copy any DLLs that the build already produced beside the executable.
$buildDllDir = Split-Path $exePath -Parent
Get-ChildItem -Path $buildDllDir -Filter "*.dll" | ForEach-Object {
    Copy-Item -Path $_.FullName -Destination $packageDir -Force
}

# Add ONNX Runtime DLLs if available.
if (Test-Path $OnnxDir) {
    $onnxDlls = @(
        "onnxruntime.dll",
        "onnxruntime_providers_shared.dll",
        "onnxruntime_providers_cuda.dll",
        "onnxruntime_providers_tensorrt.dll"
    )
    foreach ($dll in $onnxDlls) {
        $src = Join-Path $OnnxDir "lib\$dll"
        if (Test-Path $src) {
            Copy-Item -Path $src -Destination $packageDir -Force
        }
    }
}

# Add OpenCV DLLs if available from vcpkg.
if (Test-Path $VcpkgBin) {
    Get-ChildItem -Path $VcpkgBin -Filter "opencv_*4.dll" | ForEach-Object {
        Copy-Item -Path $_.FullName -Destination $packageDir -Force
    }
}

# Optional: NI-DAQmx DLLs (only if you want to bundle them).
if ($CopyNidaq -and (Test-Path $NidaqBin)) {
    Get-ChildItem -Path $NidaqBin -Filter "NIDAQmx*.dll" | ForEach-Object {
        Copy-Item -Path $_.FullName -Destination $packageDir -Force
    }
}

# Copy model assets.
if (Test-Path $ModelsDir) {
    $modelsOut = Join-Path $packageDir "models"
    New-Item -ItemType Directory -Path $modelsOut -Force | Out-Null
    Copy-Item -Path (Join-Path $ModelsDir "*.onnx") -Destination $modelsOut -Force -ErrorAction SilentlyContinue
    Copy-Item -Path (Join-Path $ModelsDir "metadata.json") -Destination $modelsOut -Force -ErrorAction SilentlyContinue
}

# Deploy Qt runtime and plugins next to the exe.
& $windeployqt (Join-Path $packageDir "droplet_pipeline.exe") | Out-Host

Write-Host "Portable package created at: $packageDir"
Write-Output $packageDir
