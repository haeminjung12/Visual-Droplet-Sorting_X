# Build

Use **Visual Studio MSBuild only**. Do **not** search for other compilers or use non‑VS toolchains.

Run from the repo root:

```powershell
& "C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe" "build\droplet_pipeline.sln" /m /p:Configuration=Release
```

# Dependencies (verify these directories exist)

- OpenCV (via vcpkg):
  - `C:\vcpkg\installed\x64-windows\include\opencv4`
  - `C:\vcpkg\installed\x64-windows\bin`
- ONNX Runtime (GPU build):
  - `C:\onnxruntime-gpu\include`
  - `C:\onnxruntime-gpu\lib`
- Hamamatsu DCAM SDK:
  - `C:\Users\goals\Codex\CNN for Droplet Sorting\python_pipeline\Hamamatsu_DCAMSDK4_v25056964\dcamsdk4\inc`
  - `C:\Users\goals\Codex\CNN for Droplet Sorting\python_pipeline\Hamamatsu_DCAMSDK4_v25056964\dcamsdk4\lib\win64`

If any of these differ on your machine, update the CMake cache (e.g. `ONNXRUNTIME_DIR` or `VCPKG_ROOT`) and reconfigure, but still build with MSBuild.

# Agent Git workflow

Goal
- main must always compile
- agents work in isolated branches
- agents may commit locally during work
- agents must not push until the task is complete

Start
1. Update main
- git checkout main
- git pull origin main

2. Create a new branch for this agent and task
- git checkout -b <agent>/<task>

Do not push at start.

Work
- Make changes locally
- Commit locally as needed to save progress
- Do not push during work

Finish
1. Make sure your branch is up to date with main
- git fetch origin
- git merge origin/main
- resolve conflicts if needed
- run build and tests locally

2. Push your branch once
- git push -u origin <agent>/<task>

3. Open a pull request from <agent>/<task> into main
- merge only after checks pass
- prefer squash merge for a clean main history

Cleanup
- delete the branch after the PR is merged