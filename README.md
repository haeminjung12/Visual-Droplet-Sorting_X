Droplet pipeline (C++)

This repository contains the full C++ pipeline for high-speed droplet sorting:
Hamamatsu DCAM frame capture -> event detection -> ONNX classification -> NI-DAQ analog output.
It includes the Qt GUI (primary entry point) and a CLI mode via the same executable.

Overview and data flow
1) Camera capture (Hamamatsu DCAM)
   - Frames are acquired from the camera SDK into a DCAM buffer.
   - A frame grabber thread pulls the newest frame and passes it to the pipeline.
2) Event detection
   - Background model is maintained and updated over time.
   - Foreground mask is computed by background subtraction + thresholding.
   - Blob selection and gating find a droplet candidate.
3) Crop + inference
   - The droplet region is cropped (square) and resized to 64x64.
   - ONNX Runtime runs classification (Empty / Single / MoreThanTwo).
4) Trigger output
   - If the classifier result matches target label (e.g., Single), the trigger fires.
   - Trigger is an analog sine wave using NI-DAQmx AO (configurable amplitude/frequency/duration/delay).

Project layout (every source file)
Root level (shared logic + CLI runner)
- CMakeLists.txt
  - Top-level build script. Configures vcpkg, OpenCV, ONNX Runtime, DCAM, NI-DAQmx.
  - Builds the Qt GUI subdirectory.
- cli_runner.h / cli_runner.cpp
  - CLI runner (invoked when the GUI exe is started with `--cli`).
  - Creates camera, detector, classifier, and trigger, then runs the loop.
- dcam_camera.h / dcam_camera.cpp
  - Minimal DCAM capture wrapper for the CLI.
  - Handles camera setup, frame acquisition, and exposure/bit depth.
- fast_event_detector.h / fast_event_detector.cpp
  - Fast background subtraction + blob detection algorithm.
  - Maintains background mean, computes difference, applies blur/morph, thresholds.
  - Filters by area/bbox and returns best droplet candidate and crop rectangle.
- event_detector.h / event_detector.cpp
  - Slower/more robust detection path (Otsu + additional filtering).
  - Used by the CLI when not in fast mode.
  - Gate logic so a single droplet only fires once per event.
  - Resets after a configured number of missed frames.
- metadata_loader.h / metadata_loader.cpp
  - Reads metadata.json for class names, input size, mean/std normalization.
  - Defines how the ONNX input is normalized.
- onnx_classifier.h / onnx_classifier.cpp
  - ONNX Runtime wrapper. Loads model and runs inference.
  - Handles CPU/GPU providers depending on ONNXRUNTIME build.
- daq_trigger.h / daq_trigger.cpp
  - NI-DAQmx wrapper for analog output (sine wave).
  - If NI-DAQmx is not found at build time, compiled as stub (no hardware).
- README_INTERNAL.md
  - Internal architecture notes, data flow, and module interactions.

Qt GUI (qt_hama_gui/)
- qt_hama_gui/CMakeLists.txt
  - Builds the Qt GUI app from these sources and links dependencies.
- qt_hama_gui/main.cpp
  - GUI entry point.
  - Tabs: camera settings, save controls, pipeline config, LabVIEW/DAQ, event detection settings,
    stats, and sequence test.
  - Displays live view with zoom/pan and exposes all pipeline controls.
  - Drives pipeline runner, logging, and stats accumulation.
- qt_hama_gui/dcam_controller.h / dcam_controller.cpp
  - DCAM camera controller for the GUI.
  - Applies exposure, resolution, binning, bit depth, and readout speed.
- qt_hama_gui/frame_grabber.h / frame_grabber.cpp
  - Grabs frames on a worker thread.
  - Emits deep-copied QImage to GUI thread for stable rendering.
- qt_hama_gui/frame_types.h
  - Frame metadata (size, bits, FPS, delivered/dropped counts).
- qt_hama_gui/pipeline_runner.h / pipeline_runner.cpp
  - Pipeline wrapper used by GUI and sequence test.
  - Connects detector, ONNX classifier, and trigger.
  - Returns PipelineEvent for UI display, logging, and stats.
- qt_hama_gui/log_teebuf.h
  - Simple stdout/Qt log tee so messages are mirrored to session_log.txt.
- qt_hama_gui/README.md
  - GUI-specific build notes.

GUI features (what it does and how)
- Live view
  - Shows the latest frame with zoom/pan.
  - Updates stats panel with frame index, FPS, and camera settings.
- Pipeline controls
  - Select ONNX model and metadata JSON.
  - Configure output directory and whether to save crops/overlays.
  - Enable/disable pipeline without stopping camera.
- LabVIEW / NI-DAQ controls
  - Shows connection status light and output configuration.
  - Configure AO channel, amplitude, frequency (kHz), duration, and delay.
  - Force Trigger button emits the configured sine output.
- Event detection controls
  - All fast detector parameters exposed: background frames, update frames,
    reset frames, min area, min/max area fraction, min bbox, margin, diff threshold,
    blur/morph radius, scale, and gap/fire shift.
- Stats tab
  - Counts total events and class counts.
  - Tracks hit vs waste based on droplet motion.
- Stats figures
  - Generates hit/waste pie chart and class distribution pie chart.
  - Save images to output folder.
- Sequence Test tab
  - Loads a folder of images into memory.
  - Feeds frames at a precise target FPS into the same pipeline path as live mode.
  - Writes a detailed CSV log for each frame (timing, detection, classification, trigger).

Event detection details
- Background mean is computed from the first N frames (bgFrames).
- Dynamic background update uses the last N non-event frames (bgUpdateFrames).
- Foreground mask = abs(frame - background) > diffThresh.
- Noise reduction with blur + morphology (blurRadius, morphRadius).
- Contours are filtered by min area, max area fraction, and min bbox size.
- Cropping region is padded by margin and then resized to 64x64 for inference.
- Event gating prevents multiple triggers while the droplet is in view.

Hit/Waste decision
- During an event, the droplet centroid is tracked.
- Decision is made when the event ends (after resetFrames misses).
- Direction is inferred from cumulative Y movement and final position:
  - Moving toward top: Waste
  - Moving toward bottom: Hit
- Decision frame is logged in both sequence and live CSVs.

Logging and outputs
- session_log.txt
  - App startup, pipeline init, camera, DAQ status, and warnings.
- Live pipeline CSV
  - Written when the pipeline is stopped.
  - Includes per-frame timing, detection, inference, trigger, and hit/waste decision data.
- Sequence test CSV
  - Written after sequence test completes.
  - Includes scheduling jitter, per-frame processing time, and pipeline results.
- Per-run folders
  - Each live run creates: pipeline_output/live_YYYYMMDD_hhmmss
  - Each sequence test creates: pipeline_output/sequence_YYYYMMDD_hhmmss
  - Crops, overlays, logs, and charts are saved inside that folder.

Build requirements
- Visual Studio 2022 (MSVC)
- CMake 3.19+
- OpenCV (core, imgproc, imgcodecs)
- ONNX Runtime (GPU or CPU)
- Hamamatsu DCAM SDK (dcamapi)
- NI-DAQmx (optional, for trigger output)

Build (PowerShell)
1) Configure paths (examples):
   - DCAM SDK: -D DCAM_SDK_DIR="C:/path/to/dcamsdk4"
   - ONNX Runtime: -D ONNXRUNTIME_DIR="C:/onnxruntime-gpu"
   - NI-DAQmx: -D NIDAQMX_DIR="C:/Program Files (x86)/National Instruments/NI-DAQ/DAQmx ANSI C Dev"

2) Configure (CMake generates the build folder) and build (MSBuild):
   cmake -S . -B build ^
     -D ONNXRUNTIME_DIR="C:/onnxruntime-gpu" ^
     -D DCAM_SDK_DIR="C:/path/to/dcamsdk4" ^
     -D NIDAQMX_DIR="C:/Program Files (x86)/National Instruments/NI-DAQ/DAQmx ANSI C Dev"
   & "C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe" "build\droplet_pipeline.sln" /m /p:Configuration=Release

Run
- GUI:
  build/qt_hama_gui/Release/droplet_pipeline.exe
- CLI (example, uses same exe):
  build/qt_hama_gui/Release/droplet_pipeline.exe --cli ^
    --onnx "C:/path/to/squeezenet_final_new_condition.onnx" ^
    --metadata "C:/path/to/metadata.json" ^
    --width 1152 --height 1152 --binning 1 --bits 12 --exposure-ms 5 ^
    --bg-frames 50 --min-area 40 --max-area-frac 0.10 --margin 5 --sigma 1.0 ^
    --target-label Single ^
    --daq-channel Dev1/ao0 --daq-amp 5 --daq-freq 10000 --daq-duration-ms 2.0 --daq-delay-ms 0 ^
    --output-dir pipeline_output

Notes
- If NI-DAQmx is not found at build time, triggers are stubbed and no hardware pulse is emitted.
- ONNX metadata.json defines input size and normalization; keep it alongside the model.
- The GUI is the primary entry point for live operation and sequence testing.
