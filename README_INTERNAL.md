# Internal Architecture Notes

This document describes how the C++ pipeline is organized and how data flows.

## Data flow (runtime)
1) DCAM capture (Hamamatsu) produces frames in grayscale.
2) Fast event detector builds/updates a rolling background, thresholds the diff, and gates detections.
3) When an event fires, the crop is squared and passed to the ONNX classifier.
4) If the predicted label matches the target label, the NI-DAQ analog output fires.

The same flow is used in both the CLI mode (`droplet_pipeline.exe --cli`) and the Qt GUI
pipeline (`droplet_pipeline.exe`).
The headless sequence runner (`sequence_headless.exe`) uses the same pipeline runner
and adds trajectory tracking plus per-run summaries.

## Key files (cpp_pipeline)
- `cli_runner.h/.cpp`  
  CLI runner: camera -> fast/precise detection -> ONNX -> NI-DAQ.
  Invoked with `droplet_pipeline.exe --cli` and CLI args for camera/detection/DAQ config.

- `sequence_headless/sequence_headless.cpp`  
  Headless sequence test runner for folder input + target FPS.
  Batches loading based on available RAM and writes sequence logs, trajectories, and summary CSVs.

- `fast_event_detector.h/.cpp`  
  Fast detection logic ported from `components/sequence_event_detection.cpp`.  
  Handles background mean, rolling updates, thresholding, morphology, and gating with gap-fire logic.

- `event_detector.h/.cpp`  
  Precise detection path (background subtraction + Otsu + contour filtering).

- `onnx_classifier.h/.cpp`  
  ONNX Runtime wrapper that resizes input, normalizes, and runs inference.

- `metadata_loader.h/.cpp`  
  Minimal JSON parser for classes, input size, and normalization parameters.

- `daq_trigger.h/.cpp`  
  NI-DAQ analog output wrapper (sine wave). Stubs when DAQmx is not available.

- `dcam_camera.h/.cpp`  
  Headless DCAM camera wrapper used by the CLI runner.

- `models/`  
  Local ONNX models for packaging (e.g., `squeezenet_final_new_condition.onnx`).

## Qt GUI (cpp_pipeline/qt_hama_gui)
- `main.cpp`  
  Live viewer with camera controls, save/record tools, and the Pipeline tab.
  The Pipeline tab loads ONNX + metadata, runs the fast detector, and triggers NI-DAQ.

- `pipeline_runner.h/.cpp`  
  GUI-side pipeline runner: FastEventDetector + OnnxClassifier + DaqTrigger.
  Processes frames from `FrameGrabber` (non-UI thread) and posts status to the UI.

- `dcam_controller.h/.cpp`  
  DCAM device controller for the Qt app (open, apply, lock frame).

- `frame_grabber.h/.cpp`  
  Worker thread that waits for frames and emits them to the UI / pipeline.

- `frame_types.h`  
  Shared structs for camera settings and frame metadata.

- `log_teebuf.h`  
  Tee buffer for logging to both console and GUI log output.

## Notes
- Fast-mode defaults are aligned with the last `sequence_event_detection` tuning.
- Place `metadata.json` next to the ONNX model or point the GUI/CLI to the correct path.
- Sequence outputs include per-frame logs, per-event trajectory CSVs, and summary CSVs with
  efficiency metrics derived from label vs. hit/waste decisions.
