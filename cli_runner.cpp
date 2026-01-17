#include "cli_runner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "daq_trigger.h"
#include "dcam_camera.h"
#include "event_detector.h"
#include "fast_event_detector.h"
#include "metadata_loader.h"
#include "onnx_classifier.h"

namespace fs = std::filesystem;

struct Args {
    std::unordered_map<std::string, std::string> values;
    std::unordered_set<std::string> flags;
};

Args parseArgs(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        std::string key = argv[i];
        if (key.rfind("--", 0) == 0) {
            if (i + 1 < argc) {
                std::string next = argv[i + 1];
                if (next.rfind("--", 0) != 0) {
                    args.values[key] = next;
                    ++i;
                    continue;
                }
            }
            args.flags.insert(key);
        }
    }
    return args;
}

bool hasFlag(const Args& args, const std::string& key) {
    return args.flags.find(key) != args.flags.end();
}

std::string getString(const Args& args, const std::string& key, const std::string& def = "") {
    auto it = args.values.find(key);
    return it != args.values.end() ? it->second : def;
}

std::string toLowerAscii(const std::string& input) {
    std::string out;
    out.reserve(input.size());
    for (unsigned char c : input) {
        out.push_back(static_cast<char>(std::tolower(c)));
    }
    return out;
}

constexpr int kCropSize = 64;

cv::Rect makeSquareRect(const cv::Rect& bbox, const cv::Size& size) {
    if (bbox.width <= 0 || bbox.height <= 0 || size.width <= 0 || size.height <= 0) {
        return cv::Rect();
    }
    int side = std::max(bbox.width, bbox.height);
    side = std::min(side, std::min(size.width, size.height));
    int cx = bbox.x + bbox.width / 2;
    int cy = bbox.y + bbox.height / 2;
    int x = cx - side / 2;
    int y = cy - side / 2;
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x + side > size.width) x = size.width - side;
    if (y + side > size.height) y = size.height - side;
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    return cv::Rect(x, y, side, side);
}

int getInt(const Args& args, const std::string& key, int def) {
    auto it = args.values.find(key);
    if (it == args.values.end()) return def;
    return std::stoi(it->second);
}

double getDouble(const Args& args, const std::string& key, double def) {
    auto it = args.values.find(key);
    if (it == args.values.end()) return def;
    return std::stod(it->second);
}

bool pathExists(const fs::path& path) {
    std::error_code ec;
    return fs::exists(path, ec);
}

fs::path getExeDir(const char* argv0) {
    if (!argv0 || std::string(argv0).empty()) {
        return fs::current_path();
    }
    std::error_code ec;
    fs::path exePath = fs::absolute(fs::path(argv0), ec);
    if (ec) {
        return fs::current_path();
    }
    return exePath.parent_path();
}

std::string resolveExistingPath(const std::vector<fs::path>& candidates) {
    for (const auto& path : candidates) {
        if (pathExists(path)) {
            return path.string();
        }
    }
    return "";
}

void printUsage() {
    std::cout <<
        "Usage:\n"
        "  droplet_pipeline [--onnx <model.onnx>] [--metadata <metadata.json>] [options]\n"
        "\n"
        "Defaults:\n"
        "  --onnx     models/squeezenet_final_new_condition.onnx\n"
        "  --metadata models/metadata.json\n"
        "\n"
        "Options:\n"
        "  --device <index>              Camera index (default: 0)\n"
        "  --width <px>                  Subarray width (default: 0 = full)\n"
        "  --height <px>                 Subarray height (default: 0 = full)\n"
        "  --binning <n>                 Binning (default: 1)\n"
        "  --bits <n>                    Bit depth (default: 12)\n"
        "  --pixel-type <mono8|mono16>   Pixel type (default: mono8)\n"
        "  --exposure-ms <ms>            Exposure in ms (default: 5)\n"
        "  --readout <fastest|slowest>   Readout speed (default: fastest)\n"
        "  --camera-trigger <internal|external|software|master>  Trigger source (default: internal)\n"
        "  --timeout-ms <ms>             Frame wait timeout (default: 1000)\n"
        "  --bg-frames <n>               Frames for background (default: 50)\n"
        "  --bg-mode <mean|median>       Background mode (default: mean)\n"
        "  --bg-update-frames <n>        Rolling background window (fast mode, default: 50)\n"
        "  --min-area <px>               Min blob area (default: 40)\n"
        "  --min-area-frac <f>           Min blob area fraction (default: 0.0)\n"
        "  --max-area-frac <f>           Max blob area fraction (default: 0.10)\n"
        "  --min-bbox <px>               Min bbox width/height (default: 32)\n"
        "  --margin <px>                 Border margin (default: 5)\n"
        "  --sigma <f>                   Gaussian sigma (default: 1.0)\n"
        "  --detect-mode <fast|precise>  Detection mode (default: fast)\n"
        "  --diff-thresh <0-255>         Fixed diff threshold (fast mode, default: 15)\n"
        "  --blur-radius <px>            Box blur radius (fast mode, default: 1)\n"
        "  --morph-radius <px>           Morph radius (fast mode, default: 1)\n"
        "  --scale <f>                   Downscale factor (fast mode, default: 0.5)\n"
        "  --frame-skip <n>              Skip n frames between checks (default: 0)\n"
        "  --reset-frames <n>            Frames with no detection to reset gate (default: 3)\n"
        "  --gap-fire-shift <px>         Min centroid shift to fire after gap (fast mode)\n"
        "  --target-label <name>         Label that triggers DAQ (default: Single)\n"
        "  --daq-channel <Dev1/ao0>      Analog output channel (default: Dev1/ao0)\n"
        "  --daq-amp <V>                 Sine amplitude (default: 5)\n"
        "  --daq-freq <Hz>               Sine frequency (default: 1000)\n"
        "  --daq-duration-ms <ms>        Sine duration (default: 5)\n"
        "  --daq-delay-ms <ms>           Delay before output (default: 0)\n"
        "  --output-dir <dir>            Save event crops/overlays\n"
        "  --save-overlay                Save overlay image with bbox/mask\n"
        "  --continuous                  Do not stop after first event\n"
        "  --help                        Show this help\n";
}

int parsePixelType(const std::string& s) {
    if (s == "mono16") return DCAM_PIXELTYPE_MONO16;
    return DCAM_PIXELTYPE_MONO8;
}

int parseReadout(const std::string& s) {
    if (s == "slowest") return DCAMPROP_READOUTSPEED__SLOWEST;
    return DCAMPROP_READOUTSPEED__FASTEST;
}

int parseCameraTriggerSource(const std::string& s) {
    if (s == "external") return DCAMPROP_TRIGGERSOURCE__EXTERNAL;
    if (s == "software") return DCAMPROP_TRIGGERSOURCE__SOFTWARE;
    if (s == "master") return DCAMPROP_TRIGGERSOURCE__MASTERPULSE;
    return DCAMPROP_TRIGGERSOURCE__INTERNAL;
}

cv::Mat toGray8(const cv::Mat& src, int bits) {
    cv::Mat gray = src;
    if (src.channels() == 3) {
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    }
    cv::Mat out;
    if (gray.type() == CV_8UC1) {
        out = gray;
    } else if (gray.type() == CV_16UC1) {
        int b = std::max(1, std::min(bits, 16));
        double scale = 255.0 / (static_cast<double>((1 << b) - 1));
        gray.convertTo(out, CV_8U, scale);
    } else {
        gray.convertTo(out, CV_8U);
    }
    return out;
}


int run_cli(int argc, char** argv) {
    Args args = parseArgs(argc, argv);
    if (hasFlag(args, "--help")) {
        printUsage();
        return 0;
    }

    std::string onnxPath = getString(args, "--onnx");
    std::string metaPath = getString(args, "--metadata");

    fs::path exeDir = getExeDir(argv[0]);
    const std::string defaultOnnxName = "squeezenet_final_new_condition.onnx";
    const std::string defaultMetaName = "metadata.json";

    if (onnxPath.empty()) {
        std::vector<fs::path> candidates = {
            fs::current_path() / "models" / defaultOnnxName,
            fs::current_path() / "cpp_pipeline" / "models" / defaultOnnxName,
            exeDir / "models" / defaultOnnxName,
            exeDir / ".." / ".." / "models" / defaultOnnxName
        };
        onnxPath = resolveExistingPath(candidates);
    }

    if (metaPath.empty()) {
        std::vector<fs::path> candidates;
        if (!onnxPath.empty()) {
            candidates.push_back(fs::path(onnxPath).parent_path() / defaultMetaName);
        }
        candidates.push_back(fs::current_path() / "models" / defaultMetaName);
        candidates.push_back(fs::current_path() / "cpp_pipeline" / "models" / defaultMetaName);
        candidates.push_back(exeDir / "models" / defaultMetaName);
        candidates.push_back(exeDir / ".." / ".." / "models" / defaultMetaName);
        metaPath = resolveExistingPath(candidates);
    }

    if (onnxPath.empty() || metaPath.empty()) {
        if (onnxPath.empty()) {
            std::cerr << "Missing --onnx and default model not found.\n";
        }
        if (metaPath.empty()) {
            std::cerr << "Missing --metadata and default metadata not found.\n";
        }
        printUsage();
        return 1;
    }

    if (!args.values.count("--onnx")) {
        std::cout << "Using default ONNX: " << onnxPath << "\n";
    }
    if (!args.values.count("--metadata")) {
        std::cout << "Using default metadata: " << metaPath << "\n";
    }
    std::string outputDir = getString(args, "--output-dir");
    bool saveOverlay = hasFlag(args, "--save-overlay");
    bool continuous = hasFlag(args, "--continuous");

    DcamCamera camera;
    int deviceIndex = getInt(args, "--device", 0);
    std::string err = camera.init(deviceIndex);
    if (!err.empty()) {
        std::cerr << "Camera init error: " << err << "\n";
        return 1;
    }

    auto shutdownCamera = [&camera]() {
        camera.stop();
        camera.cleanup();
    };

    CameraSettings settings;
    settings.width = getInt(args, "--width", 0);
    settings.height = getInt(args, "--height", 0);
    settings.binning = getInt(args, "--binning", 1);
    settings.bits = getInt(args, "--bits", 12);
    settings.pixelType = parsePixelType(getString(args, "--pixel-type", "mono8"));
    settings.exposureMs = getDouble(args, "--exposure-ms", 5.0);
    settings.readoutSpeed = parseReadout(getString(args, "--readout", "fastest"));
    settings.triggerSource = parseCameraTriggerSource(getString(args, "--camera-trigger", "internal"));
    settings.enableSubarray = settings.width > 0 && settings.height > 0;

    err = camera.apply(settings);
    if (!err.empty()) {
        std::cout << "Camera apply: " << err << "\n";
    }
    err = camera.start();
    if (!err.empty()) {
        std::cerr << "Camera start error: " << err << "\n";
        shutdownCamera();
        return 1;
    }

    Metadata meta;
    if (!LoadMetadata(metaPath, meta, err)) {
        std::cerr << "Metadata error: " << err << "\n";
        shutdownCamera();
        return 1;
    }

    OnnxClassifier classifier;
    if (!classifier.init(onnxPath, meta, err)) {
        std::cerr << "ONNX init error: " << err << "\n";
        shutdownCamera();
        return 1;
    }

    int bgFrames = getInt(args, "--bg-frames", 50);
    int timeoutMs = getInt(args, "--timeout-ms", 1000);
    std::string detectMode = getString(args, "--detect-mode", "fast");
    bool fastMode = (detectMode == "fast");
    bool resetFramesSet = args.values.find("--reset-frames") != args.values.end();
    bool minAreaSet = args.values.find("--min-area") != args.values.end();
    int resetFrames = resetFramesSet ? getInt(args, "--reset-frames", 3) : (fastMode ? 2 : 3);
    double minArea = minAreaSet ? getDouble(args, "--min-area", 40.0) : -1.0;
    double minAreaFrac = getDouble(args, "--min-area-frac", 0.0);
    double maxAreaFrac = getDouble(args, "--max-area-frac", 0.10);
    int minBbox = getInt(args, "--min-bbox", 32);
    int borderMargin = getInt(args, "--margin", 5);
    double gaussSigma = getDouble(args, "--sigma", 1.0);
    int diffThresh = getInt(args, "--diff-thresh", 15);
    int blurRadius = getInt(args, "--blur-radius", 1);
    int morphRadius = getInt(args, "--morph-radius", 1);
    double scale = getDouble(args, "--scale", 0.5);
    int bgUpdateFrames = getInt(args, "--bg-update-frames", 50);
    int gapFireShift = getInt(args, "--gap-fire-shift", 0);

    if (scale <= 0.0 || scale > 1.0) {
        std::cerr << "scale must be in (0, 1]\n";
        return 1;
    }
    if (minAreaFrac < 0.0 || minAreaFrac > 1.0) {
        std::cerr << "min-area-frac must be in [0, 1]\n";
        return 1;
    }
    if (bgUpdateFrames < 0) {
        std::cerr << "bg-update-frames must be >= 0\n";
        return 1;
    }

    EventDetectorConfig detCfg;
    EventDetector detector(detCfg);

    FastEventConfig fastCfg;
    fastCfg.bgFrames = bgFrames;
    fastCfg.bgUpdateFrames = bgUpdateFrames;
    fastCfg.resetFrames = resetFrames;
    fastCfg.minArea = minArea;
    fastCfg.minAreaFrac = minAreaFrac;
    fastCfg.maxAreaFrac = maxAreaFrac;
    fastCfg.minBbox = minBbox;
    fastCfg.margin = borderMargin;
    fastCfg.diffThresh = diffThresh;
    fastCfg.blurRadius = blurRadius;
    fastCfg.morphRadius = morphRadius;
    fastCfg.scale = scale;
    fastCfg.gapFireShift = gapFireShift;
    FastEventDetector fastDetector(fastCfg);

    std::vector<cv::Mat> bgStack;
    if (fastMode) {
        int attempts = 0;
        int maxAttempts = std::max(5, bgFrames * 5);
        int bgTimeouts = 0;
        int lastRemaining = fastDetector.backgroundFramesRemaining();
        while (!fastDetector.isReady() && attempts < maxAttempts) {
            if (!camera.waitForFrame(timeoutMs)) {
                bgTimeouts++;
                if (bgTimeouts % 10 == 0) {
                    std::cout << "Background wait timeout (" << bgTimeouts << ")\n";
                }
                attempts++;
                continue;
            }
            FrameData fd;
            if (!camera.getLatestFrame(fd)) {
                attempts++;
                continue;
            }
            cv::Mat gray8 = toGray8(fd.image, fd.meta.bits);
            fastDetector.addBackgroundFrame(gray8);
            int remaining = fastDetector.backgroundFramesRemaining();
            if (remaining != lastRemaining) {
                std::cout << "Background frames remaining: " << remaining << "\n";
                lastRemaining = remaining;
            }
            attempts++;
        }
        if (!fastDetector.isReady()) {
            std::cerr << "Background error: fast background mean is empty\n";
            shutdownCamera();
            return 1;
        }
    } else {
        int initFrames = bgFrames;
        bgStack.reserve(initFrames);
        int collected = 0;
        int bgTimeouts = 0;
        for (int i = 0; i < initFrames; ++i) {
            if (!camera.waitForFrame(timeoutMs)) {
                bgTimeouts++;
                if (bgTimeouts % 10 == 0) {
                    std::cout << "Background wait timeout (" << bgTimeouts << ")\n";
                }
                continue;
            }
            FrameData fd;
            if (!camera.getLatestFrame(fd)) continue;
            cv::Mat gray8 = toGray8(fd.image, fd.meta.bits);
            bgStack.push_back(gray8);
            collected++;
            if (collected % 5 == 0 || collected == initFrames) {
                std::cout << "Background frames collected: " << collected << "/" << initFrames << "\n";
            }
        }
        detCfg.minArea = static_cast<int>(minArea < 0.0 ? 40.0 : minArea);
        detCfg.maxAreaFrac = maxAreaFrac;
        detCfg.borderMargin = borderMargin;
        detCfg.sigma = gaussSigma;
        detCfg.bgMode = getString(args, "--bg-mode", "mean");
        detector = EventDetector(detCfg);
        if (!detector.buildBackground(bgStack, err)) {
            std::cerr << "Background error: " << err << "\n";
            shutdownCamera();
            return 1;
        }
    }

    if (!outputDir.empty()) {
        fs::create_directories(outputDir);
        if (fastMode) {
            cv::imwrite((fs::path(outputDir) / "background.png").string(), fastDetector.background());
        } else {
            cv::Mat bgVis;
            detector.background().convertTo(bgVis, CV_8U, 255.0);
            cv::imwrite((fs::path(outputDir) / "background.png").string(), bgVis);
        }
    }

    DaqTrigger trigger;
    DaqConfig daq;
    daq.channel = getString(args, "--daq-channel", "Dev1/ao0");
    daq.rangeMin = -10.0;
    daq.rangeMax = 10.0;
    daq.amplitude = getDouble(args, "--daq-amp", 5.0);
    daq.frequencyHz = getDouble(args, "--daq-freq", 1000.0);
    daq.durationMs = getDouble(args, "--daq-duration-ms", 5.0);
    daq.delayMs = getDouble(args, "--daq-delay-ms", 0.0);
    if (!daq.channel.empty()) {
        if (!trigger.init(daq, err)) {
            std::cout << "DAQ init disabled: " << err << "\n";
        }
    }

    std::string targetLabel = getString(args, "--target-label", "Single");
    std::string targetLabelLower = toLowerAscii(targetLabel);
    int frameSkip = std::max(0, getInt(args, "--frame-skip", 0));

    int64_t frameCounter = 0;
    int fpsCount = 0;
    double fps = 0.0;
    auto fpsStart = std::chrono::steady_clock::now();
    bool triggeredPrecise = false;
    int noDetectCountPrecise = 0;

    int timeoutCount = 0;
    while (true) {
        if (!camera.waitForFrame(timeoutMs)) {
            timeoutCount++;
            if (timeoutCount % 10 == 0) {
                std::cout << "Camera wait timeout (" << timeoutCount << ")\n";
            }
            continue;
        }
        timeoutCount = 0;
        FrameData fd;
        if (!camera.getLatestFrame(fd)) continue;
        frameCounter++;
        if (frameSkip > 0 && (frameCounter % (frameSkip + 1)) != 0) {
            continue;
        }

        cv::Mat gray8 = toGray8(fd.image, fd.meta.bits);
        bool detected = false;
        bool fired = false;
        EventResult ev;
        FastEventResult fastEv;

        if (fastMode) {
            fastDetector.processFrame(gray8, fastEv);
            detected = fastEv.detected;
            fired = fastEv.fired;
        } else {
            ev = detector.detect(gray8, saveOverlay);
            detected = ev.detected;
            if (detected) {
                double imgArea = static_cast<double>(gray8.rows) * static_cast<double>(gray8.cols);
                if (ev.area < (minAreaFrac * imgArea) ||
                    ev.bbox.width < minBbox ||
                    ev.bbox.height < minBbox) {
                    detected = false;
                }
            }

            if (detected) {
                noDetectCountPrecise = 0;
                if (!triggeredPrecise) {
                    fired = true;
                    triggeredPrecise = true;
                }
            } else if (triggeredPrecise) {
                noDetectCountPrecise++;
                if (noDetectCountPrecise >= resetFrames) {
                    triggeredPrecise = false;
                    noDetectCountPrecise = 0;
                }
            }
        }

        fpsCount++;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - fpsStart).count();
        if (elapsed >= 1.0) {
            fps = fpsCount / elapsed;
            fpsCount = 0;
            fpsStart = now;
        }

        if (!fired) continue;

        cv::Rect bbox = fastMode ? fastEv.bbox : ev.bbox;
        bbox &= cv::Rect(0, 0, gray8.cols, gray8.rows);
        cv::Rect squareRect = makeSquareRect(bbox, gray8.size());
        if (squareRect.width <= 0 || squareRect.height <= 0) continue;
        cv::Mat crop = gray8(squareRect).clone();
        ClassificationResult cls = classifier.classify(crop);

        double areaOut = fastMode ? fastEv.area : ev.area;
        std::cout << "[Event] frame=" << frameCounter
                  << " label=" << cls.label
                  << " area=" << areaOut
                  << " bbox=(" << bbox.x << "," << bbox.y << "," << bbox.width << "," << bbox.height << ")"
                  << " fps=" << fps << "\n";

        if (!outputDir.empty()) {
            fs::path base = fs::path(outputDir);
            std::string name = "event_frame_" + std::to_string(frameCounter) + "_label_" + cls.label;
            cv::Mat resized;
            cv::resize(crop, resized, cv::Size(kCropSize, kCropSize), 0, 0, cv::INTER_AREA);
            cv::imwrite((base / (name + ".png")).string(), resized);
            if (saveOverlay) {
                cv::Mat overlay;
                cv::cvtColor(gray8, overlay, cv::COLOR_GRAY2BGR);
                cv::rectangle(overlay, squareRect, cv::Scalar(0, 255, 0), 2);
                if (!fastMode && !ev.mask.empty()) {
                    cv::Mat colorMask;
                    cv::applyColorMap(ev.mask, colorMask, cv::COLORMAP_JET);
                    cv::addWeighted(overlay, 0.6, colorMask, 0.4, 0.0, overlay);
                }
                cv::imwrite((base / (name + "_overlay.png")).string(), overlay);
            }
        }

        if (!cls.label.empty() && trigger.isReady()) {
            if (toLowerAscii(cls.label) == targetLabelLower) {
                std::string trigErr;
                if (!trigger.fire(trigErr)) {
                    std::cout << "Trigger failed: " << trigErr << "\n";
                }
            }
        }

        if (!continuous) break;
    }

    trigger.shutdown();
    shutdownCamera();
    return 0;
}
