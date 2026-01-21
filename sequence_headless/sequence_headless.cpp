#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifdef _WIN32
#include <windows.h>
#endif

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "../qt_hama_gui/pipeline_runner.h"

namespace fs = std::filesystem;

struct Args {
    std::vector<std::string> positional;
    std::vector<std::string> flags;
    std::unordered_map<std::string, std::string> values;
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
            args.flags.push_back(key);
        } else {
            args.positional.push_back(key);
        }
    }
    return args;
}

bool hasFlag(const Args& args, const std::string& key) {
    return std::find(args.flags.begin(), args.flags.end(), key) != args.flags.end();
}

std::string getValue(const Args& args, const std::string& key, const std::string& def = "") {
    auto it = args.values.find(key);
    return it != args.values.end() ? it->second : def;
}

bool parseDouble(const std::string& text, double& out) {
    try {
        size_t idx = 0;
        out = std::stod(text, &idx);
        return idx == text.size();
    } catch (...) {
        return false;
    }
}

bool parseInt(const std::string& text, int& out) {
    try {
        size_t idx = 0;
        out = std::stoi(text, &idx);
        return idx == text.size();
    } catch (...) {
        return false;
    }
}

std::string toLowerAscii(std::string s) {
    for (auto& c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

bool hasImageExt(const fs::path& path) {
    std::string ext = toLowerAscii(path.extension().string());
    return ext == ".tif" || ext == ".tiff" || ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp";
}

std::vector<fs::path> collectImageFiles(const fs::path& dir) {
    std::vector<fs::path> files;
    std::error_code ec;
    if (!fs::exists(dir, ec) || !fs::is_directory(dir, ec)) {
        return files;
    }
    for (const auto& entry : fs::directory_iterator(dir, ec)) {
        if (ec) break;
        if (!entry.is_regular_file()) continue;
        const fs::path& path = entry.path();
        if (hasImageExt(path)) {
            files.push_back(path);
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

bool loadImageGrayscale8(const fs::path& path, cv::Mat& out) {
    cv::Mat img = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
    if (img.empty()) return false;
    if (img.channels() == 3) {
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    } else if (img.channels() == 4) {
        cv::cvtColor(img, img, cv::COLOR_BGRA2GRAY);
    }
    if (img.depth() == CV_16U) {
        img.convertTo(out, CV_8U, 255.0 / 65535.0);
    } else if (img.depth() == CV_8U) {
        out = img;
    } else {
        img.convertTo(out, CV_8U);
    }
    if (out.type() != CV_8UC1) {
        out = out.clone();
    }
    return true;
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
        std::error_code ec;
        if (fs::exists(path, ec)) {
            return path.string();
        }
    }
    return "";
}

std::string formatTimestamp(const std::chrono::system_clock::time_point& tp) {
    auto t = std::chrono::system_clock::to_time_t(tp);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

std::string formatWallTime() {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "."
        << std::setw(3) << std::setfill('0') << ms.count();
    return oss.str();
}

std::string csvQuote(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 2);
    out.push_back('"');
    for (char c : s) {
        if (c == '"') out.push_back('"');
        out.push_back(c);
    }
    out.push_back('"');
    return out;
}

std::string normalizeLabel(const std::string& label) {
    return label.empty() ? "(unclassified)" : label;
}

void printProgress(size_t current, size_t total) {
    if (total == 0) return;
    double pct = 100.0 * static_cast<double>(current) / static_cast<double>(total);
    std::cout << "\rProgress: " << std::fixed << std::setprecision(1)
              << pct << "% (" << current << "/" << total << ")" << std::flush;
}

void printStatusLine(const std::string& message) {
    std::printf("%s\n", message.c_str());
    std::fflush(stdout);
}

size_t getAvailableMemoryBytes() {
#ifdef _WIN32
    MEMORYSTATUSEX status{};
    status.dwLength = sizeof(status);
    if (GlobalMemoryStatusEx(&status)) {
        return static_cast<size_t>(status.ullAvailPhys);
    }
#endif
    return 0;
}

struct FrameData {
    cv::Mat image;
    fs::path path;
    size_t bytes = 0;
};

struct EventRecord {
    int eventId = 0;
    std::string label;
    int startFrame = -1;
    int decisionFrame = -1;
    std::string decisionDir;
    int firedFrame = -1;
    int framesTracked = 0;
    double startX = 0.0;
    double startY = 0.0;
    double endX = 0.0;
    double endY = 0.0;
    double minY = 0.0;
    double maxY = 0.0;
    double cumulativeDy = 0.0;
    double pathLength = 0.0;
    int frameHeight = 0;
};

struct EventTracker {
    int resetFrames = 2;
    bool eventActive = false;
    int missCount = 0;
    int currentEventId = 0;
    cv::Point2f startCentroid = {0.0f, 0.0f};
    cv::Point2f lastCentroid = {0.0f, 0.0f};
    bool hasCentroid = false;
    double cumulativeDy = 0.0;
    double lastY = 0.0;
    double minY = 0.0;
    double maxY = 0.0;
    double pathLength = 0.0;
    int frameHeight = 0;
    std::string currentLabel;
    int startFrame = -1;
    int firedFrame = -1;
    int framesTracked = 0;
    std::string lastEventDir = "Unknown";
    std::string lastEventLabel;
    int lastDecisionFrame = -1;
    int lastDecisionEventId = 0;
    int lastFrameNumber = -1;
    std::vector<EventRecord> events;
};

std::string decideDirection(const EventTracker& tracker) {
    std::string dir = "Unknown";
    if (!tracker.hasCentroid) return dir;
    double dy = tracker.cumulativeDy;
    double threshold = 2.0;
    if (tracker.frameHeight > 0) {
        threshold = std::max(threshold, tracker.frameHeight * 0.02);
    }
    bool movedUp = dy < -threshold;
    bool movedDown = dy > threshold;
    bool hasFrame = (tracker.frameHeight > 0);
    double mid = hasFrame ? tracker.frameHeight * 0.5 : 0.0;

    if (movedUp && (!hasFrame || tracker.lastY < mid)) {
        dir = "Waste";
    } else if (movedDown && (!hasFrame || tracker.lastY >= mid)) {
        dir = "Hit";
    } else if (hasFrame) {
        dir = (tracker.lastY < mid) ? "Waste" : "Hit";
    } else if (dy < 0.0) {
        dir = "Waste";
    } else {
        dir = "Hit";
    }
    return dir;
}

void endEvent(EventTracker& tracker, int decisionFrame) {
    if (!tracker.eventActive) return;
    std::string dir = decideDirection(tracker);

    EventRecord rec;
    rec.eventId = tracker.currentEventId;
    rec.label = normalizeLabel(tracker.currentLabel);
    rec.startFrame = tracker.startFrame;
    rec.decisionFrame = decisionFrame;
    rec.decisionDir = dir;
    rec.firedFrame = tracker.firedFrame;
    rec.framesTracked = tracker.framesTracked;
    rec.startX = tracker.startCentroid.x;
    rec.startY = tracker.startCentroid.y;
    rec.endX = tracker.lastCentroid.x;
    rec.endY = tracker.lastCentroid.y;
    rec.minY = tracker.minY;
    rec.maxY = tracker.maxY;
    rec.cumulativeDy = tracker.cumulativeDy;
    rec.pathLength = tracker.pathLength;
    rec.frameHeight = tracker.frameHeight;
    tracker.events.push_back(rec);

    tracker.lastEventDir = dir;
    tracker.lastEventLabel = rec.label;
    tracker.lastDecisionFrame = decisionFrame;
    tracker.lastDecisionEventId = tracker.currentEventId;
    tracker.eventActive = false;
    tracker.hasCentroid = false;
    tracker.missCount = 0;
    tracker.currentLabel.clear();
    tracker.cumulativeDy = 0.0;
    tracker.pathLength = 0.0;
    tracker.framesTracked = 0;
    tracker.startFrame = -1;
    tracker.firedFrame = -1;
}

void startEvent(EventTracker& tracker, const PipelineEvent& evt) {
    tracker.eventActive = true;
    tracker.missCount = 0;
    tracker.currentEventId++;
    tracker.startCentroid = evt.centroid;
    tracker.lastCentroid = evt.centroid;
    tracker.hasCentroid = true;
    tracker.cumulativeDy = 0.0;
    tracker.lastY = evt.centroid.y;
    tracker.minY = evt.centroid.y;
    tracker.maxY = evt.centroid.y;
    tracker.pathLength = 0.0;
    tracker.framesTracked = 1;
    if (evt.frameHeight > 0) tracker.frameHeight = evt.frameHeight;
    tracker.currentLabel = normalizeLabel(evt.label);
    tracker.startFrame = static_cast<int>(evt.frameNumber);
    tracker.firedFrame = evt.fired ? static_cast<int>(evt.frameNumber) : -1;
}

void updateEventTracker(EventTracker& tracker, const PipelineEvent& evt, bool processed) {
    if (!processed) return;
    tracker.lastFrameNumber = static_cast<int>(evt.frameNumber);
    if (evt.fired) {
        if (tracker.eventActive) {
            endEvent(tracker, static_cast<int>(evt.frameNumber));
        }
        startEvent(tracker, evt);
        return;
    }

    if (evt.detected) {
        if (!tracker.eventActive) {
            startEvent(tracker, evt);
        } else {
            double dx = static_cast<double>(evt.centroid.x - tracker.lastCentroid.x);
            double dy = static_cast<double>(evt.centroid.y - tracker.lastCentroid.y);
            tracker.pathLength += std::sqrt(dx * dx + dy * dy);
            tracker.cumulativeDy += dy;
            tracker.lastCentroid = evt.centroid;
            tracker.hasCentroid = true;
            tracker.lastY = evt.centroid.y;
            tracker.minY = std::min(tracker.minY, static_cast<double>(evt.centroid.y));
            tracker.maxY = std::max(tracker.maxY, static_cast<double>(evt.centroid.y));
            tracker.framesTracked++;
            if (evt.frameHeight > 0) tracker.frameHeight = evt.frameHeight;
            tracker.missCount = 0;
        }
    } else if (tracker.eventActive) {
        tracker.missCount++;
        if (tracker.missCount >= tracker.resetFrames) {
            endEvent(tracker, static_cast<int>(evt.frameNumber));
        }
    }
}

void finalizeEventTracker(EventTracker& tracker) {
    if (!tracker.eventActive) return;
    int decisionFrame = tracker.lastFrameNumber >= 0 ? tracker.lastFrameNumber : tracker.startFrame;
    endEvent(tracker, decisionFrame);
}

void writeEventTrajectoryCsv(const fs::path& path, const std::vector<EventRecord>& events) {
    std::ofstream out(path.string(), std::ios::out | std::ios::trunc);
    if (!out.is_open()) return;
    out << "event_id,label,detected_frame,decision_frame,decision_dir,fired_frame,frames_tracked,"
           "start_x,start_y,end_x,end_y,min_y,max_y,cumulative_dy,path_length,frame_height\n";
    for (const auto& rec : events) {
        out << rec.eventId << ","
            << csvQuote(rec.label) << ","
            << rec.startFrame << ","
            << rec.decisionFrame << ","
            << csvQuote(rec.decisionDir) << ","
            << rec.firedFrame << ","
            << rec.framesTracked << ","
            << std::fixed << std::setprecision(3) << rec.startX << ","
            << std::fixed << std::setprecision(3) << rec.startY << ","
            << std::fixed << std::setprecision(3) << rec.endX << ","
            << std::fixed << std::setprecision(3) << rec.endY << ","
            << std::fixed << std::setprecision(3) << rec.minY << ","
            << std::fixed << std::setprecision(3) << rec.maxY << ","
            << std::fixed << std::setprecision(3) << rec.cumulativeDy << ","
            << std::fixed << std::setprecision(3) << rec.pathLength << ","
            << rec.frameHeight
            << "\n";
    }
    out.flush();
}

void writeSummaryCsv(const fs::path& path,
                     const std::vector<EventRecord>& events,
                     const std::string& targetLabel,
                     const fs::path& sequenceFolder,
                     double fps,
                     size_t totalFrames,
                     const fs::path& outputDir) {
    std::ofstream out(path.string(), std::ios::out | std::ios::trunc);
    if (!out.is_open()) return;

    std::string targetLower = toLowerAscii(targetLabel.empty() ? "single" : targetLabel);
    std::map<std::string, int> classCounts;
    int hitCount = 0;
    int wasteCount = 0;
    int truePositive = 0;
    int trueNegative = 0;
    int falsePositive = 0;
    int falseNegative = 0;

    for (const auto& rec : events) {
        std::string label = normalizeLabel(rec.label);
        classCounts[label]++;
        std::string labelLower = toLowerAscii(label);
        bool isTarget = (labelLower == targetLower);
        bool isHit = (rec.decisionDir == "Hit");
        bool isWaste = (rec.decisionDir == "Waste");
        if (isHit) hitCount++;
        if (isWaste) wasteCount++;
        if (isTarget) {
            if (isHit) {
                truePositive++;
            } else if (isWaste) {
                falseNegative++;
            }
        } else {
            if (isWaste) {
                trueNegative++;
            } else if (isHit) {
                falsePositive++;
            }
        }
    }

    int totalDecisions = static_cast<int>(events.size());
    double efficiency = totalDecisions > 0 ? static_cast<double>(truePositive + trueNegative) / totalDecisions : 0.0;
    double precision = (truePositive + falsePositive) > 0
        ? static_cast<double>(truePositive) / (truePositive + falsePositive)
        : 0.0;
    double recall = (truePositive + falseNegative) > 0
        ? static_cast<double>(truePositive) / (truePositive + falseNegative)
        : 0.0;

    out << "metric,value\n";
    out << "sequence_folder," << csvQuote(sequenceFolder.string()) << "\n";
    out << "output_dir," << csvQuote(outputDir.string()) << "\n";
    out << "fps," << std::fixed << std::setprecision(2) << fps << "\n";
    out << "frames_total," << totalFrames << "\n";
    out << "events_detected," << events.size() << "\n";
    out << "hits," << hitCount << "\n";
    out << "wastes," << wasteCount << "\n";
    out << "true_positive_single_hit," << truePositive << "\n";
    out << "false_negative_single_waste," << falseNegative << "\n";
    out << "true_negative_non_single_waste," << trueNegative << "\n";
    out << "false_positive_non_single_hit," << falsePositive << "\n";
    out << "sorting_efficiency," << std::fixed << std::setprecision(4) << efficiency << "\n";
    out << "precision," << std::fixed << std::setprecision(4) << precision << "\n";
    out << "recall," << std::fixed << std::setprecision(4) << recall << "\n";

    out << "\nclass,label,count\n";
    for (const auto& kv : classCounts) {
        out << "class," << csvQuote(kv.first) << "," << kv.second << "\n";
    }

    out << "\nsingle_event_id,detected_frame,decision_frame,decision_dir,fired_frame,frames_tracked,start_y,end_y,cumulative_dy,path_length\n";
    for (const auto& rec : events) {
        std::string labelLower = toLowerAscii(rec.label);
        if (labelLower != targetLower) continue;
        out << rec.eventId << ","
            << rec.startFrame << ","
            << rec.decisionFrame << ","
            << csvQuote(rec.decisionDir) << ","
            << rec.firedFrame << ","
            << rec.framesTracked << ","
            << std::fixed << std::setprecision(3) << rec.startY << ","
            << std::fixed << std::setprecision(3) << rec.endY << ","
            << std::fixed << std::setprecision(3) << rec.cumulativeDy << ","
            << std::fixed << std::setprecision(3) << rec.pathLength
            << "\n";
    }
    out.flush();
}

std::vector<FrameData> loadBatch(const std::vector<fs::path>& files,
                                 size_t& index,
                                 size_t budgetBytes,
                                 size_t& bytesLoaded) {
    std::vector<FrameData> batch;
    bytesLoaded = 0;
    while (index < files.size()) {
        cv::Mat img;
        if (!loadImageGrayscale8(files[index], img)) {
            ++index;
            continue;
        }
        size_t imgBytes = img.total() * img.elemSize();
        if (!batch.empty() && bytesLoaded + imgBytes > budgetBytes) {
            break;
        }
        batch.push_back({img, files[index], imgBytes});
        bytesLoaded += imgBytes;
        ++index;
    }
    return batch;
}

void printUsage() {
    std::cout <<
        "Usage:\n"
        "  sequence_headless --input <folder> --fps <fps> [options]\n"
        "\n"
        "Options:\n"
        "  --output <dir>          Output directory (default: pipeline_output/sequence_<timestamp>)\n"
        "  --onnx <model.onnx>     Path to ONNX model\n"
        "  --metadata <meta.json>  Path to metadata JSON\n"
        "  --target-label <name>   Target label (default: Single)\n"
        "  --help                  Show this help\n";
}

int main(int argc, char** argv) {
    Args args = parseArgs(argc, argv);
    if (hasFlag(args, "--help") || hasFlag(args, "-h")) {
        printUsage();
        return 0;
    }

    std::string inputDir = getValue(args, "--input");
    std::string fpsText = getValue(args, "--fps");
    if (inputDir.empty() && args.positional.size() > 0) {
        inputDir = args.positional[0];
    }
    if (fpsText.empty() && args.positional.size() > 1) {
        fpsText = args.positional[1];
    }

    if (inputDir.empty() || fpsText.empty()) {
        printUsage();
        return 1;
    }

    double fps = 0.0;
    if (!parseDouble(fpsText, fps) || fps <= 0.0) {
        std::cerr << "Invalid fps value: " << fpsText << "\n";
        return 1;
    }

    fs::path inputPath = fs::path(inputDir);
    auto files = collectImageFiles(inputPath);
    if (files.empty()) {
        std::cerr << "No images found in " << inputDir << "\n";
        return 1;
    }

    fs::path exeDir = getExeDir(argv[0]);
    const std::string defaultOnnxName = "squeezenet_final_new_condition.onnx";
    const std::string defaultMetaName = "metadata.json";

    std::string onnxPath = getValue(args, "--onnx");
    std::string metaPath = getValue(args, "--metadata");

    if (onnxPath.empty()) {
        std::vector<fs::path> candidates = {
            fs::current_path() / "models" / defaultOnnxName,
            fs::current_path() / "cpp_pipeline_github" / "models" / defaultOnnxName,
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
        candidates.push_back(fs::current_path() / "cpp_pipeline_github" / "models" / defaultMetaName);
        candidates.push_back(exeDir / "models" / defaultMetaName);
        candidates.push_back(exeDir / ".." / ".." / "models" / defaultMetaName);
        metaPath = resolveExistingPath(candidates);
    }

    if (onnxPath.empty() || metaPath.empty()) {
        if (onnxPath.empty()) {
            std::cerr << "Missing ONNX model. Use --onnx.\n";
        }
        if (metaPath.empty()) {
            std::cerr << "Missing metadata. Use --metadata.\n";
        }
        return 1;
    }

    std::string outputArg = getValue(args, "--output");
    fs::path outputBase;
    if (!outputArg.empty()) {
        outputBase = fs::path(outputArg);
    } else {
        outputBase = fs::current_path() / "pipeline_output";
    }
    auto now = std::chrono::system_clock::now();
    std::string stamp = formatTimestamp(now);
    fs::path outputDir = outputBase / ("sequence_" + stamp);
    std::error_code ec;
    fs::create_directories(outputDir, ec);
    if (ec) {
        std::cerr << "Failed to create output directory: " << outputDir.string() << "\n";
        return 1;
    }

    printStatusLine("Sequence headless starting.");
    printStatusLine("Input: " + inputPath.string());
    printStatusLine("Output: " + outputDir.string());
    printStatusLine("Frames: " + std::to_string(files.size()));

    PipelineConfig cfg;
    cfg.onnxPath = onnxPath;
    cfg.metadataPath = metaPath;
    cfg.targetLabel = getValue(args, "--target-label", "Single");
    cfg.outputDir = outputDir.string();
    cfg.saveCrop = true;
    cfg.saveOverlay = false;
    cfg.cropSize = 64;
    cfg.frameSkip = 0;
    cfg.detect.bgFrames = 100;
    cfg.detect.bgUpdateFrames = 50;
    cfg.detect.resetFrames = 2;
    cfg.detect.minArea = -1.0;
    cfg.detect.minAreaFrac = 0.0;
    cfg.detect.maxAreaFrac = 0.10;
    cfg.detect.minBbox = 32;
    cfg.detect.margin = 5;
    cfg.detect.diffThresh = 15;
    cfg.detect.blurRadius = 1;
    cfg.detect.morphRadius = 1;
    cfg.detect.scale = 0.5;
    cfg.detect.gapFireShift = 0;
    cfg.daq.channel.clear();

    PipelineRunner pipeline;
    std::string err;
    if (!pipeline.init(cfg, err)) {
        std::cerr << "Pipeline init failed: " << err << "\n";
        return 1;
    }
    if (!err.empty()) {
        std::cout << "Pipeline warning: " << err << "\n";
    }
    pipeline.reset();
    printStatusLine("Pipeline ready.");

    std::string timestamp = stamp;
    fs::path logPath = outputDir / ("sequence_test_log_" + timestamp + ".csv");
    std::ofstream logFile(logPath.string(), std::ios::out | std::ios::trunc);
    if (!logFile.is_open()) {
        std::cerr << "Failed to open log file: " << logPath.string() << "\n";
        return 1;
    }

    logFile << "# sequence_folder=" << inputPath.string() << "\n";
    logFile << "# fps=" << std::fixed << std::setprecision(2) << fps << "\n";
    logFile << "# frames=" << files.size() << "\n";
    logFile << "# display_every=1\n";
    logFile << "# output_dir=" << outputDir.string() << "\n";
    logFile << "# onnx=" << onnxPath << "\n";
    logFile << "# metadata=" << metaPath << "\n";
    logFile << "# target_label=" << cfg.targetLabel << "\n";
    logFile << "# pipeline_enabled_before=1\n";
    logFile << "# pipeline_forced=0\n";
    logFile << "# frame_skip=" << cfg.frameSkip << "\n";
    logFile << "# detect_bg_frames=" << cfg.detect.bgFrames << "\n";
    logFile << "# detect_bg_update=" << cfg.detect.bgUpdateFrames << "\n";
    logFile << "# detect_reset_frames=" << cfg.detect.resetFrames << "\n";
    logFile << "# detect_min_area=" << std::fixed << std::setprecision(3) << cfg.detect.minArea << "\n";
    logFile << "# detect_min_area_frac=" << std::fixed << std::setprecision(6) << cfg.detect.minAreaFrac << "\n";
    logFile << "# detect_max_area_frac=" << std::fixed << std::setprecision(6) << cfg.detect.maxAreaFrac << "\n";
    logFile << "# detect_min_bbox=" << cfg.detect.minBbox << "\n";
    logFile << "# detect_margin=" << cfg.detect.margin << "\n";
    logFile << "# detect_diff_thresh=" << cfg.detect.diffThresh << "\n";
    logFile << "# detect_blur_radius=" << cfg.detect.blurRadius << "\n";
    logFile << "# detect_morph_radius=" << cfg.detect.morphRadius << "\n";
    logFile << "# detect_scale=" << std::fixed << std::setprecision(3) << cfg.detect.scale << "\n";
    logFile << "# detect_gap_fire_shift=" << cfg.detect.gapFireShift << "\n";
    logFile << "# daq_channel=\n";
    logFile << "# daq_range_min=-10\n";
    logFile << "# daq_range_max=10\n";
    logFile << "# daq_amplitude_v=0\n";
    logFile << "# daq_frequency_hz=0\n";
    logFile << "# daq_duration_ms=0\n";
    logFile << "# daq_delay_ms=0\n";
    logFile << "index,filename,scheduled_ms,actual_ms,jitter_ms,wall_time,proc_ms,processed,pipeline_enabled,pipeline_ready,bg_remaining,skip_reason,"
            << "detected,fired,area,bbox_x,bbox_y,bbox_w,bbox_h,crop_x,crop_y,crop_w,crop_h,crop_path,label,score,triggered,trigger_ok,frame_number,"
            << "event_dir,decision_frame,decision_event_id\n";
    logFile.flush();

    EventTracker tracker;
    tracker.resetFrames = cfg.detect.resetFrames;

    using clock = std::chrono::steady_clock;
    auto start = clock::now();
    std::chrono::duration<double> period(1.0 / fps);
    size_t globalIndex = 0;

    const bool useBatching = files.size() > 3000;
    size_t index = 0;
    size_t batchIndex = 0;
    while (index < files.size()) {
        size_t available = getAvailableMemoryBytes();
        size_t budget = available > 0 ? static_cast<size_t>(available * 0.8) : (512ull << 20);
        if (!useBatching) {
            budget = static_cast<size_t>(-1);
        }

        size_t bytesLoaded = 0;
        auto batch = loadBatch(files, index, budget, bytesLoaded);
        if (batch.empty()) {
            break;
        }
        batchIndex++;
        std::printf("Loaded batch %zu: %zu frames (%.2f MB)\n",
                    batchIndex,
                    batch.size(),
                    static_cast<double>(bytesLoaded) / (1024.0 * 1024.0));
        std::fflush(stdout);

        for (size_t i = 0; i < batch.size(); ++i, ++globalIndex) {
            auto target = start + period * static_cast<double>(globalIndex);
            while (true) {
                auto now = clock::now();
                if (now >= target) break;
                auto remaining = target - now;
                if (remaining > std::chrono::milliseconds(2)) {
                    std::this_thread::sleep_for(remaining - std::chrono::milliseconds(1));
                } else {
                    std::this_thread::yield();
                }
            }

            double scheduledMs = std::chrono::duration<double, std::milli>(period * static_cast<double>(globalIndex)).count();
            double actualMs = std::chrono::duration<double, std::milli>(clock::now() - start).count();
            double jitterMs = actualMs - scheduledMs;
            std::string wallTime = formatWallTime();

            PipelineEvent evt;
            int bgRemaining = 0;
            bool pipelineReady = pipeline.isReady();
            auto t0 = clock::now();
            bool processed = pipeline.processFrame(batch[i].image, evt);
            auto t1 = clock::now();
            double procMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
            if (pipelineReady) {
                bgRemaining = pipeline.backgroundFramesRemaining();
            }
            std::string skipReason;
            if (!pipelineReady) {
                skipReason = "pipeline_not_ready";
            } else if (!processed) {
                skipReason = "frame_skipped";
            }

            updateEventTracker(tracker, evt, processed);

            logFile << globalIndex << ","
                    << csvQuote(batch[i].path.filename().string()) << ","
                    << std::fixed << std::setprecision(3) << scheduledMs << ","
                    << std::fixed << std::setprecision(3) << actualMs << ","
                    << std::fixed << std::setprecision(3) << jitterMs << ","
                    << csvQuote(wallTime) << ","
                    << std::fixed << std::setprecision(3) << procMs << ","
                    << (processed ? "1" : "0") << ","
                    << "1,"
                    << (pipelineReady ? "1" : "0") << ","
                    << bgRemaining << ","
                    << csvQuote(skipReason) << ","
                    << (evt.detected ? "1" : "0") << ","
                    << (evt.fired ? "1" : "0") << ","
                    << std::fixed << std::setprecision(1) << evt.area << ","
                    << evt.bbox.x << "," << evt.bbox.y << "," << evt.bbox.width << "," << evt.bbox.height << ","
                    << evt.cropRect.x << "," << evt.cropRect.y << "," << evt.cropRect.width << "," << evt.cropRect.height << ","
                    << csvQuote(evt.cropPath) << ","
                    << csvQuote(evt.label) << ","
                    << std::fixed << std::setprecision(4) << evt.score << ","
                    << (evt.triggered ? "1" : "0") << ","
                    << (evt.triggerOk ? "1" : "0") << ","
                    << evt.frameNumber << ","
                    << csvQuote(tracker.lastEventDir) << ","
                    << tracker.lastDecisionFrame << ","
                    << tracker.lastDecisionEventId
                    << "\n";
            if (globalIndex % 50 == 0) {
                logFile.flush();
            }
            printProgress(globalIndex + 1, files.size());
            if ((globalIndex + 1) % 100 == 0 || (globalIndex + 1) == files.size()) {
                std::printf("\nProcessed %zu/%zu frames\n", globalIndex + 1, files.size());
                std::fflush(stdout);
            }
        }
        batch.clear();
    }

    std::cout << "\n";
    finalizeEventTracker(tracker);
    fs::path eventPath = outputDir / ("sequence_event_trajectory_" + timestamp + ".csv");
    fs::path summaryPath = outputDir / ("sequence_summary_" + timestamp + ".csv");
    writeEventTrajectoryCsv(eventPath, tracker.events);
    writeSummaryCsv(summaryPath, tracker.events, cfg.targetLabel, inputPath, fps, files.size(), outputDir);
    logFile.flush();
    logFile.close();
    std::cout << "Done. Log: " << logPath.string()
              << " Trajectory: " << eventPath.string()
              << " Summary: " << summaryPath.string() << "\n";
    return 0;
}
