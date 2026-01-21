#include "pipeline_runner.h"

#include <algorithm>
#include <cctype>
#include <filesystem>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace fs = std::filesystem;
namespace {
constexpr size_t kMaxTriggerQueue = 4;

std::string sanitizeLabel(const std::string& label) {
    std::string out;
    out.reserve(label.size());
    for (unsigned char c : label) {
        if (std::isalnum(c) || c == '-' || c == '_') {
            out.push_back(static_cast<char>(c));
        } else if (c == ' ') {
            out.push_back('_');
        } else {
            out.push_back('_');
        }
    }
    if (out.empty()) {
        out = "unclassified";
    }
    return out;
}
}

PipelineRunner::~PipelineRunner() {
    stopTriggerWorker();
}

void PipelineRunner::startTriggerWorker() {
    stopTriggerWorker();
    if (!triggerReady_) return;
    triggerStop_ = false;
    triggerThread_ = std::thread([this](){
        while (true) {
            {
                std::unique_lock<std::mutex> lock(triggerMutex_);
                triggerCv_.wait(lock, [&](){
                    return triggerStop_ || !triggerQueue_.empty();
                });
                if (triggerStop_) break;
                triggerQueue_.pop_front();
            }
            std::string trigErr;
            trigger_.fire(trigErr);
        }
    });
}

void PipelineRunner::stopTriggerWorker() {
    {
        std::lock_guard<std::mutex> lock(triggerMutex_);
        triggerStop_ = true;
        triggerQueue_.clear();
    }
    triggerCv_.notify_all();
    if (triggerThread_.joinable()) {
        triggerThread_.join();
    }
    triggerStop_ = false;
}

bool PipelineRunner::enqueueTrigger(std::string* err) {
    if (!triggerReady_) {
        if (err) *err = "DAQ trigger not ready";
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(triggerMutex_);
        if (triggerQueue_.size() >= kMaxTriggerQueue) {
            if (err) *err = "DAQ trigger busy";
            return false;
        }
        triggerQueue_.push_back(1);
    }
    triggerCv_.notify_one();
    return true;
}

std::string PipelineRunner::toLowerAscii(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (unsigned char c : s) {
        out.push_back(static_cast<char>(std::tolower(c)));
    }
    return out;
}

cv::Rect PipelineRunner::makeSquareRect(const cv::Rect& bbox, const cv::Size& size) {
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

bool PipelineRunner::init(const PipelineConfig& cfg, std::string& err) {
    stopTriggerWorker();
    cfg_ = cfg;
    ready_ = false;
    triggerReady_ = false;
    frameCounter_ = 0;
    targetLabelLower_ = toLowerAscii(cfg_.targetLabel);

    if (!LoadMetadata(cfg_.metadataPath, meta_, err)) {
        return false;
    }

    if (!classifier_.init(cfg_.onnxPath, meta_, err)) {
        return false;
    }

    detector_ = std::make_unique<FastEventDetector>(cfg_.detect);

    if (!cfg_.daq.channel.empty()) {
        std::string trigErr;
        if (!trigger_.init(cfg_.daq, trigErr)) {
            err = "DAQ init disabled: " + trigErr;
        } else {
            triggerReady_ = true;
            startTriggerWorker();
        }
    }

    ready_ = true;
    return true;
}

void PipelineRunner::reset() {
    if (detector_) {
        detector_->reset();
    }
    frameCounter_ = 0;
}

bool PipelineRunner::isReady() const {
    return ready_;
}

bool PipelineRunner::isTriggerReady() const {
    return triggerReady_;
}

int PipelineRunner::backgroundFramesRemaining() const {
    if (!detector_) return 0;
    return detector_->backgroundFramesRemaining();
}

bool PipelineRunner::fireTrigger(std::string& err) {
    return enqueueTrigger(&err);
}

bool PipelineRunner::processFrame(const cv::Mat& gray8In, PipelineEvent& out) {
    out = PipelineEvent{};
    if (!ready_ || !detector_) return false;
    if (gray8In.empty()) return false;

    frameCounter_++;
    out.frameNumber = frameCounter_;
    if (cfg_.frameSkip > 0 && (frameCounter_ % (cfg_.frameSkip + 1)) != 0) {
        return false;
    }

    cv::Mat gray8 = gray8In;
    if (gray8.type() != CV_8UC1) {
        if (gray8.channels() == 3) {
            cv::cvtColor(gray8, gray8, cv::COLOR_BGR2GRAY);
        }
        gray8.convertTo(gray8, CV_8U);
    }
    out.frameWidth = gray8.cols;
    out.frameHeight = gray8.rows;

    FastEventResult det;
    detector_->processFrame(gray8, det);
    out.detected = det.detected;
    out.fired = det.fired;
    out.area = det.area;
    out.bbox = det.bbox;
    out.centroid = det.centroid;

    if (!det.fired) return true;

    cv::Rect bbox = det.bbox & cv::Rect(0, 0, gray8.cols, gray8.rows);
    cv::Rect squareRect = makeSquareRect(bbox, gray8.size());
    if (squareRect.width <= 0 || squareRect.height <= 0) {
        return true;
    }
    out.cropRect = squareRect;

    cv::Mat crop = gray8(squareRect).clone();
    ClassificationResult cls = classifier_.classify(crop);
    out.label = cls.label;
    out.classified = true;
    if (!cls.scores.empty()) {
        auto bestIt = std::max_element(cls.scores.begin(), cls.scores.end());
        out.score = (bestIt != cls.scores.end()) ? *bestIt : 0.0f;
    }

    if (triggerReady_ && !cls.label.empty()) {
        if (toLowerAscii(cls.label) == targetLabelLower_) {
            out.triggered = true;
            out.triggerOk = enqueueTrigger(nullptr);
        }
    }

    if (!cfg_.outputDir.empty()) {
        fs::path base(cfg_.outputDir);
        fs::create_directories(base);
        std::string labelName = sanitizeLabel(out.label.empty() ? "unclassified" : out.label);
        fs::path labelDir = base / labelName;
        fs::create_directories(labelDir);
        std::string name = "event_frame_" + std::to_string(frameCounter_) + "_label_" + labelName;
        if (cfg_.saveCrop) {
            cv::Mat resized;
            cv::resize(crop, resized, cv::Size(cfg_.cropSize, cfg_.cropSize), 0, 0, cv::INTER_AREA);
            fs::path outPath = labelDir / (name + ".png");
            cv::imwrite(outPath.string(), resized);
            out.cropPath = outPath.string();
        }
        if (cfg_.saveOverlay) {
            cv::Mat overlay;
            cv::cvtColor(gray8, overlay, cv::COLOR_GRAY2BGR);
            cv::rectangle(overlay, squareRect, cv::Scalar(0, 255, 0), 2);
            fs::path outPath = labelDir / (name + "_overlay.png");
            cv::imwrite(outPath.string(), overlay);
            out.overlayPath = outPath.string();
        }
    }

    return true;
}
