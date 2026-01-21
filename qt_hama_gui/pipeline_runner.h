#pragma once

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core.hpp>

#include "../daq_trigger.h"
#include "../fast_event_detector.h"
#include "../metadata_loader.h"
#include "../onnx_classifier.h"

struct PipelineConfig {
    std::string onnxPath;
    std::string metadataPath;
    std::string targetLabel = "Single";
    std::string outputDir;
    bool saveCrop = false;
    bool saveOverlay = false;
    int cropSize = 64;
    int frameSkip = 0;
    FastEventConfig detect;
    DaqConfig daq;
};

struct PipelineEvent {
    bool detected = false;
    bool fired = false;
    bool classified = false;
    bool triggered = false;
    bool triggerOk = false;
    double area = 0.0;
    int frameWidth = 0;
    int frameHeight = 0;
    cv::Rect bbox;
    cv::Rect cropRect;
    cv::Point2f centroid = {0.0f, 0.0f};
    std::string label;
    float score = 0.0f;
    std::string cropPath;
    std::string overlayPath;
    int64_t frameNumber = 0;
};

class PipelineRunner {
public:
    PipelineRunner() = default;
    ~PipelineRunner();
    bool init(const PipelineConfig& cfg, std::string& err);
    void reset();
    bool isReady() const;
    bool isTriggerReady() const;
    int backgroundFramesRemaining() const;
    bool processFrame(const cv::Mat& gray8, PipelineEvent& out);
    bool fireTrigger(std::string& err);

private:
    static std::string toLowerAscii(const std::string& s);
    static cv::Rect makeSquareRect(const cv::Rect& bbox, const cv::Size& size);
    bool enqueueTrigger(std::string* err);
    void startTriggerWorker();
    void stopTriggerWorker();

    PipelineConfig cfg_;
    Metadata meta_;
    OnnxClassifier classifier_;
    std::unique_ptr<FastEventDetector> detector_;
    DaqTrigger trigger_;
    bool ready_ = false;
    bool triggerReady_ = false;
    int64_t frameCounter_ = 0;
    std::string targetLabelLower_;
    std::mutex triggerMutex_;
    std::condition_variable triggerCv_;
    std::deque<int> triggerQueue_;
    std::thread triggerThread_;
    bool triggerStop_ = false;
};
