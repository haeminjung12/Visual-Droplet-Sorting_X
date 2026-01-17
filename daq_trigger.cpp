#include "daq_trigger.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>

#ifdef HAVE_NIDAQMX
#include <NIDAQmx.h>

static std::string formatDaqError(const char* label, int32 err) {
    char buf[2048] = {};
    DAQmxGetExtendedErrorInfo(buf, static_cast<uInt32>(sizeof(buf)));
    std::string msg = label;
    msg += " (error ";
    msg += std::to_string(static_cast<int>(err));
    msg += ")";
    if (buf[0] != '\0') {
        msg += ": ";
        msg += buf;
    }
    return msg;
}
#endif

namespace {
constexpr double kMinSampleRate = 10000.0;
constexpr double kMaxSampleRate = 900000.0;
constexpr double kSamplesPerCycle = 50.0;
}

DaqTrigger::DaqTrigger()
    : ready_(false)
#ifdef HAVE_NIDAQMX
    , task_(nullptr)
#endif
{}

DaqTrigger::~DaqTrigger() {
    shutdown();
}

bool DaqTrigger::init(const DaqConfig& cfg, std::string& err) {
    cfg_ = cfg;
    ready_ = false;
    shutdown();

    if (cfg_.channel.empty()) {
        err = "DAQ channel is empty";
        return false;
    }
    if (cfg_.rangeMin >= cfg_.rangeMax) {
        err = "DAQ output range is invalid";
        return false;
    }
    double durationSec = cfg_.durationMs / 1000.0;
    if (durationSec <= 0.0) {
        err = "DAQ duration must be > 0";
        return false;
    }
    if (cfg_.frequencyHz <= 0.0) {
        err = "DAQ frequency must be > 0";
        return false;
    }
    double maxAbs = std::min(std::abs(cfg_.rangeMin), std::abs(cfg_.rangeMax));
    if (maxAbs <= 0.0) {
        err = "DAQ output range invalid";
        return false;
    }

#ifdef HAVE_NIDAQMX
    TaskHandle task = nullptr;
    int32 error = DAQmxCreateTask("", &task);
    if (DAQmxFailed(error)) {
        err = formatDaqError("DAQmxCreateTask failed", error);
        return false;
    }
    error = DAQmxCreateAOVoltageChan(task, cfg_.channel.c_str(), "", cfg_.rangeMin, cfg_.rangeMax,
                                     DAQmx_Val_Volts, nullptr);
    if (DAQmxFailed(error)) {
        err = formatDaqError("DAQmxCreateAOVoltageChan failed", error);
        DAQmxClearTask(task);
        return false;
    }

    double amplitude = std::clamp(std::abs(cfg_.amplitude), 0.0, maxAbs);
    double sampleRate = std::max(kMinSampleRate, cfg_.frequencyHz * kSamplesPerCycle);
    sampleRate = std::min(sampleRate, kMaxSampleRate);
    int samples = std::max(2, static_cast<int>(std::lround(durationSec * sampleRate)));

    waveform_.assign(samples, 0.0);
    double omega = 2.0 * 3.14159265358979323846 * cfg_.frequencyHz;
    for (int i = 0; i < samples; ++i) {
        double t = static_cast<double>(i) / sampleRate;
        waveform_[i] = amplitude * std::sin(omega * t);
    }

    error = DAQmxCfgSampClkTiming(task, "", sampleRate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, samples);
    if (DAQmxFailed(error)) {
        err = formatDaqError("DAQmxCfgSampClkTiming failed", error);
        DAQmxClearTask(task);
        return false;
    }
    error = DAQmxCfgOutputBuffer(task, static_cast<uInt32>(samples));
    if (DAQmxFailed(error)) {
        err = formatDaqError("DAQmxCfgOutputBuffer failed", error);
        DAQmxClearTask(task);
        return false;
    }

    task_ = task;
    sampleRate_ = sampleRate;
    samples_ = samples;
    ready_ = true;
    return true;
#else
    (void)cfg_;
    err = "NI-DAQmx not available at build time";
    return false;
#endif
}

void DaqTrigger::shutdown() {
#ifdef HAVE_NIDAQMX
    if (task_) {
        TaskHandle task = static_cast<TaskHandle>(task_);
        DAQmxStopTask(task);
        DAQmxClearTask(task);
        task_ = nullptr;
    }
    waveform_.clear();
    sampleRate_ = 0.0;
    samples_ = 0;
#endif
    ready_ = false;
}

bool DaqTrigger::fire(std::string& err) {
    if (!ready_) {
        err = "DAQ trigger not initialized";
        return false;
    }

#ifdef HAVE_NIDAQMX
    if (cfg_.delayMs > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(cfg_.delayMs));
    }

    if (samples_ <= 0 || waveform_.empty()) {
        err = "DAQ waveform not configured";
        return false;
    }

    TaskHandle task = static_cast<TaskHandle>(task_);
    DAQmxStopTask(task);

    int32 written = 0;
    int32 error = DAQmxWriteAnalogF64(task, samples_, 1, 10.0, DAQmx_Val_GroupByChannel,
                                      waveform_.data(), &written, nullptr);
    if (DAQmxFailed(error)) {
        err = formatDaqError("DAQmxWriteAnalogF64 failed", error);
        return false;
    }

    double timeout = 5.0 + (cfg_.durationMs + cfg_.delayMs) / 1000.0;
    error = DAQmxWaitUntilTaskDone(task, timeout);
    if (DAQmxFailed(error)) {
        err = formatDaqError("DAQmxWaitUntilTaskDone failed", error);
        DAQmxStopTask(task);
        return false;
    }
    DAQmxStopTask(task);
    return true;
#else
    err = "NI-DAQmx not available at build time";
    return false;
#endif
}

bool DaqTrigger::isReady() const {
    return ready_;
}
