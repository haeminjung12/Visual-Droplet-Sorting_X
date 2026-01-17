#include "daq_trigger.h"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>

struct Args {
    std::unordered_map<std::string, std::string> values;
    std::unordered_set<std::string> flags;
};

Args parseArgs(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        std::string key = argv[i];
        if (key == "-h") {
            args.flags.insert("--help");
            continue;
        }
        if (key.rfind("-", 0) == 0) {
            if (i + 1 < argc) {
                std::string next = argv[i + 1];
                if (next.rfind("-", 0) != 0) {
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

bool readDouble(const Args& args, const std::string& key, double& value, std::string& err) {
    auto it = args.values.find(key);
    if (it == args.values.end()) {
        return true;
    }
    try {
        size_t idx = 0;
        value = std::stod(it->second, &idx);
        if (idx != it->second.size()) {
            throw std::invalid_argument("trailing");
        }
    } catch (const std::exception&) {
        err = "Invalid value for " + key + ": " + it->second;
        return false;
    }
    return true;
}

bool readInt(const Args& args, const std::string& key, int& value, std::string& err) {
    auto it = args.values.find(key);
    if (it == args.values.end()) {
        return true;
    }
    try {
        size_t idx = 0;
        value = std::stoi(it->second, &idx);
        if (idx != it->second.size()) {
            throw std::invalid_argument("trailing");
        }
    } catch (const std::exception&) {
        err = "Invalid value for " + key + ": " + it->second;
        return false;
    }
    return true;
}

void printUsage() {
    std::cout <<
        "Usage:\n"
        "  daq_test_cli [options]\n"
        "\n"
        "Options:\n"
        "  --channel <Dev1/ao0>      Analog output channel (default: Dev1/ao0)\n"
        "  --range-min <V>           Output range min (default: -10)\n"
        "  --range-max <V>           Output range max (default: 10)\n"
        "  --amp <V>                 Sine amplitude (default: 5)\n"
        "  --freq <Hz>               Sine frequency (default: 1000)\n"
        "  --duration-ms <ms>        Sine duration (default: 5)\n"
        "  --delay-ms <ms>           Delay before output (default: 0)\n"
        "  --count <n>               Number of pulses (default: 1)\n"
        "  --interval-ms <ms>        Delay between pulses (default: 100)\n"
        "  --help, -h                Show this help\n";
}

int main(int argc, char** argv) {
    Args args = parseArgs(argc, argv);
    if (hasFlag(args, "--help")) {
        printUsage();
        return 0;
    }

    DaqConfig cfg;
    cfg.channel = getString(args, "--channel", cfg.channel);

    std::string err;
    if (!readDouble(args, "--range-min", cfg.rangeMin, err) ||
        !readDouble(args, "--range-max", cfg.rangeMax, err) ||
        !readDouble(args, "--amp", cfg.amplitude, err) ||
        !readDouble(args, "--freq", cfg.frequencyHz, err) ||
        !readDouble(args, "--duration-ms", cfg.durationMs, err) ||
        !readDouble(args, "--delay-ms", cfg.delayMs, err)) {
        std::cerr << err << "\n";
        return 1;
    }

    int count = 1;
    double intervalMs = 100.0;
    if (!readInt(args, "--count", count, err) ||
        !readDouble(args, "--interval-ms", intervalMs, err)) {
        std::cerr << err << "\n";
        return 1;
    }

    if (count < 1) {
        std::cerr << "--count must be >= 1\n";
        return 1;
    }

    DaqTrigger trigger;
    if (!trigger.init(cfg, err)) {
        std::cerr << "DAQ init failed: " << err << "\n";
        return 1;
    }

    for (int i = 0; i < count; ++i) {
        std::cout << "Firing " << (i + 1) << "/" << count
                  << " (amp=" << cfg.amplitude
                  << "V, freq=" << cfg.frequencyHz
                  << "Hz, duration=" << cfg.durationMs << "ms)\n";
        if (!trigger.fire(err)) {
            std::cerr << "DAQ fire failed: " << err << "\n";
            trigger.shutdown();
            return 1;
        }
        if (i + 1 < count && intervalMs > 0.0) {
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(intervalMs));
        }
    }

    trigger.shutdown();
    return 0;
}
