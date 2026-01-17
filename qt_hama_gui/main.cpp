#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <QtWidgets>
#include <QtCore>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QMutex>
#include <QScrollArea>
#include <QWheelEvent>
#include <QScrollBar>
#include <QStandardPaths>
#include <windows.h>
#include <dbghelp.h>
#include <algorithm>
#include <functional>
#include <atomic>
#include <exception>
#include <csignal>
#include <cstdio>
#include <thread>
#include <vector>
#include <memory>
#include <utility>
#include <chrono>
#include <cmath>
#include <string>
#include <iostream>
#include <opencv2/core.hpp>
#include "pipeline_runner.h"
#include "log_teebuf.h"
#include "frame_types.h"
#include "dcam_controller.h"
#include "frame_grabber.h"
#include "../cli_runner.h"

#pragma comment(lib, "Dbghelp.lib")

namespace {
QMutex gLogMutex;
QFile gLogFile;
QString gLogPath;
std::atomic<bool> gCrashHandled(false);
void logMessage(const QString& msg);
void logMessageNoPrune(const QString& msg);
void installLogTees();

struct SequenceFrame {
    QImage image;
    QString path;
};

struct StatsTracker {
    QMap<QString, int> classCounts;
    int totalEvents = 0;
    int hitCount = 0;
    int wasteCount = 0;
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
    int frameHeight = 0;
    QString currentLabel;
    QString lastEventDir = "Unknown";
    QString lastEventLabel;
    int lastDecisionFrame = -1;
    int lastDecisionEventId = 0;
};

struct StatsSnapshot {
    int totalEvents = 0;
    int hitCount = 0;
    int wasteCount = 0;
    bool eventActive = false;
    QString classText;
    QString lastText;
    QMap<QString, int> classCounts;
    QString lastEventDir;
    QString lastEventLabel;
    int lastDecisionFrame = -1;
    int lastDecisionEventId = 0;
};

struct LiveLogRecord {
    QString wallTime;
    qint64 frameIndex = 0;
    qint64 delivered = 0;
    qint64 dropped = 0;
    double fps = 0.0;
    double camFps = 0.0;
    double procMs = 0.0;
    bool processed = false;
    bool pipelineEnabled = false;
    bool pipelineReady = false;
    QString skipReason;
    bool detected = false;
    bool fired = false;
    double area = 0.0;
    int bboxX = 0;
    int bboxY = 0;
    int bboxW = 0;
    int bboxH = 0;
    int cropX = 0;
    int cropY = 0;
    int cropW = 0;
    int cropH = 0;
    QString cropPath;
    QString label;
    double score = 0.0;
    bool triggered = false;
    bool triggerOk = false;
    int bgRemaining = 0;
    QString eventDir;
    int decisionFrame = -1;
    int decisionEventId = 0;
    int hitCount = 0;
    int wasteCount = 0;
};

class ZoomImageView : public QScrollArea {
public:
    ZoomImageView(QWidget* parent=nullptr)
        : QScrollArea(parent), label(new QLabel), scale(1.0), hasImage(false), zoomSteps(0), effectiveScale(1.0) {
        label->setBackgroundRole(QPalette::Base);
        label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        label->setScaledContents(true); // paint-time scaling instead of allocating huge pixmaps
        setWidget(label);
        setAlignment(Qt::AlignCenter);
        setWidgetResizable(false);
        setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        setMouseTracking(true);
    }

    void setZoomChanged(const std::function<void(double)>& cb) { onZoomChanged = cb; }

    void setImage(const QImage& img) {
        if (img.isNull()) return;
        if (!hasImage) {
            scale = 1.0;
            effectiveScale = 1.0;
            hasImage = true;
            zoomSteps = 0;
            zoomReadyTimer.restart();
            if (onZoomChanged) onZoomChanged(effectiveScale);
            if (horizontalScrollBar()) horizontalScrollBar()->setValue(0);
            if (verticalScrollBar()) verticalScrollBar()->setValue(0);
        }
        // Make a deep copy so the buffer is stable while frames keep streaming.
        lastImage = img.copy();
        basePixmap = QPixmap::fromImage(lastImage);
        updatePixmap();
    }

    void resetScale() {
        scale = 1.0;
        effectiveScale = 1.0;
        zoomSteps = 0;
        if (onZoomChanged) onZoomChanged(effectiveScale);
        hasImage = !lastImage.isNull();
        updatePixmap();
    }

protected:
    void wheelEvent(QWheelEvent* ev) override {
        try {
            if (lastImage.isNull()) {
                QScrollArea::wheelEvent(ev);
                return;
            }
            if (zoomReadyTimer.isValid() && zoomReadyTimer.elapsed() < 1000) {
                ev->accept();
                return;
            }
            if (!horizontalScrollBar() || !verticalScrollBar()) {
                QScrollArea::wheelEvent(ev);
                return;
            }
            // Normalize to wheel ticks (120 per detent)
            double ticks = ev->angleDelta().y() / 120.0;
            double oldScale = scale;
            int newSteps = std::clamp(zoomSteps + static_cast<int>(std::round(ticks)), -50, 50);
            double desiredScale = std::pow(1.25, newSteps); // ~1.25x per tick
            double maxScale = computeMaxScale();
            double newScale = std::clamp(desiredScale, 0.05, maxScale); // avoid zero/negative and clamp max
            if (qFuzzyCompare(newScale, scale)) {
                ev->accept();
                return;
            }
            // Keep steps consistent with the clamped scale to avoid runaway values.
            zoomSteps = static_cast<int>(std::lround(std::log(newScale) / std::log(1.25)));

            QPointF vpPos = ev->position();
            QPointF contentPos = (vpPos + QPointF(horizontalScrollBar()->value(),
                                                  verticalScrollBar()->value())) / oldScale;

            scale = newScale;
            updatePixmap();

            horizontalScrollBar()->setValue(int(contentPos.x() * scale - vpPos.x()));
            verticalScrollBar()->setValue(int(contentPos.y() * scale - vpPos.y()));
            ev->accept();
        } catch (const std::exception& e) {
            Q_UNUSED(e);
        } catch (...) {
        }
    }

private:
    double computeMaxScale() const {
        if (basePixmap.isNull()) return 1.56;
        int w = basePixmap.width();
        int h = basePixmap.height();
        int maxDim = (std::min(w, h) <= 256) ? 8192 : 4096;
        double dimCap = static_cast<double>(maxDim) / static_cast<double>(std::max(w, h));
        // Allow more zoom for small dimensions but cap to a sane upper bound.
        return std::clamp(std::max(1.56, dimCap * 2.0), 0.1, 8.0);
    }

    void updatePixmap() {
        try {
            if (basePixmap.isNull() || scale <= 0.0) return;
            if (updatingPixmap.test_and_set()) {
                // Skip re-entrant calls that can happen when zooming rapidly during streaming.
                return;
            }
            int baseW = basePixmap.width();
            int baseH = basePixmap.height();
            QSize targetSize = (scale == 1.0)
                ? basePixmap.size()
                : QSize(std::max(1, int(std::lround(baseW * scale))),
                        std::max(1, int(std::lround(baseH * scale))));

            int maxDim = (std::min(baseW, baseH) <= 256) ? 8192 : 4096;
            if (targetSize.width() > maxDim || targetSize.height() > maxDim) {
                double factor = static_cast<double>(maxDim) / static_cast<double>(std::max(targetSize.width(), targetSize.height()));
                targetSize.setWidth(std::max(1, int(std::lround(targetSize.width() * factor))));
                targetSize.setHeight(std::max(1, int(std::lround(targetSize.height() * factor))));
            }

            label->setPixmap(basePixmap);
            label->resize(targetSize);
            label->setAlignment(Qt::AlignCenter);
            effectiveScale = static_cast<double>(targetSize.width()) / static_cast<double>(baseW);
            if (onZoomChanged) onZoomChanged(effectiveScale);
            updatingPixmap.clear();
        } catch (const std::exception& e) {
            Q_UNUSED(e);
            updatingPixmap.clear();
        } catch (...) {
            updatingPixmap.clear();
        }
    }

    QLabel* label;
    QImage lastImage;
    QPixmap basePixmap;
    double scale;
    double effectiveScale;
    bool hasImage;
    int zoomSteps;
    QElapsedTimer zoomReadyTimer;
    std::atomic_flag updatingPixmap = ATOMIC_FLAG_INIT;
    std::function<void(double)> onZoomChanged;
};

static QString formatTimeSeconds(double seconds) {
    if (seconds < 0) seconds = 0;
    int totalMs = static_cast<int>(std::lround(seconds * 1000.0));
    int ms = totalMs % 1000;
    int totalSec = totalMs / 1000;
    int s = totalSec % 60;
    int totalMin = totalSec / 60;
    int m = totalMin % 60;
    int h = totalMin / 60;
    if (h > 0) {
        return QString("%1:%2:%3.%4")
            .arg(h,2,10,QChar('0'))
            .arg(m,2,10,QChar('0'))
            .arg(s,2,10,QChar('0'))
            .arg(ms,3,10,QChar('0'));
    }
    return QString("%1:%2.%3")
        .arg(m,2,10,QChar('0'))
        .arg(s,2,10,QChar('0'))
        .arg(ms,3,10,QChar('0'));
}

static QImage renderPieChart(const QString& title,
                             const QVector<QString>& labels,
                             const QVector<double>& values,
                             const QVector<QColor>& colors,
                             const QSize& size = QSize(520, 420)) {
    QImage img(size, QImage::Format_ARGB32_Premultiplied);
    img.fill(Qt::white);

    QPainter p(&img);
    p.setRenderHint(QPainter::Antialiasing, true);
    QFont titleFont = p.font();
    titleFont.setPointSize(12);
    titleFont.setBold(true);
    p.setFont(titleFont);
    p.setPen(Qt::black);
    p.drawText(QRect(0, 0, size.width(), 30), Qt::AlignCenter, title);

    double total = 0.0;
    for (double v : values) total += v;
    QRect pieRect(20, 40, size.height() - 60, size.height() - 60);
    if (total <= 0.0) {
        p.setFont(QFont(p.font().family(), 10));
        p.drawText(pieRect, Qt::AlignCenter, "No data");
        return img;
    }

    int startAngle = 0;
    for (int i = 0; i < values.size(); ++i) {
        double fraction = values[i] / total;
        int span = static_cast<int>(std::round(fraction * 360.0 * 16.0));
        p.setBrush(colors.value(i, Qt::gray));
        p.setPen(Qt::NoPen);
        p.drawPie(pieRect, startAngle, span);
        startAngle += span;
    }

    QRect legendRect(pieRect.right() + 20, pieRect.top(), size.width() - pieRect.right() - 30, pieRect.height());
    p.setPen(Qt::black);
    QFont legendFont = p.font();
    legendFont.setPointSize(9);
    legendFont.setBold(false);
    p.setFont(legendFont);
    int y = legendRect.top();
    for (int i = 0; i < labels.size(); ++i) {
        double percent = values[i] / total * 100.0;
        QRect colorBox(legendRect.left(), y + 4, 12, 12);
        p.fillRect(colorBox, colors.value(i, Qt::gray));
        p.drawRect(colorBox);
        QString text = QString("%1  %2% (%3)")
            .arg(labels[i])
            .arg(percent, 0, 'f', 1)
            .arg(static_cast<int>(values[i]));
        p.drawText(legendRect.left() + 18, y + 14, text);
        y += 20;
    }

    return img;
}

class StatsFigureWindow : public QDialog {
public:
    explicit StatsFigureWindow(QWidget* parent=nullptr)
        : QDialog(parent) {
        setWindowTitle("Pipeline Figures");
        resize(1100, 600);
        auto layout = new QVBoxLayout;
        auto row = new QHBoxLayout;
        hitWasteLabel = new QLabel;
        classLabel = new QLabel;
        hitWasteLabel->setAlignment(Qt::AlignCenter);
        classLabel->setAlignment(Qt::AlignCenter);
        row->addWidget(hitWasteLabel, 1);
        row->addWidget(classLabel, 1);
        layout->addLayout(row);
        saveBtn = new QPushButton("Save Figures...");
        layout->addWidget(saveBtn, 0, Qt::AlignRight);
        setLayout(layout);
    }

    void setImages(const QImage& hitWaste, const QImage& classImg) {
        hitWasteImg = hitWaste;
        classImg_ = classImg;
        hitWasteLabel->setPixmap(QPixmap::fromImage(hitWasteImg));
        classLabel->setPixmap(QPixmap::fromImage(classImg_));
    }

    QPushButton* saveButton() const { return saveBtn; }

    bool saveImages(const QString& dir, const QString& prefix) const {
        if (dir.isEmpty()) return false;
        QDir out(dir);
        out.mkpath(".");
        QString hitPath = out.filePath(prefix + "_hit_waste.png");
        QString clsPath = out.filePath(prefix + "_class_dist.png");
        bool ok1 = hitWasteImg.isNull() ? false : hitWasteImg.save(hitPath);
        bool ok2 = classImg_.isNull() ? false : classImg_.save(clsPath);
        return ok1 && ok2;
    }

private:
    QLabel* hitWasteLabel = nullptr;
    QLabel* classLabel = nullptr;
    QPushButton* saveBtn = nullptr;
    QImage hitWasteImg;
    QImage classImg_;
};

class ViewerWindow : public QWidget {
public:
    ViewerWindow(QWidget* parent=nullptr)
        : QWidget(parent), fps(0.0) {
        setWindowFlags(Qt::Window);
        setWindowTitle("Capture Viewer");
        resize(1100, 800);
        setMinimumSize(800, 600);

        imageView = new ZoomImageView;
        imageView->setMinimumSize(640, 480);
        imageView->setStyleSheet("background:#000;");
        imageView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        frameLabel = new QLabel("Frame: -- / --");
        timeLabel = new QLabel("Time: -- / --");
        frameLabel->setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard);
        timeLabel->setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard);

        folderEdit = new QLineEdit;
        folderEdit->setPlaceholderText("Select capture folder...");
        auto browseBtn = new QPushButton("...");
        auto loadBtn = new QPushButton("Load");
        recentCombo = new QComboBox;
        recentCombo->setMinimumWidth(200);

        slider = new QSlider(Qt::Horizontal);
        slider->setRange(0, 0);
        slider->setEnabled(false);

        prevBtn = new QPushButton("<");
        nextBtn = new QPushButton(">");
        prevBtn->setEnabled(false);
        nextBtn->setEnabled(false);

        auto folderRow = new QHBoxLayout;
        folderRow->addWidget(new QLabel("Folder"));
        folderRow->addWidget(folderEdit, 1);
        folderRow->addWidget(browseBtn);
        folderRow->addWidget(loadBtn);

        auto recentRow = new QHBoxLayout;
        recentRow->addWidget(new QLabel("Recent"));
        recentRow->addWidget(recentCombo, 1);

        auto navRow = new QHBoxLayout;
        navRow->addWidget(prevBtn);
        navRow->addWidget(nextBtn);
        navRow->addWidget(frameLabel, 1);

        auto infoCol = new QVBoxLayout;
        infoCol->addLayout(folderRow);
        infoCol->addLayout(recentRow);
        infoCol->addWidget(timeLabel);
        infoCol->addLayout(navRow);
        infoCol->addWidget(slider);
        infoCol->addStretch(1);

        auto rightPane = new QWidget;
        rightPane->setLayout(infoCol);
        rightPane->setMinimumWidth(320);

        auto layout = new QHBoxLayout;
        layout->addWidget(imageView, 3);
        layout->addWidget(rightPane, 1);
        setLayout(layout);

        imageView->setZoomChanged(nullptr);

        QObject::connect(browseBtn, &QPushButton::clicked, [this](){
            QString dir = QFileDialog::getExistingDirectory(this, "Select capture folder", folderEdit->text());
            if (!dir.isEmpty()) folderEdit->setText(dir);
        });
        QObject::connect(loadBtn, &QPushButton::clicked, [this](){
            loadFolder(folderEdit->text());
        });
        QObject::connect(recentCombo, &QComboBox::activated, [this](int idx){
            if (idx < 0) return;
            QString dir = recentCombo->itemText(idx);
            if (!dir.isEmpty()) {
                folderEdit->setText(dir);
                loadFolder(dir);
            }
        });
        QObject::connect(slider, &QSlider::valueChanged, [this](int v){
            loadFrame(v);
        });
        QObject::connect(prevBtn, &QPushButton::clicked, [this](){
            if (frameFiles.isEmpty()) return;
            int v = std::max(0, slider->value() - 1);
            slider->setValue(v);
        });
        QObject::connect(nextBtn, &QPushButton::clicked, [this](){
            if (frameFiles.isEmpty()) return;
            int v = std::min(slider->maximum(), slider->value() + 1);
            slider->setValue(v);
        });

        auto leftShortcut = new QShortcut(QKeySequence(Qt::Key_Left), this);
        auto rightShortcut = new QShortcut(QKeySequence(Qt::Key_Right), this);
        auto ctrlLeftShortcut = new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_Left), this);
        auto ctrlRightShortcut = new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_Right), this);
        auto pageUpShortcut = new QShortcut(QKeySequence(Qt::Key_PageUp), this);
        auto pageDownShortcut = new QShortcut(QKeySequence(Qt::Key_PageDown), this);
        QObject::connect(leftShortcut, &QShortcut::activated, [this](){
            stepFrames(-1);
        });
        QObject::connect(rightShortcut, &QShortcut::activated, [this](){
            stepFrames(1);
        });
        QObject::connect(ctrlLeftShortcut, &QShortcut::activated, [this](){
            stepFrames(-5);
        });
        QObject::connect(ctrlRightShortcut, &QShortcut::activated, [this](){
            stepFrames(5);
        });
        QObject::connect(pageUpShortcut, &QShortcut::activated, [this](){
            stepFrames(-10);
        });
        QObject::connect(pageDownShortcut, &QShortcut::activated, [this](){
            stepFrames(10);
        });

        loadRecentFolders();
    }

private:
    void stepFrames(int delta) {
        if (frameFiles.isEmpty()) return;
        int v = std::clamp(slider->value() + delta, 0, slider->maximum());
        slider->setValue(v);
    }

    void loadRecentFolders() {
        QSettings settings;
        QStringList recent = settings.value("viewer/recentFolders").toStringList();
        recentCombo->clear();
        for (const QString& path : recent) {
            recentCombo->addItem(path);
        }
    }

    void updateRecentFolders(const QString& dirPath) {
        QSettings settings;
        QStringList recent = settings.value("viewer/recentFolders").toStringList();
        recent.removeAll(dirPath);
        recent.prepend(dirPath);
        while (recent.size() > 10) recent.removeLast();
        settings.setValue("viewer/recentFolders", recent);
        recentCombo->clear();
        for (const QString& path : recent) {
            recentCombo->addItem(path);
        }
    }

    void loadFolder(const QString& dirPath) {
        QDir dir(dirPath);
        if (!dir.exists()) {
            QMessageBox::warning(this, "Folder not found", "The selected folder does not exist.");
            return;
        }
        QStringList filters;
        filters << "*.tif" << "*.tiff" << "*.TIF" << "*.TIFF";
        frameFiles = dir.entryList(filters, QDir::Files, QDir::Name);
        for (QString& f : frameFiles) {
            f = dir.absoluteFilePath(f);
        }
        fps = readFpsFromInfo(dir.absoluteFilePath("capture_info.txt"));
        slider->setEnabled(!frameFiles.isEmpty());
        prevBtn->setEnabled(!frameFiles.isEmpty());
        nextBtn->setEnabled(!frameFiles.isEmpty());
        int count = static_cast<int>(frameFiles.size());
        slider->setRange(0, std::max(0, count - 1));
        slider->setValue(0);
        updateTimeLabel(0);
        if (frameFiles.isEmpty()) {
            frameLabel->setText("Frame: -- / --");
        } else {
            frameLabel->setText(QString("Frame: %1 / %2").arg(1).arg(count));
            updateRecentFolders(dir.absolutePath());
        }
    }

    double readFpsFromInfo(const QString& infoPath) const {
        QFile f(infoPath);
        if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return 0.0;
        QTextStream ts(&f);
        double foundFps = 0.0;
        while (!ts.atEnd()) {
            QString line = ts.readLine().trimmed();
            if (line.startsWith("Internal FPS:", Qt::CaseInsensitive) ||
                line.startsWith("FPS:", Qt::CaseInsensitive)) {
                QStringList parts = line.split(":");
                if (parts.size() >= 2) {
                    bool ok = false;
                    double val = parts.last().trimmed().toDouble(&ok);
                    if (ok) foundFps = val;
                }
            }
        }
        return foundFps;
    }

    void loadFrame(int index) {
        if (frameFiles.isEmpty()) return;
        int count = static_cast<int>(frameFiles.size());
        index = std::clamp(index, 0, count - 1);
        QImageReader reader(frameFiles.at(index));
        reader.setAutoTransform(true);
        QImage img = reader.read();
        if (img.isNull()) {
            QMessageBox::warning(this, "Read error", "Failed to load image:\n" + reader.errorString());
            return;
        }
        imageView->setImage(img);
        frameLabel->setText(QString("Frame: %1 / %2").arg(index + 1).arg(count));
        updateTimeLabel(index);
    }

    void updateTimeLabel(int index) {
        int count = static_cast<int>(frameFiles.size());
        if (fps <= 0.0 || count == 0) {
            timeLabel->setText("Time: -- / --");
            return;
        }
        double totalSec = static_cast<double>(count) / fps;
        double currentSec = static_cast<double>(index) / fps;
        timeLabel->setText(QString("Time: %1 / %2").arg(formatTimeSeconds(currentSec)).arg(formatTimeSeconds(totalSec)));
    }

    ZoomImageView* imageView;
    QLabel* frameLabel;
    QLabel* timeLabel;
    QLineEdit* folderEdit;
    QComboBox* recentCombo;
    QSlider* slider;
    QPushButton* prevBtn;
    QPushButton* nextBtn;
    QStringList frameFiles;
    double fps;
};

void pruneLogs() {
    QFileInfo fi(gLogFile);
    QString baseDir = fi.dir().absolutePath();
    QString baseName = "session_log";
    QStringList files = QDir(baseDir).entryList(QStringList() << (baseName + "*.txt"), QDir::Files, QDir::Time);
    for (int i = 50; i < files.size(); ++i) {
        QString path = baseDir + "/" + files[i];
        QFile file(path);
        if (!file.remove()) {
            logMessageNoPrune(QString("Failed to remove log file %1: %2").arg(path, file.errorString()));
        }
    }
}

void logMessage(const QString& msg) {
    QMutexLocker locker(&gLogMutex);
    if (!gLogFile.isOpen()) {
        if (!gLogFile.open(QIODevice::Append | QIODevice::Text)) return;
    }
    QTextStream ts(&gLogFile);
    const QString line = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz") + " " + msg;
    ts << line << "\n";
    ts.flush();
    gLogFile.close();
    pruneLogs();
}

void logMessageNoPrune(const QString& msg) {
    QMutexLocker locker(&gLogMutex);
    if (!gLogFile.isOpen()) {
        if (!gLogFile.open(QIODevice::Append | QIODevice::Text)) return;
    }
    QTextStream ts(&gLogFile);
    const QString line = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz") + " " + msg;
    ts << line << "\n";
    ts.flush();
    gLogFile.close();
}

void qtLogHandler(QtMsgType type, const QMessageLogContext& ctx, const QString& msg) {
    Q_UNUSED(ctx);
    QString level;
    switch (type) {
        case QtDebugMsg: level="DEBUG"; break;
        case QtInfoMsg: level="INFO"; break;
        case QtWarningMsg: level="WARN"; break;
        case QtCriticalMsg: level="CRIT"; break;
        case QtFatalMsg: level="FATAL"; break;
    }
    logMessage(QString("[%1] %2").arg(level, msg));
}

void termHandler() {
    logMessage("std::terminate called");
    std::_Exit(1);
}

LONG WINAPI unhandledExceptionFilter(EXCEPTION_POINTERS* info) {
    if (gCrashHandled.exchange(true)) {
        return EXCEPTION_EXECUTE_HANDLER;
    }

    DWORD code = info && info->ExceptionRecord ? info->ExceptionRecord->ExceptionCode : 0;
    void* addr = info && info->ExceptionRecord ? info->ExceptionRecord->ExceptionAddress : nullptr;
    logMessage(QString("Unhandled exception: code=0x%1 addr=0x%2")
        .arg(code, 8, 16, QChar('0'))
        .arg(reinterpret_cast<quintptr>(addr), sizeof(quintptr) * 2, 16, QChar('0')));

    wchar_t basePath[MAX_PATH] = {};
    DWORD len = GetTempPathW(MAX_PATH, basePath);
    if (len == 0 || len >= MAX_PATH) {
        DWORD mlen = GetModuleFileNameW(nullptr, basePath, MAX_PATH);
        if (mlen > 0 && mlen < MAX_PATH) {
            for (DWORD i = mlen; i > 0; --i) {
                if (basePath[i] == L'\\' || basePath[i] == L'/') {
                    basePath[i + 1] = L'\0';
                    break;
                }
            }
        } else {
            basePath[0] = L'.';
            basePath[1] = L'\\';
            basePath[2] = L'\0';
        }
    }

    SYSTEMTIME st = {};
    GetLocalTime(&st);
    wchar_t fileName[128] = {};
    swprintf(fileName, 128, L"droplet_crash_%04d%02d%02d_%02d%02d%02d.dmp",
             st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);

    wchar_t dumpPath[MAX_PATH] = {};
    swprintf(dumpPath, MAX_PATH, L"%s%s", basePath, fileName);

    HANDLE hFile = CreateFileW(dumpPath, GENERIC_WRITE, FILE_SHARE_READ, nullptr,
                               CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (hFile != INVALID_HANDLE_VALUE) {
        MINIDUMP_EXCEPTION_INFORMATION mei = {};
        mei.ThreadId = GetCurrentThreadId();
        mei.ExceptionPointers = info;
        mei.ClientPointers = FALSE;
        MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), hFile,
                          MiniDumpNormal, info ? &mei : nullptr, nullptr, nullptr);
        CloseHandle(hFile);
        logMessage(QString("Crash dump saved: %1").arg(QString::fromWCharArray(dumpPath)));
    } else {
        DWORD err = GetLastError();
        logMessage(QString("Failed to create crash dump file. err=%1").arg(err));
    }
    return EXCEPTION_EXECUTE_HANDLER;
}

void installLogTees() {
    static LogTeeBuf loggerBuf(std::cout.rdbuf(), [](const QString& m){ logMessage(m); });
    static std::ostream loggerStream(&loggerBuf);
    std::cout.rdbuf(loggerStream.rdbuf());
    std::cerr.rdbuf(loggerStream.rdbuf());
}
} // namespace


int main(int argc, char *argv[]) {
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--cli") {
#ifdef _WIN32
            if (GetConsoleWindow() == nullptr) {
                AllocConsole();
                FILE* out = nullptr;
                FILE* err = nullptr;
                freopen_s(&out, "CONOUT$", "w", stdout);
                freopen_s(&err, "CONOUT$", "w", stderr);
            }
#endif
            return run_cli(argc, argv);
        }
    }
    QApplication app(argc, argv);
    QCoreApplication::setOrganizationName("Hamamatsu");
    QCoreApplication::setApplicationName("qt_hama_gui");

    gLogPath = QCoreApplication::applicationDirPath() + "/session_log.txt";
    gLogFile.setFileName(gLogPath);
    if (QFile::exists(gLogPath)) QFile::remove(gLogPath);
    pruneLogs();
    qInstallMessageHandler(qtLogHandler);
    std::set_terminate(termHandler);
    SetUnhandledExceptionFilter(unhandledExceptionFilter);
    installLogTees();
    logMessage(QString("Log file: %1").arg(gLogPath));

    QWidget window;
    window.setWindowTitle("Hamamatsu Live View");
    window.resize(1280, 800);
    window.setMinimumSize(900, 600);
    bool viewerOnly = false;

    // Live view area with zoomable/pannable view
    auto imageView = new ZoomImageView;
    imageView->setMinimumSize(640, 480);
    imageView->setStyleSheet("background:#000;");
    imageView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    // Info panel
    auto statusLabel = new QLabel("Status: Not initialized");
    statusLabel->setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard);
    statusLabel->setTextFormat(Qt::PlainText);
    auto statsLabel = new QLabel("Resolution: --\nFPS: --\nFrame: --");
    statsLabel->setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard);
    statsLabel->setTextFormat(Qt::PlainText);
    statsLabel->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    statsLabel->setMinimumWidth(220);
    // Buttons
    auto startBtn = new QPushButton("Start");
    auto stopBtn = new QPushButton("Stop");
    auto pipelineStartBtn = new QPushButton("Start Pipeline");
    auto pipelineStopBtn = new QPushButton("Stop Pipeline");
    pipelineStopBtn->setEnabled(false);
    auto reconnectBtn = new QPushButton("Reconnect");
    auto applyBtn = new QPushButton("Apply Settings");
    auto viewerBtn = new QPushButton("Viewer");
    auto tabWidget = new QTabWidget;

    // Settings controls
    auto presetCombo = new QComboBox;
    presetCombo->addItem("2304 x 2304", QVariant::fromValue(QSize(2304,2304)));
    presetCombo->addItem("2304 x 1152", QVariant::fromValue(QSize(2304,1152)));
    presetCombo->addItem("2304 x 576", QVariant::fromValue(QSize(2304,576)));
    presetCombo->addItem("2304 x 288", QVariant::fromValue(QSize(2304,288)));
    presetCombo->addItem("2304 x 144", QVariant::fromValue(QSize(2304,144)));
    presetCombo->addItem("2304 x 72", QVariant::fromValue(QSize(2304,72)));
    presetCombo->addItem("2304 x 36", QVariant::fromValue(QSize(2304,36)));
    presetCombo->addItem("2304 x 16", QVariant::fromValue(QSize(2304,16)));
    presetCombo->addItem("2304 x 8", QVariant::fromValue(QSize(2304,8)));
    presetCombo->addItem("2304 x 4", QVariant::fromValue(QSize(2304,4)));
    presetCombo->addItem("1152 x 1152", QVariant::fromValue(QSize(1152,1152)));
    presetCombo->addItem("1152 x 576", QVariant::fromValue(QSize(1152,576)));
    presetCombo->addItem("1152 x 288", QVariant::fromValue(QSize(1152,288)));
    presetCombo->addItem("1152 x 144", QVariant::fromValue(QSize(1152,144)));
    presetCombo->addItem("576 x 576", QVariant::fromValue(QSize(576,576)));
    presetCombo->addItem("576 x 288", QVariant::fromValue(QSize(576,288)));
    presetCombo->addItem("576 x 144", QVariant::fromValue(QSize(576,144)));
    presetCombo->addItem("288 x 288", QVariant::fromValue(QSize(288,288)));
    presetCombo->addItem("288 x 144", QVariant::fromValue(QSize(288,144)));
    presetCombo->addItem("144 x 144", QVariant::fromValue(QSize(144,144)));
    presetCombo->addItem("Custom", QVariant::fromValue(QSize(-1,-1)));

    auto customWidthSpin = new QSpinBox;
    customWidthSpin->setRange(1, 4096);
    customWidthSpin->setValue(2304);
    auto customHeightSpin = new QSpinBox;
    customHeightSpin->setRange(1, 4096);
    customHeightSpin->setValue(2304);
    presetCombo->addItem("512 x 128", QVariant::fromValue(QSize(512,128)));
    presetCombo->addItem("512 x 64", QVariant::fromValue(QSize(512,64)));
    presetCombo->addItem("256 x 64", QVariant::fromValue(QSize(256,64)));
    presetCombo->addItem("256 x 32", QVariant::fromValue(QSize(256,32)));

    auto binCombo = new QComboBox;
    binCombo->addItems({"1","2","4"});
    binCombo->setCurrentIndex(0);

    auto bitsCombo = new QComboBox;
    bitsCombo->addItems({"8","12","16"});
    bitsCombo->setCurrentIndex(0); // default 8-bit

    auto binIndCheck = new QCheckBox("Independent binning");
    auto binHSpin = new QSpinBox;
    auto binVSpin = new QSpinBox;
    binHSpin->setMinimum(1); binHSpin->setMaximum(8); binHSpin->setValue(1);
    binVSpin->setMinimum(1); binVSpin->setMaximum(8); binVSpin->setValue(1);

    auto exposureSpin = new QDoubleSpinBox;
    exposureSpin->setSuffix(" ms");
    exposureSpin->setDecimals(3);
    exposureSpin->setSingleStep(0.1);
    exposureSpin->setMinimum(0.01);
    exposureSpin->setMaximum(10000.0);
    exposureSpin->setValue(10.0);

    auto readoutCombo = new QComboBox;
    readoutCombo->addItem("Fastest", DCAMPROP_READOUTSPEED__FASTEST);
    readoutCombo->addItem("Slowest", DCAMPROP_READOUTSPEED__SLOWEST);
    readoutCombo->setCurrentIndex(0);

    auto logCheck = new QCheckBox("Enable logging (session_log.txt)");
    logCheck->setChecked(true);

    // Save controls
    QString defaultSaveDir = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    if (defaultSaveDir.isEmpty())
        defaultSaveDir = QCoreApplication::applicationDirPath();
    auto savePathEdit = new QLineEdit(defaultSaveDir);
    auto saveBrowseBtn = new QPushButton("...");
    auto saveOpenBtn = new QPushButton("Open Folder");
    auto saveStartBtn = new QPushButton("Start Save");
    auto saveStopBtn = new QPushButton("Stop Save");
    saveStopBtn->setEnabled(false);
    auto captureBtn = new QPushButton("Capture Frame");
    auto saveInfoLabel = new QLabel("Elapsed: 0.0 s\nFrames: 0");
    QDialog* savingDialog = nullptr;
    QLabel* savingDialogLabel = nullptr;
    QProgressBar* savingProgress = nullptr;

    auto displayEverySpin = new QSpinBox;
    displayEverySpin->setMinimum(1);
    displayEverySpin->setMaximum(1000);
    displayEverySpin->setValue(1);

    auto controlLayout = new QVBoxLayout;
    controlLayout->addWidget(statusLabel);
    controlLayout->addWidget(statsLabel);
    auto pipelineControlRow = new QHBoxLayout;
    pipelineControlRow->addWidget(pipelineStartBtn);
    pipelineControlRow->addWidget(pipelineStopBtn);
    controlLayout->addLayout(pipelineControlRow);

    auto tabFormats = new QWidget;
    auto grid = new QGridLayout;
    grid->addWidget(new QLabel("Preset"),0,0);
    grid->addWidget(presetCombo,0,1);
    grid->addWidget(new QLabel("Custom W/H"),1,0);
    auto customLayout = new QHBoxLayout;
    customLayout->addWidget(customWidthSpin);
    customLayout->addWidget(customHeightSpin);
    grid->addLayout(customLayout,1,1);
    grid->addWidget(new QLabel("Binning"),2,0);
    grid->addWidget(binCombo,2,1);
    grid->addWidget(binIndCheck,3,0,1,2);
    grid->addWidget(new QLabel("Bin H/V"),4,0);
    auto binHVLayout = new QHBoxLayout;
    binHVLayout->addWidget(binHSpin);
    binHVLayout->addWidget(binVSpin);
    grid->addLayout(binHVLayout,4,1);
    grid->addWidget(new QLabel("Bits"),5,0);
    grid->addWidget(bitsCombo,5,1);
    grid->addWidget(new QLabel("Exposure (ms)"),6,0);
    grid->addWidget(exposureSpin,6,1);
    grid->addWidget(new QLabel("Readout speed"),7,0);
    grid->addWidget(readoutCombo,7,1);
    grid->addWidget(new QLabel("Display every Nth frame"),8,0);
    grid->addWidget(displayEverySpin,8,1);
    grid->addWidget(logCheck,9,0,1,2);
    tabFormats->setLayout(grid);

    tabWidget->addTab(tabFormats, "Formats / Speed");

    auto saveLayout = new QGridLayout;
    saveLayout->addWidget(new QLabel("Save path"),0,0);
    saveLayout->addWidget(savePathEdit,0,1);
    saveLayout->addWidget(saveBrowseBtn,0,2);
    saveLayout->addWidget(saveOpenBtn,0,3);
    saveLayout->addWidget(saveInfoLabel,1,0,1,4);
    saveLayout->addWidget(saveStartBtn,2,2);
    saveLayout->addWidget(saveStopBtn,2,3);
    saveLayout->addWidget(captureBtn,3,2,1,2);
    auto saveWidget = new QWidget;
    saveWidget->setLayout(saveLayout);
    tabWidget->addTab(saveWidget, "Save");

    // Pipeline defaults (fast event detection)
    FastEventConfig pipelineDetectCfg;
    pipelineDetectCfg.bgFrames = 100;
    pipelineDetectCfg.bgUpdateFrames = 50;
    pipelineDetectCfg.resetFrames = 2;
    pipelineDetectCfg.minArea = -1.0;
    pipelineDetectCfg.minAreaFrac = 0.0;
    pipelineDetectCfg.maxAreaFrac = 0.10;
    pipelineDetectCfg.minBbox = 32;
    pipelineDetectCfg.margin = 5;
    pipelineDetectCfg.diffThresh = 15;
    pipelineDetectCfg.blurRadius = 1;
    pipelineDetectCfg.morphRadius = 1;
    pipelineDetectCfg.scale = 0.5;
    pipelineDetectCfg.gapFireShift = 0;

    // Pipeline controls (event detection + ONNX + DAQ)
    auto pipelineEnableCheck = new QCheckBox("Enable pipeline");
    pipelineEnableCheck->setChecked(false);
    auto pipelineStatusLabel = new QLabel("Pipeline: not loaded");
    pipelineStatusLabel->setWordWrap(true);

    auto onnxEdit = new QLineEdit;
    auto onnxBrowseBtn = new QPushButton("...");
    auto metaEdit = new QLineEdit;
    auto metaBrowseBtn = new QPushButton("...");
    auto outputEdit = new QLineEdit;
    auto outputBrowseBtn = new QPushButton("...");
    auto targetLabelEdit = new QLineEdit("Single");
    auto saveCropCheck = new QCheckBox("Save crops");
    auto saveOverlayCheck = new QCheckBox("Save overlays");
    auto loadPipelineBtn = new QPushButton("Load Pipeline");

    auto frameSkipSpin = new QSpinBox;
    frameSkipSpin->setRange(0, 1000);
    frameSkipSpin->setValue(0);

    auto daqChannelEdit = new QLineEdit("Dev1/ao0");
    auto amplitudeSpin = new QDoubleSpinBox;
    amplitudeSpin->setDecimals(3);
    amplitudeSpin->setRange(0.0, 10.0);
    amplitudeSpin->setValue(5.0);
    amplitudeSpin->setSuffix(" V");
    auto freqSpin = new QDoubleSpinBox;
    freqSpin->setDecimals(3);
    freqSpin->setRange(0.001, 200.0);
    freqSpin->setValue(10.0);
    freqSpin->setSuffix(" kHz");
    auto durationSpin = new QDoubleSpinBox;
    durationSpin->setDecimals(3);
    durationSpin->setRange(0.1, 10000.0);
    durationSpin->setValue(5.0);
    durationSpin->setSuffix(" ms");
    auto delaySpin = new QDoubleSpinBox;
    delaySpin->setDecimals(3);
    delaySpin->setRange(0.0, 10000.0);
    delaySpin->setValue(0.0);
    delaySpin->setSuffix(" ms");

    auto pipelineLayout = new QGridLayout;
    int row = 0;
    pipelineLayout->addWidget(pipelineEnableCheck, row++, 0, 1, 4);
    pipelineLayout->addWidget(new QLabel("ONNX path"), row, 0);
    pipelineLayout->addWidget(onnxEdit, row, 1, 1, 2);
    pipelineLayout->addWidget(onnxBrowseBtn, row++, 3);
    pipelineLayout->addWidget(new QLabel("Metadata path"), row, 0);
    pipelineLayout->addWidget(metaEdit, row, 1, 1, 2);
    pipelineLayout->addWidget(metaBrowseBtn, row++, 3);
    pipelineLayout->addWidget(new QLabel("Output dir"), row, 0);
    pipelineLayout->addWidget(outputEdit, row, 1, 1, 2);
    pipelineLayout->addWidget(outputBrowseBtn, row++, 3);
    pipelineLayout->addWidget(new QLabel("Target label"), row, 0);
    pipelineLayout->addWidget(targetLabelEdit, row++, 1, 1, 3);
    pipelineLayout->addWidget(saveCropCheck, row, 0);
    pipelineLayout->addWidget(saveOverlayCheck, row++, 1, 1, 2);
    pipelineLayout->addWidget(new QLabel("Frame skip"), row, 0);
    pipelineLayout->addWidget(frameSkipSpin, row++, 1, 1, 2);
    pipelineLayout->addWidget(loadPipelineBtn, row++, 0, 1, 2);
    pipelineLayout->addWidget(pipelineStatusLabel, row++, 0, 1, 4);

    auto pipelineWidget = new QWidget;
    pipelineWidget->setLayout(pipelineLayout);

    auto labviewStatusDot = new QLabel;
    labviewStatusDot->setFixedSize(14, 14);
    labviewStatusDot->setStyleSheet("background:#666;border-radius:7px;border:1px solid #333;");
    auto labviewStatusText = new QLabel("Disconnected");
    auto labviewStatusRow = new QHBoxLayout;
    labviewStatusRow->setContentsMargins(0, 0, 0, 0);
    labviewStatusRow->addWidget(labviewStatusDot);
    labviewStatusRow->addWidget(labviewStatusText, 1);

    auto labviewOutputLabel = new QLabel("Output: --");
    labviewOutputLabel->setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard);

    auto labviewLayout = new QGridLayout;
    int labRow = 0;
    labviewLayout->addWidget(new QLabel("Status"), labRow, 0);
    labviewLayout->addLayout(labviewStatusRow, labRow++, 1, 1, 2);
    labviewLayout->addWidget(labviewOutputLabel, labRow++, 0, 1, 3);
    labviewLayout->addWidget(new QLabel("Output range"), labRow, 0);
    labviewLayout->addWidget(new QLabel("-10 V to +10 V"), labRow++, 1, 1, 2);
    labviewLayout->addWidget(new QLabel("AO channel"), labRow, 0);
    labviewLayout->addWidget(daqChannelEdit, labRow++, 1, 1, 2);
    labviewLayout->addWidget(new QLabel("Amplitude"), labRow, 0);
    labviewLayout->addWidget(amplitudeSpin, labRow++, 1, 1, 2);
    labviewLayout->addWidget(new QLabel("Frequency (kHz)"), labRow, 0);
    labviewLayout->addWidget(freqSpin, labRow++, 1, 1, 2);
    labviewLayout->addWidget(new QLabel("Duration"), labRow, 0);
    labviewLayout->addWidget(durationSpin, labRow++, 1, 1, 2);
    labviewLayout->addWidget(new QLabel("Delay"), labRow, 0);
    labviewLayout->addWidget(delaySpin, labRow++, 1, 1, 2);
    auto labviewTestBtn = new QPushButton("Force Trigger");
    labviewLayout->addWidget(labviewTestBtn, labRow++, 0, 1, 2);
    auto labviewReconnectBtn = new QPushButton("Reconnect LabVIEW");
    labviewLayout->addWidget(labviewReconnectBtn, labRow++, 0, 1, 2);

    auto labviewWidget = new QWidget;
    labviewWidget->setLayout(labviewLayout);

    auto bgFramesSpin = new QSpinBox;
    bgFramesSpin->setRange(1, 10000);
    bgFramesSpin->setValue(pipelineDetectCfg.bgFrames);
    auto bgUpdateSpin = new QSpinBox;
    bgUpdateSpin->setRange(0, 10000);
    bgUpdateSpin->setValue(pipelineDetectCfg.bgUpdateFrames);
    auto resetFramesSpin = new QSpinBox;
    resetFramesSpin->setRange(1, 1000);
    resetFramesSpin->setValue(pipelineDetectCfg.resetFrames);
    auto minAreaSpin = new QDoubleSpinBox;
    minAreaSpin->setDecimals(1);
    minAreaSpin->setRange(-1.0, 1e9);
    minAreaSpin->setValue(pipelineDetectCfg.minArea);
    auto minAreaFracSpin = new QDoubleSpinBox;
    minAreaFracSpin->setDecimals(4);
    minAreaFracSpin->setRange(0.0, 1.0);
    minAreaFracSpin->setSingleStep(0.001);
    minAreaFracSpin->setValue(pipelineDetectCfg.minAreaFrac);
    auto maxAreaFracSpin = new QDoubleSpinBox;
    maxAreaFracSpin->setDecimals(4);
    maxAreaFracSpin->setRange(0.0, 1.0);
    maxAreaFracSpin->setSingleStep(0.001);
    maxAreaFracSpin->setValue(pipelineDetectCfg.maxAreaFrac);
    auto minBboxSpin = new QSpinBox;
    minBboxSpin->setRange(1, 10000);
    minBboxSpin->setValue(pipelineDetectCfg.minBbox);
    auto marginSpin = new QSpinBox;
    marginSpin->setRange(0, 10000);
    marginSpin->setValue(pipelineDetectCfg.margin);
    auto diffThreshSpin = new QSpinBox;
    diffThreshSpin->setRange(0, 255);
    diffThreshSpin->setValue(pipelineDetectCfg.diffThresh);
    auto blurRadiusSpin = new QSpinBox;
    blurRadiusSpin->setRange(0, 25);
    blurRadiusSpin->setValue(pipelineDetectCfg.blurRadius);
    auto morphRadiusSpin = new QSpinBox;
    morphRadiusSpin->setRange(0, 25);
    morphRadiusSpin->setValue(pipelineDetectCfg.morphRadius);
    auto scaleSpin = new QDoubleSpinBox;
    scaleSpin->setDecimals(3);
    scaleSpin->setRange(0.05, 1.0);
    scaleSpin->setSingleStep(0.05);
    scaleSpin->setValue(pipelineDetectCfg.scale);
    auto gapFireSpin = new QSpinBox;
    gapFireSpin->setRange(0, 10000);
    gapFireSpin->setValue(pipelineDetectCfg.gapFireShift);

    auto detectLayout = new QGridLayout;
    int detRow = 0;
    detectLayout->addWidget(new QLabel("Background frames"), detRow, 0);
    detectLayout->addWidget(bgFramesSpin, detRow++, 1);
    detectLayout->addWidget(new QLabel("BG update frames"), detRow, 0);
    detectLayout->addWidget(bgUpdateSpin, detRow++, 1);
    detectLayout->addWidget(new QLabel("Reset frames"), detRow, 0);
    detectLayout->addWidget(resetFramesSpin, detRow++, 1);
    detectLayout->addWidget(new QLabel("Min area (-1=auto)"), detRow, 0);
    detectLayout->addWidget(minAreaSpin, detRow++, 1);
    detectLayout->addWidget(new QLabel("Min area frac"), detRow, 0);
    detectLayout->addWidget(minAreaFracSpin, detRow++, 1);
    detectLayout->addWidget(new QLabel("Max area frac"), detRow, 0);
    detectLayout->addWidget(maxAreaFracSpin, detRow++, 1);
    detectLayout->addWidget(new QLabel("Min bbox"), detRow, 0);
    detectLayout->addWidget(minBboxSpin, detRow++, 1);
    detectLayout->addWidget(new QLabel("Margin"), detRow, 0);
    detectLayout->addWidget(marginSpin, detRow++, 1);
    detectLayout->addWidget(new QLabel("Diff thresh"), detRow, 0);
    detectLayout->addWidget(diffThreshSpin, detRow++, 1);
    detectLayout->addWidget(new QLabel("Blur radius"), detRow, 0);
    detectLayout->addWidget(blurRadiusSpin, detRow++, 1);
    detectLayout->addWidget(new QLabel("Morph radius"), detRow, 0);
    detectLayout->addWidget(morphRadiusSpin, detRow++, 1);
    detectLayout->addWidget(new QLabel("Scale"), detRow, 0);
    detectLayout->addWidget(scaleSpin, detRow++, 1);
    detectLayout->addWidget(new QLabel("Gap fire shift"), detRow, 0);
    detectLayout->addWidget(gapFireSpin, detRow++, 1);
    auto detectWidget = new QWidget;
    detectWidget->setLayout(detectLayout);

    auto statsEventsLabel = new QLabel("Events: 0");
    auto statsClassLabel = new QLabel("Classes:\n(none)");
    auto statsHitLabel = new QLabel("Hits: 0\nWastes: 0");
    auto statsLastLabel = new QLabel("Last event: --");
    statsEventsLabel->setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard);
    statsClassLabel->setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard);
    statsHitLabel->setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard);
    statsLastLabel->setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard);
    statsClassLabel->setWordWrap(true);
    statsLastLabel->setWordWrap(true);
    auto statsResetBtn = new QPushButton("Reset Stats");
    auto statsShowBtn = new QPushButton("Show Figures");

    auto statsLayout = new QVBoxLayout;
    statsLayout->addWidget(statsEventsLabel);
    statsLayout->addWidget(statsHitLabel);
    statsLayout->addWidget(statsLastLabel);
    statsLayout->addWidget(statsClassLabel, 1);
    statsLayout->addWidget(statsShowBtn);
    statsLayout->addWidget(statsResetBtn);
    auto statsWidget = new QWidget;
    statsWidget->setLayout(statsLayout);

    auto seqFolderEdit = new QLineEdit;
    seqFolderEdit->setPlaceholderText("Select sequence folder...");
    auto seqBrowseBtn = new QPushButton("...");
    auto seqLoadBtn = new QPushButton("Load into memory");
    auto seqStartBtn = new QPushButton("Start Test");
    auto seqStopBtn = new QPushButton("Stop");
    seqStartBtn->setEnabled(false);
    seqStopBtn->setEnabled(false);

    auto seqFpsSpin = new QDoubleSpinBox;
    seqFpsSpin->setDecimals(2);
    seqFpsSpin->setRange(0.1, 100000.0);
    seqFpsSpin->setValue(500.0);

    auto seqStatusLabel = new QLabel("No sequence loaded.");
    seqStatusLabel->setWordWrap(true);
    auto seqLogLabel = new QLabel("Log: (none)");
    seqLogLabel->setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard);

    auto seqLayout = new QGridLayout;
    int seqRow = 0;
    seqLayout->addWidget(new QLabel("Folder"), seqRow, 0);
    seqLayout->addWidget(seqFolderEdit, seqRow, 1, 1, 2);
    seqLayout->addWidget(seqBrowseBtn, seqRow++, 3);
    seqLayout->addWidget(new QLabel("FPS"), seqRow, 0);
    seqLayout->addWidget(seqFpsSpin, seqRow++, 1, 1, 2);
    seqLayout->addWidget(seqLoadBtn, seqRow, 0, 1, 2);
    seqLayout->addWidget(seqStartBtn, seqRow, 2);
    seqLayout->addWidget(seqStopBtn, seqRow++, 3);
    seqLayout->addWidget(seqStatusLabel, seqRow++, 0, 1, 4);
    seqLayout->addWidget(seqLogLabel, seqRow++, 0, 1, 4);
    auto seqWidget = new QWidget;
    seqWidget->setLayout(seqLayout);

    auto pipelineTabs = new QTabWidget;
    pipelineTabs->addTab(pipelineWidget, "Pipeline");
    pipelineTabs->addTab(labviewWidget, "LabVIEW");
    pipelineTabs->addTab(detectWidget, "Event Detection");
    pipelineTabs->addTab(statsWidget, "Stats");
    pipelineTabs->addTab(seqWidget, "Sequence Test");
    auto pipelineGroup = new QGroupBox("Pipeline / LabVIEW");
    auto pipelineGroupLayout = new QVBoxLayout;
    pipelineGroupLayout->addWidget(pipelineTabs);
    pipelineGroup->setLayout(pipelineGroupLayout);

    auto btnRow = new QHBoxLayout;
    btnRow->addWidget(startBtn);
    btnRow->addWidget(stopBtn);
    btnRow->addWidget(reconnectBtn);

    controlLayout->addWidget(tabWidget);
    controlLayout->addWidget(pipelineGroup);
    controlLayout->addWidget(viewerBtn);
    controlLayout->addLayout(btnRow);
    controlLayout->addWidget(applyBtn);
    controlLayout->addStretch(1);

    auto rightWidget = new QWidget;
    rightWidget->setLayout(controlLayout);
    rightWidget->setMinimumWidth(320);

    auto mainLayout = new QHBoxLayout;
    mainLayout->addWidget(imageView, 3);
    mainLayout->addWidget(rightWidget, 1);
    window.setLayout(mainLayout);

    // Logging helper
    auto logLine = [&](const QString& msg) {
        if (!logCheck->isChecked()) return;
        logMessage(msg);
    };

    auto csvQuote = [](const QString& s)->QString {
        QString out = s;
        out.replace("\"", "\"\"");
        return "\"" + out + "\"";
    };

    auto buildRunOutputDir = [&](const QString& prefix)->QString {
        QString base = outputEdit->text().trimmed();
        if (base.isEmpty()) base = QCoreApplication::applicationDirPath();
        QDir baseDir(base);
        QString leaf = baseDir.dirName();
        if (leaf.startsWith("sequence_") || leaf.startsWith("live_") || leaf.startsWith("test_")) {
            baseDir.cdUp();
        }
        baseDir.mkpath(".");
        QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
        QString runName = QString("%1_%2").arg(prefix, stamp);
        QString runDir = baseDir.filePath(runName);
        baseDir.mkpath(runName);
        return runDir;
    };

    auto pickExistingPath = [](const QStringList& candidates)->QString {
        for (const auto& c : candidates) {
            if (QFileInfo::exists(c)) return c;
        }
        return candidates.isEmpty() ? QString() : candidates.first();
    };

    QString appDir = QCoreApplication::applicationDirPath();
    auto findModelUpwards = [&](const QString& filename)->QString {
        QDir dir(appDir);
        for (int i = 0; i < 6; ++i) {
            QString candidate = dir.filePath("models/" + filename);
            if (QFileInfo::exists(candidate)) return candidate;
            if (!dir.cdUp()) break;
        }
        return QString();
    };
    QString defaultOnnxRel = "../../../models/squeezenet_final_new_condition.onnx";
    QString defaultMetaRel = "../../../models/metadata.json";
    QString defaultOnnxAbs = QDir(appDir).absoluteFilePath(defaultOnnxRel);
    QString defaultMetaAbs = QDir(appDir).absoluteFilePath(defaultMetaRel);
    QString defaultOnnxFromProject = findModelUpwards("squeezenet_final_new_condition.onnx");
    QString defaultMetaFromProject = findModelUpwards("metadata.json");
    QStringList onnxCandidates = {
        defaultOnnxFromProject,
        defaultOnnxAbs,
        appDir + "/squeezenet_final_new_condition.onnx",
        appDir + "/models/squeezenet_final_new_condition.onnx",
        appDir + "/../models/squeezenet_final_new_condition.onnx",
        appDir + "/../../models/squeezenet_final_new_condition.onnx"
    };
    QStringList metaCandidates = {
        defaultMetaFromProject,
        defaultMetaAbs,
        appDir + "/metadata.json",
        appDir + "/models/metadata.json",
        appDir + "/../models/metadata.json",
        appDir + "/../../models/metadata.json"
    };
    QString onnxPicked = pickExistingPath(onnxCandidates);
    if (onnxPicked.isEmpty()) {
        onnxPicked = defaultOnnxRel;
    } else {
        onnxPicked = QDir(appDir).relativeFilePath(onnxPicked);
    }
    QString metaPicked = pickExistingPath(metaCandidates);
    if (metaPicked.isEmpty()) {
        metaPicked = defaultMetaRel;
    } else {
        metaPicked = QDir(appDir).relativeFilePath(metaPicked);
    }
    onnxEdit->setText(onnxPicked);
    metaEdit->setText(metaPicked);
    if (outputEdit->text().isEmpty()) {
        outputEdit->setText(QDir(defaultSaveDir).filePath("pipeline_output"));
    }

    auto resolveAppRelative = [&](const QString& path)->QString {
        if (path.isEmpty()) return path;
        QFileInfo info(path);
        if (info.isAbsolute()) return info.absoluteFilePath();
        QString abs = QDir(appDir).absoluteFilePath(path);
        if (QFileInfo::exists(abs)) return abs;
        QString fallback = findModelUpwards(QFileInfo(path).fileName());
        if (!fallback.isEmpty()) return fallback;
        return abs;
    };

    QTimer labviewApplyTimer;
    bool autoApplyLabview = true;
    std::function<void()> scheduleLabviewApply = [](){};
    auto setLabviewStatus = [&](const QString& text, const QString& color){
        labviewStatusText->setText(text);
        labviewStatusDot->setStyleSheet(QString("background:%1;border-radius:7px;border:1px solid #333;").arg(color));
    };
    labviewApplyTimer.setSingleShot(true);
    labviewApplyTimer.setInterval(300);
    auto updateLabviewOutput = [&](){
        QString channel = daqChannelEdit->text().trimmed();
        if (channel.isEmpty()) {
            labviewOutputLabel->setText("Output: (disabled)");
            return;
        }
        labviewOutputLabel->setText(QString("Output: %1 | amp=%2 V freq=%3 kHz dur=%4 ms delay=%5 ms")
            .arg(channel)
            .arg(amplitudeSpin->value(), 0, 'f', 2)
            .arg(freqSpin->value(), 0, 'f', 3)
            .arg(durationSpin->value(), 0, 'f', 2)
            .arg(delaySpin->value(), 0, 'f', 2));
    };
    updateLabviewOutput();
    setLabviewStatus("Disconnected", "#666");

    QObject::connect(daqChannelEdit, &QLineEdit::textChanged, [&](){
        updateLabviewOutput();
        scheduleLabviewApply();
    });
    QObject::connect(amplitudeSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), [&](){
        updateLabviewOutput();
        scheduleLabviewApply();
    });
    QObject::connect(freqSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), [&](){
        updateLabviewOutput();
        scheduleLabviewApply();
    });
    QObject::connect(durationSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), [&](){
        updateLabviewOutput();
        scheduleLabviewApply();
    });
    QObject::connect(delaySpin, qOverload<double>(&QDoubleSpinBox::valueChanged), [&](){
        updateLabviewOutput();
        scheduleLabviewApply();
    });

    std::shared_ptr<std::vector<SequenceFrame>> sequenceFrames;
    QMutex sequenceMutex;
    std::atomic<bool> sequenceRunning(false);
    std::atomic<bool> sequenceStarting(false);
    std::atomic<bool> sequenceStop(false);
    std::thread sequenceThread;
    bool sequencePrevPipelineChecked = false;
    StatsTracker stats;
    QMutex statsMutex;
    QMutex liveLogMutex;
    std::vector<LiveLogRecord> liveLog;
    std::atomic<bool> liveLogging(false);
    QDateTime liveLogStart;
    std::function<void()> startLiveLogging;
    std::function<void()> stopLiveLogging;

    imageView->setZoomChanged(nullptr);

    DcamController controller(&window);
    FrameGrabber grabber(&controller);
    QImage lastFrame;
    FrameMeta lastMeta{};
    PipelineRunner pipeline;
    QMutex pipelineMutex;
    std::atomic<bool> pipelineEnabled(false);
    bool labviewTriggerReady = false;

    auto refreshExposureLimits = [&](){
        if (!controller.isOpened()) return;
        DCAMPROP_ATTR attr = {};
        attr.cbSize = sizeof(attr);
        attr.iProp = DCAM_IDPROP_EXPOSURETIME;
        if (!failed(dcamprop_getattr(controller.handle(), &attr))) {
            double min_ms = attr.valuemin * 1000.0;
            double max_ms = attr.valuemax * 1000.0;
            exposureSpin->setMinimum(min_ms);
            exposureSpin->setMaximum(max_ms);
        }
        double cur=0;
        if (!failed(dcamprop_getvalue(controller.handle(), DCAM_IDPROP_EXPOSURETIME, &cur))) {
            exposureSpin->setValue(cur * 1000.0);
        }
    };

    QObject::connect(presetCombo, qOverload<int>(&QComboBox::currentIndexChanged), [&](int){
        bool isCustom = presetCombo->currentData().toSize().width() < 0;
        customWidthSpin->setEnabled(isCustom);
        customHeightSpin->setEnabled(isCustom);
    });
    // Initialize state
    customWidthSpin->setEnabled(false);
    customHeightSpin->setEnabled(false);

    auto applySettings = [&](){
        QSize preset = presetCombo->currentData().toSize();
        bool isCustom = preset.width() < 0 || preset.height() < 0;
        int bin = binCombo->currentText().toInt();
        int bits = bitsCombo->currentText().toInt();
        int pixel = (bits > 8) ? DCAM_PIXELTYPE_MONO16 : DCAM_PIXELTYPE_MONO8;
        double exp_ms = exposureSpin->value();
        double exp_s = exp_ms / 1000.0;
        int readout = readoutCombo->currentData().toInt();
        ApplySettings s;
        s.width = isCustom ? customWidthSpin->value() : preset.width();
        s.height = isCustom ? customHeightSpin->value() : preset.height();
        s.binning = bin;
        s.binningIndependent = binIndCheck->isChecked();
        s.binH = binHSpin->value();
        s.binV = binVSpin->value();
        s.bits = bits;
        s.pixelType = pixel;
        s.exposure_s = exp_s;
        s.readoutSpeed = readout;
        s.bundleEnabled = false;
        s.bundleCount = 0;
        logLine(QString("Apply: preset=%1x%2 bin=%3 binH=%4 binV=%5 bits=%6 pixType=%7 exp_ms=%8 readout=%9")
            .arg(s.width).arg(s.height).arg(s.binning).arg(s.binH).arg(s.binV)
            .arg(s.bits).arg(s.pixelType).arg(exp_ms,0,'f',3).arg(readout));
        QString err = controller.apply(s);
        auto logReadback = [&](){
            if (!controller.isOpened()) return;
            HDCAM h = controller.handle();
            double w=0,hgt=0,binrb=0,bitsrb=0,pt=0,fps=0,ro=0,exp_rb=0,binHrb=0,binVrb=0;
            dcamprop_getvalue(h, DCAM_IDPROP_IMAGE_WIDTH, &w);
            dcamprop_getvalue(h, DCAM_IDPROP_IMAGE_HEIGHT, &hgt);
            dcamprop_getvalue(h, DCAM_IDPROP_BINNING, &binrb);
            dcamprop_getvalue(h, DCAM_IDPROP_BITSPERCHANNEL, &bitsrb);
            dcamprop_getvalue(h, DCAM_IDPROP_IMAGE_PIXELTYPE, &pt);
            dcamprop_getvalue(h, DCAM_IDPROP_INTERNALFRAMERATE, &fps);
            dcamprop_getvalue(h, DCAM_IDPROP_READOUTSPEED, &ro);
            dcamprop_getvalue(h, DCAM_IDPROP_EXPOSURETIME, &exp_rb);
            dcamprop_getvalue(h, DCAM_IDPROP_BINNING_HORZ, &binHrb);
            dcamprop_getvalue(h, DCAM_IDPROP_BINNING_VERT, &binVrb);
            logLine(QString("Readback: w=%1 h=%2 bin=%3 binH=%4 binV=%5 bits=%6 pixType=%7 exp_ms=%8 camfps=%9 readout=%10")
                .arg(w,0,'f',0).arg(hgt,0,'f',0).arg(binrb,0,'f',1)
                .arg(binHrb,0,'f',1).arg(binVrb,0,'f',1)
                .arg(bitsrb,0,'f',0).arg(pt,0,'f',0)
                .arg(exp_rb*1000.0,0,'f',3).arg(fps,0,'f',1).arg(ro,0,'f',0));
        };
        if (!err.isEmpty()) {
            if (err.startsWith("WARN:")) {
                statusLabel->setText("Applied with warnings: " + err.mid(5));
                grabber.startGrabbing();
            } else {
                statusLabel->setText("Apply error: " + err);
            }
        } else {
            statusLabel->setText("Applied. Streaming");
            grabber.startGrabbing();
        }
        grabber.setDisplayEvery(displayEverySpin->value());
        logReadback();
        if (pipeline.isReady()) {
            QMutexLocker lock(&pipelineMutex);
            pipeline.reset();
            pipelineStatusLabel->setText("Pipeline: warming (settings changed)");
        }
    };

    QTimer applyTimer;
    applyTimer.setSingleShot(true);
    applyTimer.setInterval(250);
    bool autoApplyCamera = false;
    QObject::connect(&applyTimer, &QTimer::timeout, [&](){
        if (viewerOnly) return;
        applySettings();
    });
    auto scheduleApplySettings = [&](){
        if (!autoApplyCamera || viewerOnly) return;
        if (!controller.isOpened()) return;
        applyTimer.start();
    };

    auto setViewerOnly = [&](){
        viewerOnly = true;
        statusLabel->setText("Viewer-only mode (camera init failed).");
        startBtn->setEnabled(false);
        stopBtn->setEnabled(false);
        reconnectBtn->setEnabled(false);
        applyBtn->setEnabled(false);
        tabWidget->setEnabled(false);
    };

    auto doInit = [&]()->bool{
        QString err = controller.initAndOpen();
        if (!err.isEmpty()) {
            statusLabel->setText("Init error: " + err);
            auto choice = QMessageBox::question(
                &window,
                "Init failed",
                "Camera init failed:\n" + err + "\n\nLaunch viewer-only mode?",
                QMessageBox::Yes | QMessageBox::No,
                QMessageBox::Yes);
            if (choice == QMessageBox::Yes) {
                logLine("Init failed; switching to viewer-only mode.");
                setViewerOnly();
                return false;
            }
            QMetaObject::invokeMethod(&app, "quit", Qt::QueuedConnection);
            return false;
        } else {
            statusLabel->setText("Initialized.");
            refreshExposureLimits();
            // Force default exposure to 10 ms on camera and UI
            dcamprop_setvalue(controller.handle(), DCAM_IDPROP_EXPOSURETIME, 0.010);
            exposureSpin->setValue(10.0);
            // Apply selected bits/pixel type on init
            int bits = bitsCombo->currentText().toInt();
            int pixel = (bits > 8) ? DCAM_PIXELTYPE_MONO16 : DCAM_PIXELTYPE_MONO8;
            dcamprop_setvalue(controller.handle(), DCAM_IDPROP_IMAGE_PIXELTYPE, pixel);
            dcamprop_setvalue(controller.handle(), DCAM_IDPROP_BITSPERCHANNEL, bits);
            autoApplyCamera = true;
            logLine("Init success");
            return true;
        }
    };

    QObject::connect(reconnectBtn, &QPushButton::clicked, [&](){
        QString err = controller.reconnect();
        if (!err.isEmpty()) statusLabel->setText("Reconnect error: " + err);
        else {
            statusLabel->setText("Reconnected.");
            refreshExposureLimits();
        }
    });

    QObject::connect(startBtn, &QPushButton::clicked, [&](){
        if (viewerOnly) return;
        if (controller.isOpened()) {
            int bits = bitsCombo->currentText().toInt();
            int pixel = (bits > 8) ? DCAM_PIXELTYPE_MONO16 : DCAM_PIXELTYPE_MONO8;
            dcamprop_setvalue(controller.handle(), DCAM_IDPROP_IMAGE_PIXELTYPE, pixel);
            dcamprop_setvalue(controller.handle(), DCAM_IDPROP_BITSPERCHANNEL, bits);
        }
        QString err = controller.start();
        if (!err.isEmpty()) statusLabel->setText("Start error: " + err);
        else {
            statusLabel->setText("Capture started.");
            grabber.startGrabbing();
            if (pipeline.isReady()) {
                QMutexLocker lock(&pipelineMutex);
                pipeline.reset();
                pipelineStatusLabel->setText("Pipeline: warming (capture start)");
            }
        }
    });

    QObject::connect(stopBtn, &QPushButton::clicked, [&](){
        if (viewerOnly) return;
        grabber.stopGrabbing();
        controller.stop();
        statusLabel->setText("Capture stopped.");
        if (pipelineEnabled.load()) {
            pipelineStatusLabel->setText("Pipeline: paused");
        }
    });

    QObject::connect(applyBtn, &QPushButton::clicked, [&](){
        if (viewerOnly) return;
        applySettings();
    });

    QObject::connect(presetCombo, qOverload<int>(&QComboBox::currentIndexChanged), [&](){
        scheduleApplySettings();
    });
    QObject::connect(customWidthSpin, qOverload<int>(&QSpinBox::valueChanged), [&](){
        scheduleApplySettings();
    });
    QObject::connect(customHeightSpin, qOverload<int>(&QSpinBox::valueChanged), [&](){
        scheduleApplySettings();
    });
    QObject::connect(binCombo, qOverload<int>(&QComboBox::currentIndexChanged), [&](){
        scheduleApplySettings();
    });
    QObject::connect(binIndCheck, &QCheckBox::toggled, [&](){
        scheduleApplySettings();
    });
    QObject::connect(binHSpin, qOverload<int>(&QSpinBox::valueChanged), [&](){
        scheduleApplySettings();
    });
    QObject::connect(binVSpin, qOverload<int>(&QSpinBox::valueChanged), [&](){
        scheduleApplySettings();
    });
    QObject::connect(bitsCombo, qOverload<int>(&QComboBox::currentIndexChanged), [&](){
        scheduleApplySettings();
    });
    QObject::connect(exposureSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), [&](){
        scheduleApplySettings();
    });
    QObject::connect(readoutCombo, qOverload<int>(&QComboBox::currentIndexChanged), [&](){
        scheduleApplySettings();
    });

    QPointer<ViewerWindow> viewerWindow;
    QPointer<StatsFigureWindow> statsFigureWindow;
    QObject::connect(viewerBtn, &QPushButton::clicked, [&](){
        if (viewerWindow) {
            viewerWindow->raise();
            viewerWindow->activateWindow();
            return;
        }
        viewerWindow = new ViewerWindow(nullptr);
        viewerWindow->setAttribute(Qt::WA_DeleteOnClose);
        QObject::connect(viewerWindow, &QObject::destroyed, [&](){ viewerWindow = nullptr; });
        viewerWindow->show();
    });

    // Save state
    auto saveBuffer = std::make_shared<std::vector<QImage>>();
    auto saveMutex = std::make_shared<QMutex>();
    std::atomic<bool> recording{false};
    std::atomic<bool> saving{false};
    QElapsedTimer recordTimer;
    QDateTime recordStartTime;
    std::atomic<int> recordedFrames{0};
    QTimer saveInfoTimer;
    saveInfoTimer.setInterval(200);

    QObject::connect(saveBrowseBtn, &QPushButton::clicked, [&](){
        QString dir = QFileDialog::getExistingDirectory(&window, "Select save directory", savePathEdit->text());
        if (!dir.isEmpty()) savePathEdit->setText(dir);
    });
    QObject::connect(saveOpenBtn, &QPushButton::clicked, [&](){
        QString dir = savePathEdit->text();
        if (dir.isEmpty()) dir = QCoreApplication::applicationDirPath();
        QDir().mkpath(dir);
        QDesktopServices::openUrl(QUrl::fromLocalFile(dir));
    });

    QObject::connect(onnxBrowseBtn, &QPushButton::clicked, [&](){
        QString file = QFileDialog::getOpenFileName(&window, "Select ONNX model", onnxEdit->text(),
                                                    "ONNX Model (*.onnx)");
        if (!file.isEmpty()) onnxEdit->setText(file);
    });
    QObject::connect(metaBrowseBtn, &QPushButton::clicked, [&](){
        QString file = QFileDialog::getOpenFileName(&window, "Select metadata JSON", metaEdit->text(),
                                                    "JSON (*.json)");
        if (!file.isEmpty()) metaEdit->setText(file);
    });
    QObject::connect(outputBrowseBtn, &QPushButton::clicked, [&](){
        QString dir = QFileDialog::getExistingDirectory(&window, "Select output directory", outputEdit->text());
        if (!dir.isEmpty()) outputEdit->setText(dir);
    });

    QObject::connect(pipelineEnableCheck, &QCheckBox::toggled, [&](bool enabled){
        pipelineEnabled.store(enabled);
        pipelineStartBtn->setEnabled(!enabled && !sequenceRunning.load());
        pipelineStopBtn->setEnabled(enabled && !sequenceRunning.load());
        if (!enabled) {
            pipelineStatusLabel->setText("Pipeline: paused");
            setLabviewStatus("Disabled", "#666");
            if (!sequenceRunning.load() && !sequenceStarting.load()) {
                stopLiveLogging();
            }
        } else if (daqChannelEdit->text().trimmed().isEmpty()) {
            setLabviewStatus("Disabled", "#666");
        } else if (labviewTriggerReady) {
            setLabviewStatus("Connected", "#2ecc71");
        } else {
            setLabviewStatus("Disconnected", "#c0392b");
        }
        if (enabled && !sequenceRunning.load() && !sequenceStarting.load()) {
            bool ready = false;
            {
                QMutexLocker lock(&pipelineMutex);
                ready = pipeline.isReady();
            }
            if (ready && !liveLogging.load()) {
                startLiveLogging();
            }
        }
    });

    auto loadPipeline = [&](bool enableAfter){
        logMessage("Pipeline init requested");
        PipelineConfig cfg;
        QString onnxResolved = resolveAppRelative(onnxEdit->text());
        QString metaResolved = resolveAppRelative(metaEdit->text());
        cfg.onnxPath = onnxResolved.toStdString();
        cfg.metadataPath = metaResolved.toStdString();
        cfg.targetLabel = targetLabelEdit->text().toStdString();
        cfg.outputDir = outputEdit->text().toStdString();
        cfg.saveCrop = saveCropCheck->isChecked();
        cfg.saveOverlay = saveOverlayCheck->isChecked();
        cfg.cropSize = 64;
        cfg.frameSkip = frameSkipSpin->value();
        pipelineDetectCfg.bgFrames = bgFramesSpin->value();
        pipelineDetectCfg.bgUpdateFrames = bgUpdateSpin->value();
        pipelineDetectCfg.resetFrames = resetFramesSpin->value();
        pipelineDetectCfg.minArea = minAreaSpin->value();
        pipelineDetectCfg.minAreaFrac = minAreaFracSpin->value();
        pipelineDetectCfg.maxAreaFrac = maxAreaFracSpin->value();
        pipelineDetectCfg.minBbox = minBboxSpin->value();
        pipelineDetectCfg.margin = marginSpin->value();
        pipelineDetectCfg.diffThresh = diffThreshSpin->value();
        pipelineDetectCfg.blurRadius = blurRadiusSpin->value();
        pipelineDetectCfg.morphRadius = morphRadiusSpin->value();
        pipelineDetectCfg.scale = scaleSpin->value();
        pipelineDetectCfg.gapFireShift = gapFireSpin->value();
        cfg.detect = pipelineDetectCfg;
        cfg.daq.channel = daqChannelEdit->text().trimmed().toStdString();
        cfg.daq.rangeMin = -10.0;
        cfg.daq.rangeMax = 10.0;
        cfg.daq.amplitude = amplitudeSpin->value();
        cfg.daq.frequencyHz = freqSpin->value() * 1000.0;
        cfg.daq.durationMs = durationSpin->value();
        cfg.daq.delayMs = delaySpin->value();

        logMessage(QString("Pipeline init paths: onnx=%1 meta=%2").arg(onnxEdit->text(), metaEdit->text()));
        logMessage(QString("Pipeline init resolved paths: onnx=%1 meta=%2").arg(onnxResolved, metaResolved));
        logMessage(QString("DAQ config: channel=%1 range=[-10,10] amp=%2V freq=%3Hz duration=%4ms delay=%5ms")
            .arg(daqChannelEdit->text().trimmed())
            .arg(amplitudeSpin->value(), 0, 'f', 3)
            .arg(freqSpin->value() * 1000.0, 0, 'f', 1)
            .arg(durationSpin->value(), 0, 'f', 3)
            .arg(delaySpin->value(), 0, 'f', 3));

        std::string err;
        {
            QMutexLocker locker(&pipelineMutex);
            if (!pipeline.init(cfg, err)) {
                pipelineStatusLabel->setText(QString("Pipeline error: %1").arg(QString::fromStdString(err)));
                pipelineEnabled.store(false);
                pipelineEnableCheck->setChecked(false);
                labviewTriggerReady = false;
                setLabviewStatus("Disconnected", "#c0392b");
                logMessage(QString("Pipeline init failed: %1").arg(QString::fromStdString(err)));
                return;
            }
            pipeline.reset();
            labviewTriggerReady = pipeline.isTriggerReady();
        }

        if (!err.empty()) {
            pipelineStatusLabel->setText(QString("Pipeline ready (DAQ off): %1").arg(QString::fromStdString(err)));
            logMessage(QString("Pipeline init warning: %1").arg(QString::fromStdString(err)));
        } else {
            pipelineStatusLabel->setText("Pipeline ready");
            logMessage("Pipeline init success");
        }
        pipelineEnabled.store(enableAfter);
        if (enableAfter) {
            pipelineEnableCheck->setChecked(true);
        }
        if (enableAfter && !sequenceRunning.load() && !sequenceStarting.load() && !liveLogging.load()) {
            startLiveLogging();
        }

        if (cfg.daq.channel.empty()) {
            setLabviewStatus("Disabled", "#666");
        } else if (labviewTriggerReady) {
            setLabviewStatus("Connected", "#2ecc71");
        } else {
            setLabviewStatus("Disconnected", "#c0392b");
        }
        updateLabviewOutput();
    };

    QObject::connect(&labviewApplyTimer, &QTimer::timeout, [&](){
        if (viewerOnly) return;
        bool enableAfter = pipelineEnableCheck->isChecked();
        loadPipeline(enableAfter);
    });
    scheduleLabviewApply = [&](){
        if (!autoApplyLabview) return;
        labviewApplyTimer.start();
    };

    QObject::connect(loadPipelineBtn, &QPushButton::clicked, [&](){
        loadPipeline(true);
    });

    QObject::connect(pipelineStartBtn, &QPushButton::clicked, [&](){
        if (sequenceRunning.load()) return;
        QString runDir = buildRunOutputDir("live");
        if (!runDir.isEmpty()) {
            outputEdit->setText(runDir);
        }
        bool ready = false;
        {
            QMutexLocker lock(&pipelineMutex);
            ready = pipeline.isReady();
        }
        if (!ready) {
            loadPipeline(true);
        } else {
            pipelineEnableCheck->setChecked(true);
        }
        statusLabel->setText("Pipeline started.");
    });

    QObject::connect(pipelineStopBtn, &QPushButton::clicked, [&](){
        if (sequenceRunning.load()) return;
        pipelineEnableCheck->setChecked(false);
        statusLabel->setText("Pipeline stopped.");
    });

    QObject::connect(labviewReconnectBtn, &QPushButton::clicked, [&](){
        bool enableAfter = pipelineEnableCheck->isChecked();
        loadPipeline(enableAfter);
    });

    QObject::connect(labviewTestBtn, &QPushButton::clicked, [&](){
        std::string trigErr;
        bool ok = false;
        bool usedPipeline = false;
        {
            QMutexLocker lock(&pipelineMutex);
            if (pipeline.isTriggerReady()) {
                ok = pipeline.fireTrigger(trigErr);
                usedPipeline = true;
            }
        }
        if (!usedPipeline) {
            DaqConfig cfg;
            cfg.channel = daqChannelEdit->text().trimmed().toStdString();
            cfg.rangeMin = -10.0;
            cfg.rangeMax = 10.0;
            cfg.amplitude = amplitudeSpin->value();
            cfg.frequencyHz = freqSpin->value() * 1000.0;
            cfg.durationMs = durationSpin->value();
            cfg.delayMs = delaySpin->value();
            DaqTrigger manualTrigger;
            if (!manualTrigger.init(cfg, trigErr)) {
                ok = false;
            } else {
                ok = manualTrigger.fire(trigErr);
            }
        }
        if (ok) {
            statusLabel->setText("DAQ trigger sent.");
            setLabviewStatus("Connected", "#2ecc71");
        } else {
            statusLabel->setText("DAQ trigger failed: " + QString::fromStdString(trigErr));
            setLabviewStatus("Disconnected", "#c0392b");
            logMessage(QString("DAQ force trigger failed: %1").arg(QString::fromStdString(trigErr)));
        }
    });

    QObject::connect(captureBtn, &QPushButton::clicked, [&](){
        if (lastFrame.isNull()) {
            statusLabel->setText("No frame to capture");
            return;
        }
        QString baseDir = savePathEdit->text();
        if (baseDir.isEmpty()) baseDir = QCoreApplication::applicationDirPath();
        QDir dir(baseDir);
        dir.mkpath(".");
        QString fname = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss_zzz") + ".tiff";
        QString outPath = dir.filePath(fname);
        if (lastFrame.save(outPath, "TIFF")) {
            statusLabel->setText("Captured: " + fname);
            logLine("Captured frame to " + outPath);
        } else {
            statusLabel->setText("Capture failed");
        }
    });

    auto startSaving = [&](){
        if (saving.load()) {
            statusLabel->setText("Already saving to disk");
            return;
        }
        recording = true;
        {
            QMutexLocker lk(saveMutex.get());
            saveBuffer->clear();
        }
        recordedFrames = 0;
        recordTimer.restart();
        recordStartTime = QDateTime::currentDateTime();
        saveStartBtn->setEnabled(false);
        saveStopBtn->setEnabled(true);
        logLine("Recording started");
        statusLabel->setText("Recording...");
        saveInfoLabel->setText("Elapsed: 0.0 s\nFrames: 0");
        saveInfoTimer.start();
    };

    auto stopSaving = [&](){
        if (!recording.load()) return;
        recording = false;
        saveStartBtn->setEnabled(true);
        saveStopBtn->setEnabled(false);
        saveInfoTimer.stop();

        std::shared_ptr<std::vector<QImage>> frames = std::make_shared<std::vector<QImage>>();
        {
            QMutexLocker lk(saveMutex.get());
            frames->swap(*saveBuffer);
        }
        if (frames->empty()) {
            statusLabel->setText("No frames to save");
            return;
        }

        QString baseDir = savePathEdit->text();
        if (baseDir.isEmpty()) baseDir = QCoreApplication::applicationDirPath();
        QDir dir(baseDir);
        dir.mkpath(".");
        QString sub = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
        QString outDir = dir.filePath(sub);
        dir.mkpath(outDir);

        saving = true;
        statusLabel->setText("Saving to disk...");
        logLine(QString("Saving %1 frames to %2").arg(frames->size()).arg(outDir));
        if (!savingDialog) {
            savingDialog = new QDialog(&window);
            savingDialog->setWindowTitle("Saving...");
            savingDialog->setModal(true);
            auto layout = new QVBoxLayout(savingDialog);
            savingDialogLabel = new QLabel(savingDialog);
            savingProgress = new QProgressBar(savingDialog);
            savingProgress->setMinimum(0);
            layout->addWidget(savingDialogLabel);
            layout->addWidget(savingProgress);
            savingDialog->setLayout(layout);
        }
        int totalFrames = static_cast<int>(frames->size());
        savingDialogLabel->setText(QString("Saving %1 frames...").arg(totalFrames));
        savingProgress->setRange(0, totalFrames);
        savingProgress->setValue(0);
        savingDialog->show();

        FrameMeta metaCopy = lastMeta;
        double expMsCopy = exposureSpin->value();
        QString recordStartStr = recordStartTime.toString("yyyy-MM-dd hh:mm:ss.zzz");

        std::thread([frames, outDir, logLine, statusLabel, savingDialog, savingProgress, totalFrames, metaCopy, expMsCopy, recordStartStr, &saving](){
            int width = std::max(6, static_cast<int>(std::ceil(std::log10(std::max<size_t>(1, frames->size())))));
            for (size_t i = 0; i < frames->size(); ++i) {
                const QImage& im = frames->at(i);
                QString fname = QString("%1.tiff").arg(static_cast<int>(i), width, 10, QChar('0'));
                QString path = outDir + "/" + fname;
                im.save(path, "TIFF");
                if (savingProgress && (i % 100 == 0 || i + 1 == frames->size())) {
                    int v = static_cast<int>(i + 1);
                    QMetaObject::invokeMethod(savingProgress, [savingProgress, v](){
                        savingProgress->setValue(v);
                    }, Qt::QueuedConnection);
                }
            }
            // Write metadata file
            QFile infoFile(outDir + "/capture_info.txt");
            if (infoFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
                QTextStream ts(&infoFile);
                ts << "Start: " << recordStartStr << "\n";
                ts << "Frames: " << frames->size() << "\n";
                ts << "Resolution: " << metaCopy.width << " x " << metaCopy.height << "\n";
                ts << "Binning: " << metaCopy.binning << "\n";
                ts << "Bits: " << metaCopy.bits << "\n";
                ts << "Exposure(ms): " << expMsCopy << "\n";
                ts << "Internal FPS: " << metaCopy.internalFps << "\n";
                ts << "Readout speed: " << metaCopy.readoutSpeed << "\n";
                ts.flush();
                infoFile.close();
            }
            logLine(QString("Saved %1 frames to %2").arg(frames->size()).arg(outDir));
            QMetaObject::invokeMethod(statusLabel, [statusLabel](){
                statusLabel->setText("Save complete");
            }, Qt::QueuedConnection);
            if (savingDialog) {
                QMetaObject::invokeMethod(savingDialog, [savingDialog](){
                    savingDialog->hide();
                }, Qt::QueuedConnection);
            }
            saving = false;
        }).detach();
    };

    QObject::connect(saveStartBtn, &QPushButton::clicked, startSaving);
    QObject::connect(saveStopBtn, &QPushButton::clicked, stopSaving);

    QObject::connect(&saveInfoTimer, &QTimer::timeout, [&](){
        if (!recording.load()) return;
        double elapsed = recordTimer.isValid() ? recordTimer.elapsed() / 1000.0 : 0.0;
        saveInfoLabel->setText(QString("Elapsed: %1 s\nFrames: %2")
            .arg(elapsed,0,'f',1).arg(recordedFrames.load()));
    });

    auto updatePipelineStatus = [&](const PipelineEvent& evt, int bgRemaining, bool pipelineReady){
        QMetaObject::invokeMethod(pipelineStatusLabel, [pipelineStatusLabel, evt, bgRemaining, pipelineReady](){
            if (!pipelineReady) {
                pipelineStatusLabel->setText("Pipeline: not loaded");
                return;
            }
            if (bgRemaining > 0) {
                pipelineStatusLabel->setText(QString("Pipeline: warming (%1 frames)").arg(bgRemaining));
                return;
            }
            if (evt.fired) {
                pipelineStatusLabel->setText(QString("Event: %1 (score %2) area=%3")
                    .arg(QString::fromStdString(evt.label))
                    .arg(evt.score, 0, 'f', 3)
                    .arg(evt.area, 0, 'f', 0));
            } else {
                pipelineStatusLabel->setText("Pipeline: running");
            }
        }, Qt::QueuedConnection);
    };

    auto buildClassText = [&](const QMap<QString, int>& counts)->QString {
        if (counts.isEmpty()) return "Classes:\n(none)";
        QStringList order = {"Empty", "Single", "MoreThanTwo", ">2", "2"};
        QSet<QString> used;
        QString text = "Classes:";
        for (const QString& name : order) {
            if (counts.contains(name)) {
                text += QString("\n%1: %2").arg(name).arg(counts.value(name));
                used.insert(name);
            }
        }
        for (auto it = counts.begin(); it != counts.end(); ++it) {
            if (used.contains(it.key())) continue;
            text += QString("\n%1: %2").arg(it.key()).arg(it.value());
        }
        return text;
    };

    auto makeStatsSnapshot = [&](const StatsTracker& s)->StatsSnapshot {
        StatsSnapshot snap;
        snap.totalEvents = s.totalEvents;
        snap.hitCount = s.hitCount;
        snap.wasteCount = s.wasteCount;
        snap.eventActive = s.eventActive;
        snap.classText = buildClassText(s.classCounts);
        snap.classCounts = s.classCounts;
        snap.lastEventDir = s.lastEventDir;
        snap.lastEventLabel = s.lastEventLabel;
        snap.lastDecisionFrame = s.lastDecisionFrame;
        snap.lastDecisionEventId = s.lastDecisionEventId;
        if (!s.lastEventLabel.isEmpty()) {
            snap.lastText = QString("Last event: %1 (%2)").arg(s.lastEventLabel, s.lastEventDir);
        } else {
            snap.lastText = QString("Last event: --");
        }
        return snap;
    };

    auto getStatsSnapshot = [&]()->StatsSnapshot {
        QMutexLocker lock(&statsMutex);
        return makeStatsSnapshot(stats);
    };

    auto buildStatsFigures = [&](const StatsSnapshot& snap){
        int hit = snap.hitCount;
        int waste = snap.wasteCount;
        QImage hitWaste = renderPieChart("Hit vs Waste",
                                         {"Hit", "Waste"},
                                         {static_cast<double>(hit), static_cast<double>(waste)},
                                         {QColor(46, 204, 113), QColor(192, 57, 43)});

        int empty = 0;
        int single = 0;
        int more = 0;
        for (auto it = snap.classCounts.begin(); it != snap.classCounts.end(); ++it) {
            QString label = it.key().trimmed().toLower();
            int count = it.value();
            if (label.contains("empty")) {
                empty += count;
            } else if (label.contains("single")) {
                single += count;
            } else if (label.contains("more") || label.contains(">") || label == "2") {
                more += count;
            } else if (!label.isEmpty() && label != "(unclassified)") {
                more += count;
            }
        }
        QImage classImg = renderPieChart("Class Distribution",
                                         {"0", "1", ">2"},
                                         {static_cast<double>(empty),
                                          static_cast<double>(single),
                                          static_cast<double>(more)},
                                         {QColor(52, 152, 219), QColor(241, 196, 15), QColor(155, 89, 182)});
        return std::pair<QImage, QImage>(hitWaste, classImg);
    };

    auto saveStatsFigures = [&](const QString& outDir, const QString& prefix, const StatsSnapshot& snap)->bool {
        if (outDir.isEmpty()) return false;
        auto figures = buildStatsFigures(snap);
        QDir out(outDir);
        out.mkpath(".");
        QString hitPath = out.filePath(prefix + "_hit_waste.png");
        QString clsPath = out.filePath(prefix + "_class_dist.png");
        bool ok1 = !figures.first.isNull() && figures.first.save(hitPath);
        bool ok2 = !figures.second.isNull() && figures.second.save(clsPath);
        return ok1 && ok2;
    };

    auto updateStatsFigureWindow = [&](const StatsSnapshot& snap){
        if (!statsFigureWindow) return;
        auto figures = buildStatsFigures(snap);
        statsFigureWindow->setImages(figures.first, figures.second);
    };

    auto applyStatsSnapshot = [&](const StatsSnapshot& snap){
        QMetaObject::invokeMethod(statsEventsLabel, [=](){
            statsEventsLabel->setText(QString("Events: %1  Active: %2")
                .arg(snap.totalEvents)
                .arg(snap.eventActive ? "Yes" : "No"));
            statsHitLabel->setText(QString("Hits: %1\nWastes: %2")
                .arg(snap.hitCount)
                .arg(snap.wasteCount));
            statsClassLabel->setText(snap.classText);
            statsLastLabel->setText(snap.lastText);
        }, Qt::QueuedConnection);
    };

    auto resetStats = [&](){
        StatsSnapshot snap;
        {
            QMutexLocker lock(&statsMutex);
            stats = StatsTracker{};
            snap = makeStatsSnapshot(stats);
        }
        applyStatsSnapshot(snap);
    };

    auto showStatsFigures = [&](){
        StatsSnapshot snap = getStatsSnapshot();
        auto figures = buildStatsFigures(snap);
        if (!statsFigureWindow) {
            statsFigureWindow = new StatsFigureWindow(&window);
            statsFigureWindow->setAttribute(Qt::WA_DeleteOnClose);
            QObject::connect(statsFigureWindow, &QObject::destroyed, [&](){ statsFigureWindow = nullptr; });
            QObject::connect(statsFigureWindow->saveButton(), &QPushButton::clicked, [&](){
                QString outDir = outputEdit->text().trimmed();
                if (outDir.isEmpty()) outDir = QCoreApplication::applicationDirPath();
                QString dir = QFileDialog::getExistingDirectory(statsFigureWindow, "Select output directory", outDir);
                if (dir.isEmpty()) return;
                QString prefix = QDateTime::currentDateTime().toString("stats_yyyyMMdd_hhmmss");
                if (statsFigureWindow->saveImages(dir, prefix)) {
                    statusLabel->setText("Saved stats figures to " + dir);
                    logLine("Saved stats figures to " + dir);
                } else {
                    statusLabel->setText("Failed to save stats figures.");
                }
            });
        }
        statsFigureWindow->setImages(figures.first, figures.second);
        statsFigureWindow->show();
        statsFigureWindow->raise();
        statsFigureWindow->activateWindow();
    };

    auto endEventLocked = [&](StatsTracker& s, int decisionFrame){
        if (!s.eventActive) return;
        QString dir = "Unknown";
        if (s.hasCentroid) {
            double dy = s.cumulativeDy;
            double threshold = 2.0;
            if (s.frameHeight > 0) {
                threshold = std::max(threshold, s.frameHeight * 0.02);
            }
            bool movedUp = dy < -threshold;
            bool movedDown = dy > threshold;
            bool hasFrame = (s.frameHeight > 0);
            double mid = hasFrame ? s.frameHeight * 0.5 : 0.0;

            if (movedUp && (!hasFrame || s.lastY < mid)) {
                s.wasteCount++;
                dir = "Waste";
            } else if (movedDown && (!hasFrame || s.lastY >= mid)) {
                s.hitCount++;
                dir = "Hit";
            } else if (hasFrame) {
                if (s.lastY < mid) {
                    s.wasteCount++;
                    dir = "Waste";
                } else {
                    s.hitCount++;
                    dir = "Hit";
                }
            } else if (dy < 0.0) {
                s.wasteCount++;
                dir = "Waste";
            } else {
                s.hitCount++;
                dir = "Hit";
            }
        }
        s.lastEventDir = dir;
        s.lastEventLabel = s.currentLabel;
        s.lastDecisionFrame = decisionFrame;
        s.lastDecisionEventId = s.currentEventId;
        s.eventActive = false;
        s.hasCentroid = false;
        s.missCount = 0;
        s.currentLabel.clear();
        s.cumulativeDy = 0.0;
    };

    auto updateStatsFromEvent = [&](const PipelineEvent& evt, bool processed){
        if (!processed) return;
        StatsSnapshot snap;
        {
            QMutexLocker lock(&statsMutex);
            if (evt.fired) {
                if (stats.eventActive) {
                    endEventLocked(stats, evt.frameNumber);
                }
                stats.eventActive = true;
                stats.missCount = 0;
                stats.currentEventId++;
                stats.startCentroid = evt.centroid;
                stats.lastCentroid = evt.centroid;
                stats.hasCentroid = true;
                stats.cumulativeDy = 0.0;
                stats.lastY = evt.centroid.y;
                stats.minY = evt.centroid.y;
                stats.maxY = evt.centroid.y;
                if (evt.frameHeight > 0) stats.frameHeight = evt.frameHeight;
                stats.totalEvents++;
                QString label = QString::fromStdString(evt.label);
                if (label.isEmpty()) label = "(unclassified)";
                stats.currentLabel = label;
                stats.classCounts[label] = stats.classCounts.value(label) + 1;
            } else if (evt.detected) {
                if (!stats.eventActive) {
                    stats.eventActive = true;
                    stats.missCount = 0;
                    stats.currentEventId++;
                    stats.startCentroid = evt.centroid;
                    stats.lastCentroid = evt.centroid;
                    stats.hasCentroid = true;
                    stats.cumulativeDy = 0.0;
                    stats.lastY = evt.centroid.y;
                    stats.minY = evt.centroid.y;
                    stats.maxY = evt.centroid.y;
                    if (evt.frameHeight > 0) stats.frameHeight = evt.frameHeight;
                    stats.totalEvents++;
                    QString label = QString::fromStdString(evt.label);
                    if (label.isEmpty()) label = "(unclassified)";
                    stats.currentLabel = label;
                    stats.classCounts[label] = stats.classCounts.value(label) + 1;
                } else {
                    stats.cumulativeDy += static_cast<double>(evt.centroid.y - stats.lastCentroid.y);
                    stats.lastCentroid = evt.centroid;
                    stats.hasCentroid = true;
                    stats.lastY = evt.centroid.y;
                    stats.minY = std::min(stats.minY, static_cast<double>(evt.centroid.y));
                    stats.maxY = std::max(stats.maxY, static_cast<double>(evt.centroid.y));
                    if (evt.frameHeight > 0) stats.frameHeight = evt.frameHeight;
                    stats.missCount = 0;
                }
            } else if (stats.eventActive) {
                stats.missCount++;
                if (stats.missCount >= pipelineDetectCfg.resetFrames) {
                    endEventLocked(stats, evt.frameNumber);
                }
            }
            snap = makeStatsSnapshot(stats);
        }
        applyStatsSnapshot(snap);
    };

    auto processPipelineFrame = [&](const QImage& img,
                                    PipelineEvent& evt,
                                    int& bgRemaining,
                                    bool& pipelineReady,
                                    double* procMsOut)->bool {
        bgRemaining = 0;
        pipelineReady = false;
        if (!pipelineEnabled.load() || img.isNull()) return false;

        cv::Mat gray(img.height(), img.width(), CV_8UC1,
                     const_cast<uchar*>(img.bits()), img.bytesPerLine());
        cv::Mat grayCopy = gray.clone();

        auto t0 = std::chrono::steady_clock::now();
        bool processed = false;
        {
            QMutexLocker lock(&pipelineMutex);
            pipelineReady = pipeline.isReady();
            if (pipelineReady) {
                processed = pipeline.processFrame(grayCopy, evt);
                bgRemaining = pipeline.backgroundFramesRemaining();
            }
        }
        auto t1 = std::chrono::steady_clock::now();
        if (procMsOut) {
            *procMsOut = std::chrono::duration<double, std::milli>(t1 - t0).count();
        }

        updatePipelineStatus(evt, bgRemaining, pipelineReady);
        updateStatsFromEvent(evt, processed);
        return processed;
    };

    auto writeLiveLogCsv = [&](const QString& outDir,
                               const QString& prefix,
                               const std::vector<LiveLogRecord>& records)->QString {
        if (outDir.isEmpty()) return QString();
        QDir out(outDir);
        out.mkpath(".");
        QString path = out.filePath(prefix + "_live_log.csv");
        QFile logFile(path);
        if (!logFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            return QString();
        }
        QTextStream ts(&logFile);
        ts << "wall_time,frame_index,delivered,dropped,fps,cam_fps,proc_ms,processed,pipeline_enabled,pipeline_ready,bg_remaining,skip_reason,"
              "detected,fired,area,bbox_x,bbox_y,bbox_w,bbox_h,crop_x,crop_y,crop_w,crop_h,crop_path,label,score,triggered,trigger_ok,"
              "event_dir,decision_frame,decision_event_id,hit_count,waste_count\n";
        for (const auto& rec : records) {
            ts << csvQuote(rec.wallTime) << ","
               << rec.frameIndex << ","
               << rec.delivered << ","
               << rec.dropped << ","
               << QString::number(rec.fps, 'f', 2) << ","
               << QString::number(rec.camFps, 'f', 2) << ","
               << QString::number(rec.procMs, 'f', 3) << ","
               << (rec.processed ? "1" : "0") << ","
               << (rec.pipelineEnabled ? "1" : "0") << ","
               << (rec.pipelineReady ? "1" : "0") << ","
               << rec.bgRemaining << ","
               << csvQuote(rec.skipReason) << ","
               << (rec.detected ? "1" : "0") << ","
               << (rec.fired ? "1" : "0") << ","
               << QString::number(rec.area, 'f', 1) << ","
               << rec.bboxX << "," << rec.bboxY << "," << rec.bboxW << "," << rec.bboxH << ","
               << rec.cropX << "," << rec.cropY << "," << rec.cropW << "," << rec.cropH << ","
               << csvQuote(rec.cropPath) << ","
               << csvQuote(rec.label) << ","
               << QString::number(rec.score, 'f', 4) << ","
               << (rec.triggered ? "1" : "0") << ","
               << (rec.triggerOk ? "1" : "0") << ","
               << csvQuote(rec.eventDir) << ","
               << rec.decisionFrame << ","
               << rec.decisionEventId << ","
               << rec.hitCount << ","
               << rec.wasteCount
               << "\n";
        }
        ts.flush();
        logFile.close();
        return path;
    };

    startLiveLogging = [&](){
        QMutexLocker lock(&liveLogMutex);
        liveLog.clear();
        liveLogStart = QDateTime::currentDateTime();
        liveLogging.store(true);
    };

    stopLiveLogging = [&](){
        if (!liveLogging.exchange(false)) return;
        std::vector<LiveLogRecord> records;
        {
            QMutexLocker lock(&liveLogMutex);
            records = liveLog;
        }
        StatsSnapshot snap = getStatsSnapshot();
        QString outDir = outputEdit->text().trimmed();
        if (outDir.isEmpty()) outDir = QCoreApplication::applicationDirPath();
        QString timestamp = liveLogStart.isValid()
            ? liveLogStart.toString("yyyyMMdd_hhmmss")
            : QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
        QString prefix = "live_" + timestamp;
        QString logPath = writeLiveLogCsv(outDir, prefix, records);
        saveStatsFigures(outDir, prefix, snap);
        updateStatsFigureWindow(snap);
        if (!logPath.isEmpty()) {
            statusLabel->setText("Pipeline stopped. Log: " + logPath);
            logLine("Saved live pipeline log to " + logPath);
        } else {
            statusLabel->setText("Pipeline stopped. Failed to write log.");
        }
    };

    auto formatBytes = [](size_t bytes)->QString {
        const double mb = static_cast<double>(bytes) / (1024.0 * 1024.0);
        return QString("%1 MB").arg(mb, 0, 'f', 1);
    };

    auto setSequenceUiRunning = [&](bool running){
        seqStartBtn->setEnabled(!running);
        seqStopBtn->setEnabled(running);
        seqLoadBtn->setEnabled(!running);
        pipelineWidget->setEnabled(!running);
        labviewWidget->setEnabled(!running);
        detectWidget->setEnabled(!running);
        pipelineStartBtn->setEnabled(!running && !pipelineEnabled.load());
        pipelineStopBtn->setEnabled(!running && pipelineEnabled.load());
        if (!viewerOnly) {
            startBtn->setEnabled(!running);
            stopBtn->setEnabled(!running);
            reconnectBtn->setEnabled(!running);
            applyBtn->setEnabled(!running);
        }
    };

    auto updateSequenceStatus = [&](const QString& text){
        QMetaObject::invokeMethod(seqStatusLabel, [seqStatusLabel, text](){
            seqStatusLabel->setText(text);
        }, Qt::QueuedConnection);
    };

    auto collectSequenceFiles = [&](const QString& dirPath)->QStringList {
        QDir dir(dirPath);
        QStringList filters;
        filters << "*.tif" << "*.tiff" << "*.TIF" << "*.TIFF"
                << "*.png" << "*.PNG" << "*.jpg" << "*.JPG"
                << "*.jpeg" << "*.JPEG" << "*.bmp" << "*.BMP";
        return dir.entryList(filters, QDir::Files, QDir::Name);
    };

    QObject::connect(seqBrowseBtn, &QPushButton::clicked, [&](){
        QString dir = QFileDialog::getExistingDirectory(&window, "Select sequence folder", seqFolderEdit->text());
        if (!dir.isEmpty()) seqFolderEdit->setText(dir);
    });

    QObject::connect(seqLoadBtn, &QPushButton::clicked, [&](){
        if (sequenceRunning.load()) return;
        sequenceStop.store(false);
        QString dirPath = seqFolderEdit->text().trimmed();
        QDir dir(dirPath);
        if (!dir.exists()) {
            seqStatusLabel->setText("Sequence folder not found.");
            return;
        }
        QStringList files = collectSequenceFiles(dirPath);
        if (files.isEmpty()) {
            seqStatusLabel->setText("No images found in folder.");
            return;
        }

        seqLoadBtn->setEnabled(false);
        seqStartBtn->setEnabled(false);
        updateSequenceStatus(QString("Loading %1 frames...").arg(files.size()));

        std::thread([&, dirPath, files](){
            auto frames = std::make_shared<std::vector<SequenceFrame>>();
            frames->reserve(files.size());
            size_t totalBytes = 0;
            int loaded = 0;
            for (const QString& rel : files) {
                if (sequenceStop.load()) break;
                QString absPath = QDir(dirPath).absoluteFilePath(rel);
                QImageReader reader(absPath);
                reader.setAutoTransform(true);
                QImage img = reader.read();
                if (img.isNull()) {
                    continue;
                }
                if (img.format() != QImage::Format_Grayscale8) {
                    img = img.convertToFormat(QImage::Format_Grayscale8);
                }
                totalBytes += static_cast<size_t>(img.sizeInBytes());
                frames->push_back({img, absPath});
                loaded++;
                if (loaded % 100 == 0) {
                    updateSequenceStatus(QString("Loaded %1 / %2 frames...").arg(loaded).arg(files.size()));
                }
            }
            {
                QMutexLocker lock(&sequenceMutex);
                sequenceFrames = frames;
            }
            QString status = QString("Loaded %1 frames (%2).")
                .arg(frames->size())
                .arg(formatBytes(totalBytes));
            updateSequenceStatus(status);
            QMetaObject::invokeMethod(seqLoadBtn, [seqLoadBtn, seqStartBtn](){
                seqLoadBtn->setEnabled(true);
                seqStartBtn->setEnabled(true);
            }, Qt::QueuedConnection);
        }).detach();
    });

    auto stopSequenceTest = [&](){
        sequenceStop.store(true);
        if (sequenceThread.joinable()) {
            sequenceThread.join();
        }
        if (sequenceRunning.load()) {
            sequenceRunning.store(false);
            setSequenceUiRunning(false);
            seqStatusLabel->setText("Sequence stopped.");
            statusLabel->setText("Sequence test stopped.");
        }
    };

    QObject::connect(seqStopBtn, &QPushButton::clicked, stopSequenceTest);

    QObject::connect(statsResetBtn, &QPushButton::clicked, resetStats);
    QObject::connect(statsShowBtn, &QPushButton::clicked, showStatsFigures);

    QObject::connect(seqStartBtn, &QPushButton::clicked, [&](){
        if (sequenceRunning.load()) return;
        std::shared_ptr<std::vector<SequenceFrame>> frames;
        {
            QMutexLocker lock(&sequenceMutex);
            frames = sequenceFrames;
        }
        if (!frames || frames->empty()) {
            seqStatusLabel->setText("No sequence loaded.");
            return;
        }
        double fps = seqFpsSpin->value();
        if (fps <= 0.0) {
            seqStatusLabel->setText("FPS must be greater than 0.");
            return;
        }

        if (liveLogging.load()) {
            stopLiveLogging();
        }
        QString runDir = buildRunOutputDir("sequence");
        if (!runDir.isEmpty()) {
            outputEdit->setText(runDir);
        }
        sequencePrevPipelineChecked = pipelineEnableCheck->isChecked();
        sequenceStarting.store(true);
        loadPipeline(true);
        bool pipelineReady = false;
        {
            QMutexLocker lock(&pipelineMutex);
            pipelineReady = pipeline.isReady();
        }
        sequenceStarting.store(false);
        if (!pipelineReady) {
            seqStatusLabel->setText("Pipeline not ready. Fix settings and load pipeline.");
            return;
        }
        if (sequenceThread.joinable()) {
            sequenceThread.join();
        }
        sequenceStop.store(false);
        sequenceRunning.store(true);
        setSequenceUiRunning(true);

        if (!viewerOnly) {
            grabber.stopGrabbing();
            controller.stop();
        }
        statusLabel->setText("Sequence test running.");
        if (pipeline.isReady()) {
            QMutexLocker lock(&pipelineMutex);
            pipeline.reset();
            pipelineStatusLabel->setText("Pipeline: warming (sequence start)");
        }

        QString outDir = outputEdit->text().trimmed();
        if (outDir.isEmpty()) {
            outDir = QCoreApplication::applicationDirPath();
        }
        QString onnxResolved = resolveAppRelative(onnxEdit->text());
        QString metaResolved = resolveAppRelative(metaEdit->text());
        QString targetLabel = targetLabelEdit->text().trimmed();
        QString seqFolder = seqFolderEdit->text().trimmed();
        QString daqChannel = daqChannelEdit->text().trimmed();
        double daqAmp = amplitudeSpin->value();
        double daqFreqHz = freqSpin->value() * 1000.0;
        double daqDuration = durationSpin->value();
        double daqDelay = delaySpin->value();
        QDir out(outDir);
        out.mkpath(".");
        QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
        QString logPath = out.filePath("sequence_test_log_" + timestamp + ".csv");
        seqLogLabel->setText("Log: " + logPath);
        seqStatusLabel->setText(QString("Running %1 frames at %2 fps...")
            .arg(frames->size()).arg(fps,0,'f',2));

        int displayEvery = std::max(1, displayEverySpin->value());

        int frameSkip = frameSkipSpin->value();
        int bgFrames = bgFramesSpin->value();
        int bgUpdate = bgUpdateSpin->value();
        int resetFrames = resetFramesSpin->value();
        double minArea = minAreaSpin->value();
        double minAreaFrac = minAreaFracSpin->value();
        double maxAreaFrac = maxAreaFracSpin->value();
        int minBbox = minBboxSpin->value();
        int margin = marginSpin->value();
        int diffThresh = diffThreshSpin->value();
        int blurRadius = blurRadiusSpin->value();
        int morphRadius = morphRadiusSpin->value();
        double scale = scaleSpin->value();
        int gapFireShift = gapFireSpin->value();

        sequenceThread = std::thread([&, frames, fps, displayEvery, logPath, outDir, onnxResolved, metaResolved, targetLabel, seqFolder,
                                      daqChannel, daqAmp, daqFreqHz, daqDuration, daqDelay,
                                      frameSkip, bgFrames, bgUpdate, resetFrames, minArea, minAreaFrac, maxAreaFrac,
                                      minBbox, margin, diffThresh, blurRadius, morphRadius, scale, gapFireShift](){
            QFile logFile(logPath);
            if (!logFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
                updateSequenceStatus("Failed to open sequence log.");
                sequenceRunning.store(false);
                QMetaObject::invokeMethod(&window, [&, logPath](){
                    setSequenceUiRunning(false);
                    statusLabel->setText("Sequence test failed (log open).");
                    seqLogLabel->setText("Log: " + logPath);
                }, Qt::QueuedConnection);
                return;
            }
            QTextStream ts(&logFile);
            ts << "# sequence_folder=" << seqFolder << "\n";
            ts << "# fps=" << QString::number(fps, 'f', 2) << "\n";
            ts << "# frames=" << frames->size() << "\n";
            ts << "# display_every=" << displayEvery << "\n";
            ts << "# output_dir=" << outDir << "\n";
            ts << "# onnx=" << onnxResolved << "\n";
            ts << "# metadata=" << metaResolved << "\n";
            ts << "# target_label=" << targetLabel << "\n";
            ts << "# pipeline_enabled_before=" << (sequencePrevPipelineChecked ? 1 : 0) << "\n";
            ts << "# pipeline_forced=" << (sequencePrevPipelineChecked ? 0 : 1) << "\n";
            ts << "# frame_skip=" << frameSkip << "\n";
            ts << "# detect_bg_frames=" << bgFrames << "\n";
            ts << "# detect_bg_update=" << bgUpdate << "\n";
            ts << "# detect_reset_frames=" << resetFrames << "\n";
            ts << "# detect_min_area=" << QString::number(minArea, 'f', 3) << "\n";
            ts << "# detect_min_area_frac=" << QString::number(minAreaFrac, 'f', 6) << "\n";
            ts << "# detect_max_area_frac=" << QString::number(maxAreaFrac, 'f', 6) << "\n";
            ts << "# detect_min_bbox=" << minBbox << "\n";
            ts << "# detect_margin=" << margin << "\n";
            ts << "# detect_diff_thresh=" << diffThresh << "\n";
            ts << "# detect_blur_radius=" << blurRadius << "\n";
            ts << "# detect_morph_radius=" << morphRadius << "\n";
            ts << "# detect_scale=" << QString::number(scale, 'f', 3) << "\n";
            ts << "# detect_gap_fire_shift=" << gapFireShift << "\n";
            ts << "# daq_channel=" << daqChannel << "\n";
            ts << "# daq_range_min=-10\n";
            ts << "# daq_range_max=10\n";
            ts << "# daq_amplitude_v=" << QString::number(daqAmp, 'f', 3) << "\n";
            ts << "# daq_frequency_hz=" << QString::number(daqFreqHz, 'f', 1) << "\n";
            ts << "# daq_duration_ms=" << QString::number(daqDuration, 'f', 3) << "\n";
            ts << "# daq_delay_ms=" << QString::number(daqDelay, 'f', 3) << "\n";
            ts << "index,filename,scheduled_ms,actual_ms,jitter_ms,wall_time,proc_ms,processed,pipeline_enabled,pipeline_ready,bg_remaining,skip_reason,"
                  "detected,fired,area,bbox_x,bbox_y,bbox_w,bbox_h,crop_x,crop_y,crop_w,crop_h,crop_path,label,score,triggered,trigger_ok,frame_number,"
                  "event_dir,decision_frame,decision_event_id\n";
            ts.flush();

            auto csvQuote = [](const QString& s)->QString {
                QString out = s;
                out.replace("\"", "\"\"");
                return "\"" + out + "\"";
            };

            using clock = std::chrono::steady_clock;
            auto start = clock::now();
            std::chrono::duration<double> period(1.0 / fps);

            for (size_t i = 0; i < frames->size(); ++i) {
                if (sequenceStop.load()) break;
                auto target = start + period * static_cast<double>(i);
                while (!sequenceStop.load()) {
                    auto now = clock::now();
                    if (now >= target) break;
                    auto remaining = target - now;
                    if (remaining > std::chrono::milliseconds(2)) {
                        std::this_thread::sleep_for(remaining - std::chrono::milliseconds(1));
                    } else {
                        std::this_thread::yield();
                    }
                }
                if (sequenceStop.load()) break;

                const SequenceFrame& frame = frames->at(i);
                double scheduledMs = std::chrono::duration<double, std::milli>(period * static_cast<double>(i)).count();
                double actualMs = std::chrono::duration<double, std::milli>(clock::now() - start).count();
                double jitterMs = actualMs - scheduledMs;
                QString wallTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");

                FrameMeta meta;
                meta.width = frame.image.width();
                meta.height = frame.image.height();
                meta.bits = 8;
                meta.binning = 1.0;
                meta.frameIndex = static_cast<qint64>(i);
                meta.delivered = static_cast<qint64>(i + 1);
                meta.dropped = 0;
                meta.internalFps = fps;

                PipelineEvent evt;
                int bgRemaining = 0;
                bool pipelineReady = false;
                double procMs = 0.0;
                bool processed = processPipelineFrame(frame.image, evt, bgRemaining, pipelineReady, &procMs);
                bool enabledNow = pipelineEnabled.load();
                QString skipReason;
                if (!enabledNow) {
                    skipReason = "pipeline_disabled";
                } else if (!pipelineReady) {
                    skipReason = "pipeline_not_ready";
                } else if (!processed) {
                    skipReason = "frame_skipped";
                }

                if (displayEvery > 0 && (static_cast<int>(i) % displayEvery == 0)) {
                    QImage imgCopy = frame.image;
                    QMetaObject::invokeMethod(&window, [&, imgCopy, meta, i, fps, frames](){
                        imageView->setImage(imgCopy);
                        lastFrame = imgCopy;
                        lastMeta = meta;
                        statsLabel->setText(QString("Source: Sequence\nResolution: %1 x %2\nBits: %3\nFPS: %4\nFrame: %5 / %6")
                            .arg(meta.width).arg(meta.height).arg(meta.bits)
                            .arg(fps,0,'f',2).arg(i + 1).arg(frames->size()));
                    }, Qt::QueuedConnection);
                }

                QString cropPath = QString::fromStdString(evt.cropPath);
                QString label = QString::fromStdString(evt.label);
                StatsSnapshot snap = getStatsSnapshot();
                ts << i << ","
                   << csvQuote(QFileInfo(frame.path).fileName()) << ","
                   << QString::number(scheduledMs,'f',3) << ","
                   << QString::number(actualMs,'f',3) << ","
                   << QString::number(jitterMs,'f',3) << ","
                   << csvQuote(wallTime) << ","
                   << QString::number(procMs,'f',3) << ","
                   << (processed ? "1" : "0") << ","
                   << (enabledNow ? "1" : "0") << ","
                   << (pipelineReady ? "1" : "0") << ","
                   << bgRemaining << ","
                   << csvQuote(skipReason) << ","
                   << (evt.detected ? "1" : "0") << ","
                   << (evt.fired ? "1" : "0") << ","
                   << QString::number(evt.area,'f',1) << ","
                   << evt.bbox.x << "," << evt.bbox.y << "," << evt.bbox.width << "," << evt.bbox.height << ","
                   << evt.cropRect.x << "," << evt.cropRect.y << "," << evt.cropRect.width << "," << evt.cropRect.height << ","
                   << csvQuote(cropPath) << ","
                   << csvQuote(label) << ","
                   << QString::number(evt.score,'f',4) << ","
                   << (evt.triggered ? "1" : "0") << ","
                   << (evt.triggerOk ? "1" : "0") << ","
                   << evt.frameNumber << ","
                   << csvQuote(snap.lastEventDir) << ","
                   << snap.lastDecisionFrame << ","
                   << snap.lastDecisionEventId
                   << "\n";
                if (i % 50 == 0) {
                    ts.flush();
                }
            }

            ts.flush();
            logFile.close();

            sequenceRunning.store(false);
            QMetaObject::invokeMethod(&window, [&, logPath](){
                setSequenceUiRunning(false);
                seqStatusLabel->setText("Sequence finished.");
                statusLabel->setText("Sequence test finished.");
                seqLogLabel->setText("Log: " + logPath);
            }, Qt::QueuedConnection);
        });
    });

    grabber.setRecordHook([saveMutex, saveBuffer, &recording, &recordedFrames,
                           &pipelineEnabled, &sequenceRunning, &processPipelineFrame,
                           &liveLogging, &liveLogMutex, &liveLog, &getStatsSnapshot](const QImage& img, const FrameMeta& meta, double fps){
        if (recording.load()) {
            QMutexLocker lk(saveMutex.get());
            saveBuffer->push_back(img.copy());
            recordedFrames++;
        }

        if (sequenceRunning.load()) return;

        PipelineEvent evt;
        int bgRemaining = 0;
        bool pipelineReady = false;
        double procMs = 0.0;
        bool processed = processPipelineFrame(img, evt, bgRemaining, pipelineReady, &procMs);

        if (liveLogging.load()) {
            bool enabledNow = pipelineEnabled.load();
            QString skipReason;
            if (!enabledNow) {
                skipReason = "pipeline_disabled";
            } else if (!pipelineReady) {
                skipReason = "pipeline_not_ready";
            } else if (!processed) {
                skipReason = "frame_skipped";
            }

            LiveLogRecord rec;
            rec.wallTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");
            rec.frameIndex = meta.frameIndex;
            rec.delivered = meta.delivered;
            rec.dropped = meta.dropped;
            rec.fps = fps;
            rec.camFps = meta.internalFps;
            rec.procMs = procMs;
            rec.processed = processed;
            rec.pipelineEnabled = enabledNow;
            rec.pipelineReady = pipelineReady;
            rec.skipReason = skipReason;
            rec.bgRemaining = bgRemaining;
            rec.detected = evt.detected;
            rec.fired = evt.fired;
            rec.area = evt.area;
            rec.bboxX = evt.bbox.x;
            rec.bboxY = evt.bbox.y;
            rec.bboxW = evt.bbox.width;
            rec.bboxH = evt.bbox.height;
            rec.cropX = evt.cropRect.x;
            rec.cropY = evt.cropRect.y;
            rec.cropW = evt.cropRect.width;
            rec.cropH = evt.cropRect.height;
            rec.cropPath = QString::fromStdString(evt.cropPath);
            rec.label = QString::fromStdString(evt.label);
            rec.score = evt.score;
            rec.triggered = evt.triggered;
            rec.triggerOk = evt.triggerOk;
            StatsSnapshot snap = getStatsSnapshot();
            rec.eventDir = snap.lastEventDir;
            rec.decisionFrame = snap.lastDecisionFrame;
            rec.decisionEventId = snap.lastDecisionEventId;
            rec.hitCount = snap.hitCount;
            rec.wasteCount = snap.wasteCount;
            QMutexLocker lk(&liveLogMutex);
            liveLog.push_back(rec);
        }
    });

    QObject::connect(&grabber, &FrameGrabber::frameReady, &window,
                     [&, imageView, statsLabel, logCheck](const QImage& img, FrameMeta meta, double fps){
        if (!img.isNull()) {
            imageView->setImage(img);
            lastFrame = img;
        }
        lastMeta = meta;
        statsLabel->setText(QString("Resolution: %1 x %2\nBinning: %3\nBits: %4\nFPS: %5 (Cam: %6)\nFrame: %7\nDelivered: %8 Dropped: %9\nReadout: %10")
            .arg(meta.width).arg(meta.height).arg(meta.binning,0,'f',1).arg(meta.bits)
            .arg(fps,0,'f',1).arg(meta.internalFps,0,'f',1).arg(meta.frameIndex).arg(meta.delivered).arg(meta.dropped).arg(meta.readoutSpeed,0,'f',0));
        if (logCheck->isChecked() && (meta.frameIndex % 100 == 0)) {
            logLine(QString("Frame=%1 FPS=%2 camfps=%3 delivered=%4 dropped=%5")
                .arg(meta.frameIndex).arg(fps,0,'f',1).arg(meta.internalFps,0,'f',1).arg(meta.delivered).arg(meta.dropped));
        }
    }, Qt::QueuedConnection);

    QObject::connect(&app, &QApplication::aboutToQuit, [&](){
        stopSequenceTest();
        stopLiveLogging();
        grabber.stopGrabbing();
        controller.stop();
        controller.cleanup();
        logMessage("Exiting application");
    });

    window.show();
    doInit();
    QTimer::singleShot(0, [&](){
        loadPipeline(false);
    });
    int rc = 0;
    try {
        rc = app.exec();
    } catch (const std::exception& e) {
        logMessage(QString("Fatal exception: %1").arg(e.what()));
        rc = 1;
    } catch (...) {
        logMessage("Fatal unknown exception");
        rc = 1;
    }
    logMessage(QString("Event loop exited with code %1").arg(rc));
    return rc;
}
