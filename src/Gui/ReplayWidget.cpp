#include <Gui/ReplayWidget.h>
#include <Gui/Theme.h>

#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QVBoxLayout>

#include <algorithm>
#include <functional>

namespace {
    // Playback position poll period
    constexpr int TICK_MS = 10;

    QString formatTime(uint64_t micros)
    {
        const uint64_t tenths = micros / 100000;
        return QString("%1:%2.%3")
            .arg(tenths / 600, 2, 10, QChar('0'))
            .arg((tenths / 10) % 60, 2, 10, QChar('0'))
            .arg(tenths % 10);
    }
}

ReplayWidget::ReplayWidget(QWidget* parent)
    : QFrame(parent)
{
    setupUI();

    connect(openButton, &QPushButton::clicked, this, [this]() { chooseRecording(rec.folder()); });
    connect(backButton, &QPushButton::clicked, this, [this]() { step(-1); });
    connect(playButton, &QPushButton::clicked, this, [this]() { togglePlay(); });
    connect(forwardButton, &QPushButton::clicked, this, [this]() { step(1); });
    connect(slider, &QSlider::valueChanged, this, [this](int value) { seek(static_cast<size_t>(value)); });
    connect(speedBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
        setSpeed(speedBox->itemData(index).toDouble());
    });
    connect(liveButton, &QPushButton::clicked, this, [this]() {
        pause();
        emit liveRequested();
    });
    connect(&tickTimer, &QTimer::timeout, this, [this]() { onTick(); });

    // Keyboard: space play/pause, arrows step; only active while the replay is shown
    auto addShortcut = [this](const QKeySequence& key, std::function<void()> action) {
        auto* shortcut = new QShortcut(key, this);
        shortcut->setEnabled(false);
        connect(shortcut, &QShortcut::activated, this, action);
        shortcuts.push_back(shortcut);
    };
    addShortcut(Qt::Key_Space, [this]() { togglePlay(); });
    addShortcut(Qt::Key_Left, [this]() { step(-1); });
    addShortcut(Qt::Key_Right, [this]() { step(1); });

    updateControls();
}

ReplayWidget::~ReplayWidget()
{
    if (loading.valid()) {
        loading.wait();
    }
}

void ReplayWidget::setupUI()
{
    setObjectName("card");

    titleLabel = new QLabel("REPLAY", this);
    titleLabel->setObjectName("cardTitle");

    statusLabel = new QLabel(this);
    QFont mono("monospace");
    mono.setStyleHint(QFont::Monospace);
    statusLabel->setFont(mono);
    statusLabel->setObjectName("muted");

    openButton = new QPushButton("Open…", this);
    openButton->setToolTip("Open another recording folder");

    backButton = new QPushButton("◀", this);
    backButton->setToolTip("Previous row (←)");

    playButton = new QPushButton(this);
    playButton->setObjectName("playButton");
    playButton->setMinimumWidth(80);

    forwardButton = new QPushButton("▶", this);
    forwardButton->setToolTip("Next row (→)");

    speedBox = new QComboBox(this);
    speedBox->setToolTip("Playback speed");
    for (double s : {0.25, 0.5, 1.0, 2.0, 4.0}) {
        speedBox->addItem(QString("%1×").arg(s), s);
    }
    speedBox->setCurrentIndex(2);

    slider = new QSlider(Qt::Horizontal, this);
    slider->setToolTip("Drag to seek");
    slider->setRange(0, 0);

    liveButton = new QPushButton("Live", this);
    liveButton->setToolTip("Stop the replay and show the sensors again");

    for (QWidget* w : std::initializer_list<QWidget*>{openButton, backButton, playButton, forwardButton, speedBox, slider, liveButton}) {
        w->setCursor(Qt::PointingHandCursor);
        w->setFocusPolicy(Qt::NoFocus);     // Keep the keyboard for the shortcuts
    }

    auto* titleRow = new QHBoxLayout();
    titleRow->setSpacing(6);
    titleRow->addWidget(titleLabel);
    titleRow->addStretch();
    titleRow->addWidget(statusLabel);

    auto* controlRow = new QHBoxLayout();
    controlRow->setSpacing(6);
    controlRow->addWidget(openButton);
    controlRow->addWidget(backButton);
    controlRow->addWidget(playButton);
    controlRow->addWidget(forwardButton);
    controlRow->addWidget(speedBox);
    controlRow->addWidget(slider, 1);
    controlRow->addWidget(liveButton);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(12, 10, 12, 10);
    layout->setSpacing(6);
    layout->addLayout(titleRow);
    layout->addLayout(controlRow);
}

bool ReplayWidget::chooseRecording(const QString& startFolder)
{
    pause();

    QFileDialog dialog(this, "Open recording folder");
    dialog.setFileMode(QFileDialog::Directory);
    dialog.setOption(QFileDialog::ShowDirsOnly);
    dialog.setOption(QFileDialog::DontUseNativeDialog); // Otherwise it does not show up
    if (!startFolder.isEmpty()) {
        QDir parent(startFolder);
        parent.cdUp();
        dialog.setDirectory(parent);
        dialog.selectFile(startFolder);
    }
    if (dialog.exec() != QDialog::Accepted || dialog.selectedFiles().isEmpty()) {
        return false;
    }

    // The loader thread reads the recording: let it finish before replacing it
    if (loading.valid()) {
        loading.wait();
        loading = {};
    }
    requestedRow.reset();

    QString error;
    if (!rec.open(dialog.selectedFiles().first(), error)) {
        QMessageBox::warning(this, "Replay", error);
        return false;
    }

    titleLabel->setText(QString("REPLAY  ·  %1").arg(QFileInfo(rec.folder()).fileName()));
    titleLabel->setToolTip(rec.logFile());
    {
        const QSignalBlocker blocker(slider);
        slider->setRange(0, static_cast<int>(rec.size()) - 1);
        slider->setValue(0);
    }
    currentRow = 0;
    requestedRow = 0;
    tickTimer.start(TICK_MS);
    updateControls();
    return true;
}

const Recording& ReplayWidget::recording() const
{
    return rec;
}

void ReplayWidget::showEvent(QShowEvent* event)
{
    QFrame::showEvent(event);
    for (QShortcut* shortcut : shortcuts) {
        shortcut->setEnabled(true);
    }
    if (rec.size() > 0) {
        tickTimer.start(TICK_MS);
    }
}

void ReplayWidget::hideEvent(QHideEvent* event)
{
    QFrame::hideEvent(event);
    for (QShortcut* shortcut : shortcuts) {
        shortcut->setEnabled(false);
    }
    tickTimer.stop();
}

void ReplayWidget::onTick()
{
    // Show the row that finished loading
    if (loading.valid() && loading.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        RecordedFrame frame = loading.get();
        currentRow = frame.row;
        emit frameLoaded(frame);
        updateControls();
    }
    if (loading.valid() || rec.size() == 0) {
        return;
    }

    // Start reading the next row: a requested one, or the one that is due while playing
    if (requestedRow) {
        startLoading(*requestedRow, true);
        requestedRow.reset();
    } else if (playing) {
        if (currentRow + 1 >= rec.size()) {
            playing = false;
            updateControls();
            return;
        }
        const uint64_t now = playTime();
        if (now >= rec.time(currentRow + 1)) {
            // When reading is slower than the recording, skip to the row that is due now
            startLoading(std::max(currentRow + 1, rec.rowAt(now)), false);
        }
    }
}

void ReplayWidget::startLoading(size_t row, bool requested)
{
    loadingRow = std::min(row, rec.size() - 1);
    loadingRequested = requested;
    loading = std::async(std::launch::async, [this, row = loadingRow]() { return rec.load(row); });
}

uint64_t ReplayWidget::playTime() const
{
    return anchorTime + static_cast<uint64_t>(static_cast<double>(clock.nsecsElapsed()) / 1000.0 * speed);
}

void ReplayWidget::play()
{
    if (rec.size() == 0) {
        return;
    }
    // At the end: start over
    size_t from = requestedRow.value_or(currentRow);
    if (from + 1 >= rec.size()) {
        from = 0;
        requestedRow = 0;
    }
    anchorTime = rec.time(from);
    clock.start();
    playing = true;
    updateControls();
}

void ReplayWidget::pause()
{
    playing = false;
    updateControls();
}

void ReplayWidget::togglePlay()
{
    if (playing) {
        pause();
    } else {
        play();
    }
}

void ReplayWidget::step(int delta)
{
    if (rec.size() == 0) {
        return;
    }
    pause();
    // Step from the row on screen, or from the step that is still being read, so fast key repeats are not lost
    const size_t from = requestedRow ? *requestedRow : (loading.valid() && loadingRequested ? loadingRow : currentRow);
    const auto last = static_cast<long long>(rec.size()) - 1;
    seek(static_cast<size_t>(std::clamp(static_cast<long long>(from) + delta, 0LL, last)));
}

void ReplayWidget::seek(size_t row)
{
    if (rec.size() == 0) {
        return;
    }
    requestedRow = std::min(row, rec.size() - 1);
    if (playing) {
        anchorTime = rec.time(*requestedRow);
        clock.start();
    }
    updateControls();
}

void ReplayWidget::setSpeed(double newSpeed)
{
    if (playing) {
        anchorTime = playTime();
        clock.start();
    }
    speed = newSpeed;
}

void ReplayWidget::updateControls()
{
    const bool hasRecording = rec.size() > 0;
    for (QWidget* w : std::initializer_list<QWidget*>{backButton, playButton, forwardButton, speedBox, slider}) {
        w->setEnabled(hasRecording);
    }
    playButton->setText(playing ? "❚❚  Pause" : "▶  Play");
    playButton->setToolTip(playing ? "Pause (space)" : "Play (space)");

    if (!hasRecording) {
        statusLabel->setText("NO RECORDING");
        return;
    }

    // The slider follows the row on screen, unless the user just picked another one
    const size_t shownRow = requestedRow.value_or(currentRow);
    if (!slider->isSliderDown()) {
        const QSignalBlocker blocker(slider);
        slider->setValue(static_cast<int>(shownRow));
    }
    statusLabel->setText(QString("%1 %2 / %3 · row %4 / %5")
        .arg(playing ? "▶" : "❚❚")
        .arg(formatTime(rec.time(shownRow)))
        .arg(formatTime(rec.time(rec.size() - 1)))
        .arg(shownRow + 1)
        .arg(rec.size()));
}
