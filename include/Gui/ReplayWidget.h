#pragma once

#include <QComboBox>
#include <QElapsedTimer>
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QShortcut>
#include <QSlider>
#include <QTimer>

#include <future>
#include <optional>

#include <Gui/Recording.h>

// Plays back a recording: open, play/pause, step, seek and playback speed.
// The files of a row are read on a background thread; while playing, rows that cannot be read in time are skipped,
// so the playback keeps the recorded pace.
class ReplayWidget : public QFrame {
    Q_OBJECT
public:
    explicit ReplayWidget(QWidget* parent = nullptr);
    ~ReplayWidget();

    // Asks for a recording folder, starting in startFolder. Returns true when a recording was opened.
    bool chooseRecording(const QString& startFolder);
    const Recording& recording() const;
    void pause();

signals:
    // A row was read from disk and should be shown
    void frameLoaded(const RecordedFrame& frame);
    void liveRequested();

protected:
    void showEvent(QShowEvent* event) override;
    void hideEvent(QHideEvent* event) override;

private:
    Recording rec;
    size_t currentRow = 0;                  // Row on screen
    std::future<RecordedFrame> loading;     // Row being read
    size_t loadingRow = 0;
    bool loadingRequested = false;          // The row being read was asked for by a step / seek (not by playback)
    std::optional<size_t> requestedRow;     // Row to read next (step / seek)

    bool playing = false;
    double speed = 1.0;
    uint64_t anchorTime = 0;                // Recording time (µs) at the moment the clock was started
    QElapsedTimer clock;
    QTimer tickTimer;

    QPushButton* openButton;
    QPushButton* backButton;
    QPushButton* playButton;
    QPushButton* forwardButton;
    QComboBox* speedBox;
    QSlider* slider;
    QPushButton* liveButton;
    QLabel* titleLabel;
    QLabel* statusLabel;
    std::vector<QShortcut*> shortcuts;

    void setupUI();
    void onTick();
    void play();
    void togglePlay();
    void step(int delta);
    void seek(size_t row);
    void setSpeed(double newSpeed);
    uint64_t playTime() const;
    void startLoading(size_t row, bool requested);
    void updateControls();
};
