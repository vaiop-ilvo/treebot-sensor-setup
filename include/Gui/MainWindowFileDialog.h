#pragma once
#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QWidget>
#include <QLabel>
#include <QStatusBar>
#include <QMenuBar>
#include <QAction>
#include <Gui/RecordWidget.h>

class MainWindowFileDialog : public QMainWindow {

public:
    explicit MainWindowFileDialog(QWidget* parent = nullptr);
    ~MainWindowFileDialog() = default;

private slots:
    void onRecordingStarted();
    void onRecordingPaused();
    void onFolderChanged(const QString& folder);

private:
    void setupUI();
    void setupMenuBar();

    // Widgets
    QWidget* centralWidget;
    RecordWidget* recordWidget;
    
    // Layout
    QHBoxLayout* mainLayout;
    
    // Menu actions
    QAction* exitAction;
    QAction* aboutAction;
};