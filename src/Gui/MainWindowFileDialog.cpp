#include <Gui/MainWindowFileDialog.h>
#include <QApplication>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QStatusBar>
#include <QMenuBar>
#include <QAction>
#include <QDebug>

MainWindowFileDialog::MainWindowFileDialog(QWidget* parent)
    : QMainWindow(parent)
{
    setupUI();
    setupMenuBar();
    
    // Set window properties
    setWindowTitle("Recording Application");
    setMinimumSize(600, 400);
    resize(800, 500);
    
    // Initialize status
    statusBar()->showMessage("Ready to record", 2000);
}

void MainWindowFileDialog::setupUI() {
    // Create central widget
    centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);
    
    // Create main layout
    mainLayout = new QHBoxLayout(centralWidget);
    mainLayout->setContentsMargins(20, 20, 20, 20);
    mainLayout->setSpacing(15);
    
    // Add some spacing
    mainLayout->addSpacing(20);
    
    // Create and add record widget
    recordWidget = new RecordWidget(this);
    mainLayout->addWidget(recordWidget);
    connect(recordWidget, &RecordWidget::recordingChanged, this, [this](bool recording) {
        if (recording) {
            onRecordingStarted();
        } else {
            onRecordingPaused();
        }
    });
    connect(recordWidget, &RecordWidget::folderChanged, this, &MainWindowFileDialog::onFolderChanged);
    
    // Add stretch to push everything to top
    mainLayout->addStretch();
    
    // Add status bar
    statusBar()->showMessage("Application started");
}

void MainWindowFileDialog::setupMenuBar() {
    // File menu
    QMenu* fileMenu = menuBar()->addMenu("&File");
    
    exitAction = new QAction("E&xit", this);
    exitAction->setShortcut(QKeySequence::Quit);
    exitAction->setStatusTip("Exit the application");
    connect(exitAction, &QAction::triggered, this, &QWidget::close);
    fileMenu->addAction(exitAction);
    
    // Help menu
    QMenu* helpMenu = menuBar()->addMenu("&Help");
    
    aboutAction = new QAction("&About", this);
    aboutAction->setStatusTip("Show the application's About box");
    connect(aboutAction, &QAction::triggered, this, [this]() {
        QMessageBox::about(this, "About Recording App",
                          "Recording Application v1.0\n\n"
                          "A simple recording control interface.");
    });
    helpMenu->addAction(aboutAction);
}

void MainWindowFileDialog::onRecordingStarted() {
    qDebug() << "Status: Recording...";
    statusBar()->showMessage("Recording started");
    
    // You can add your recording logic here
    qDebug() << "Recording started in MainWindowFileDialog";
}

void MainWindowFileDialog::onRecordingPaused() {
    qDebug() << "Status: Paused";
    statusBar()->showMessage("Recording paused");
    
    // You can add your pause logic here
    qDebug() << "Recording paused in MainWindowFileDialog";
}

void MainWindowFileDialog::onFolderChanged(const QString& folder) {
    qDebug() << "Status: Folder set to " << folder;
    statusBar()->showMessage(QString("Output folder: %1").arg(folder));
    
    // You can add your folder handling logic here
    qDebug() << "Output folder changed to:" << folder;
}