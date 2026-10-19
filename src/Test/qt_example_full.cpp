#include <QApplication>
#include <QTimer>
#include <Gui/MainWindow.h>
#include <Gui/Theme.h>

#include <atomic>
#include <csignal>

namespace {
    // Set by Ctrl+C / SIGTERM; the event loop then quits so the cameras are closed properly
    std::atomic<bool> quitRequested{false};

    void requestQuit(int) { quitRequested.store(true); }
}

// Usage: qt_example_full [path/to/sensors.json]
int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    app.setApplicationName("Treebot sensor");
    Theme::apply(app);

    std::signal(SIGINT, requestQuit);
    std::signal(SIGTERM, requestQuit);
    QTimer quitTimer;
    QObject::connect(&quitTimer, &QTimer::timeout, &app, [] {
        if (quitRequested.load()) {
            QApplication::quit();
        }
    });
    quitTimer.start(100);

    const std::string configFile = argc > 1 ? argv[1] : "";
    MainWindow window(SensorConfig::load(configFile));
    window.resize(1600, 1000);
    window.show();

    return app.exec();
}
