#include <QApplication>
#include <Gui/Theme.h>
#include <Gui/MainWindowLidar.h>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    Theme::apply(app);

    MainWindowLidar window;
    window.resize(800, 600);
    window.show();

    return app.exec();
}