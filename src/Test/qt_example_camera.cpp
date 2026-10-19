#include <QApplication>
#include <Gui/Theme.h>
#include <Gui/MainWindowCamera.h>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    Theme::apply(app);

    MainWindowCamera window;
    window.resize(800, 600);
    window.show();

    return app.exec();
}