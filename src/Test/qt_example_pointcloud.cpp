#include <QApplication>
#include <Gui/PointCloudWidgetExample.h>

int main(int argc, char** argv) {
    QApplication app(argc, argv);

    PointCloudWidgetExample w;
    w.resize(800, 600);
    w.show();

    return app.exec();
}
