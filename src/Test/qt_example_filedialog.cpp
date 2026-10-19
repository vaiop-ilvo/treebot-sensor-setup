#include <QApplication>
#include <Gui/Theme.h>
#include <Gui/MainWindowFileDialog.h>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    Theme::apply(app);
    
    // Set application properties
    app.setApplicationName("Recording Application");
    app.setApplicationVersion("1.0");
    app.setOrganizationName("Your Company");
    
    // Create and show main window
    MainWindowFileDialog window;
    window.show();
    
    return app.exec();
}