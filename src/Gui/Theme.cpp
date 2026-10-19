#include <Gui/Theme.h>

#include <QApplication>
#include <QFont>
#include <QPalette>
#include <QStyleFactory>

namespace Theme {

QColor statusColor(Status status)
{
    switch (status) {
        case Status::Ok: return ok;
        case Status::Waiting: return warning;
        default: return error;
    }
}

void apply(QApplication& app)
{
    app.setStyle(QStyleFactory::create("Fusion"));

    QPalette pal;
    pal.setColor(QPalette::Window, background);
    pal.setColor(QPalette::WindowText, text);
    pal.setColor(QPalette::Base, panel);
    pal.setColor(QPalette::AlternateBase, panelRaised);
    pal.setColor(QPalette::ToolTipBase, panelRaised);
    pal.setColor(QPalette::ToolTipText, text);
    pal.setColor(QPalette::Text, text);
    pal.setColor(QPalette::Button, panelRaised);
    pal.setColor(QPalette::ButtonText, text);
    pal.setColor(QPalette::Highlight, accent);
    pal.setColor(QPalette::HighlightedText, Qt::white);
    pal.setColor(QPalette::Link, accent);
    pal.setColor(QPalette::PlaceholderText, textMuted);
    pal.setColor(QPalette::Disabled, QPalette::Text, textMuted);
    pal.setColor(QPalette::Disabled, QPalette::ButtonText, textMuted);
    pal.setColor(QPalette::Disabled, QPalette::WindowText, textMuted);
    app.setPalette(pal);

    QFont font = app.font();
    font.setPointSize(10);
    app.setFont(font);

    // %1 text, %2 panel, %3 panelRaised, %4 border, %5 textMuted, %6 accent (dark green), %7 ILVO green, %8 background
    app.setStyleSheet(QString(R"(
        QToolTip { color: %1; background: %2; border: 1px solid %4; padding: 4px; }
        QFrame#card { background: %2; border: 1px solid %4; border-top: 3px solid %7; border-radius: 10px; }
        QLabel#cardTitle { color: %6; font-size: 8pt; font-weight: 700; }
        QLabel#muted { color: %5; }
        QLabel#appTitle { color: %6; font-size: 15pt; font-weight: 700; }
        QLabel#appSubtitle { color: %5; }
        QPushButton {
            background: %3; color: %1; border: 1px solid %4; border-radius: 6px;
            padding: 6px 14px; font-weight: 600;
        }
        QPushButton:hover { border-color: %6; }
        QPushButton:pressed { background: %4; }
        QPushButton:disabled { color: %5; background: %8; border-color: %4; }
        QPushButton#playButton { background: %7; border-color: %6; color: %1; }
        QPushButton#playButton:hover { background: %6; color: white; }
        QPushButton#playButton:disabled { background: %8; border-color: %4; color: %5; }
        QPushButton#recordButton { background: #d1242f; border-color: #a40e26; color: white; }
        QPushButton#recordButton:hover { background: #a40e26; }
        QPushButton#recordButton:disabled { background: %8; border-color: %4; color: %5; }
        QToolButton#rotateButton {
            background: %3; color: %1; border: 1px solid %4; border-radius: 4px;
            padding: 0px 5px; font-size: 11pt;
        }
        QToolButton#rotateButton:hover { border-color: %6; }
        QToolButton#rotateButton:pressed { background: %4; }
        QToolButton#rotateButton:disabled { color: %5; background: %8; border-color: %4; }
        QComboBox { background: %3; color: %1; border: 1px solid %4; border-radius: 6px; padding: 4px 8px; }
        QComboBox:hover { border-color: %6; }
        QComboBox QAbstractItemView { background: %2; color: %1; selection-background-color: %6; selection-color: white; }
        QSlider::groove:horizontal { height: 6px; background: %4; border-radius: 3px; }
        QSlider::sub-page:horizontal { background: %7; border-radius: 3px; }
        QSlider::handle:horizontal { width: 14px; margin: -5px 0; background: %6; border-radius: 7px; }
        QSlider::handle:horizontal:disabled { background: %4; }
        QSplitter::handle { background: %8; }
        QSplitter::handle:hover { background: %7; }
        QStatusBar { background: %2; color: %5; border-top: 3px solid %7; }
        QStatusBar QLabel { color: %5; padding: 0 8px; }
        QMenuBar { background: %2; color: %1; }
        QMenuBar::item:selected { background: %3; }
        QMenu { background: %2; color: %1; border: 1px solid %4; }
        QMenu::item:selected { background: %6; color: white; }
    )")
        .arg(text.name(), panel.name(), panelRaised.name(), border.name(), textMuted.name(), accent.name(),
             ilvoGreen.name(), background.name()));
}

}
