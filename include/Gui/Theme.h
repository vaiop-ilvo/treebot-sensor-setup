#pragma once

#include <QColor>
#include <QString>

class QApplication;

// Colours shared by all widgets of the application, based on the ILVO house style
namespace Theme {
    // ILVO brand greens
    inline const QColor ilvoGreen{0xa3, 0xcc, 0x00};
    inline const QColor ilvoGreenMid{0x8b, 0xae, 0x00};
    inline const QColor ilvoGreenDark{0x6f, 0x8b, 0x00};

    inline const QColor background{0xfa, 0xfa, 0xfa};
    inline const QColor panel{0xff, 0xff, 0xff};
    inline const QColor panelRaised{0xf5, 0xf7, 0xea};
    inline const QColor border{0xdd, 0xe3, 0xcc};
    inline const QColor text{0x1f, 0x24, 0x10};
    inline const QColor textMuted{0x6b, 0x70, 0x5c};
    inline const QColor accent = ilvoGreenDark;
    inline const QColor ok = ilvoGreenMid;
    inline const QColor warning{0xc2, 0x7c, 0x0e};
    inline const QColor error{0xd1, 0x24, 0x2f};

    // Camera images and the point cloud stay on a dark background
    inline const QColor viewport{0x11, 0x14, 0x19};
    inline const QColor viewportText{0xa7, 0xaf, 0xa0};

    enum class Status { Offline, Waiting, Ok };

    QColor statusColor(Status status);

    // Fusion style, ILVO palette and the global stylesheet
    void apply(QApplication& app);
}
