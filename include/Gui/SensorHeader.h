#pragma once

#include <QHBoxLayout>
#include <QLabel>
#include <QWidget>

#include <Gui/Theme.h>

// Header row of a sensor tile: ● name   ip            stats
class SensorHeader : public QWidget {

public:
    explicit SensorHeader(const QString& title, QWidget* parent = nullptr);

    void setTitle(const QString& title);
    void setAddress(const QString& address);
    void setStatus(Theme::Status status, const QString& tooltip = QString());
    void setStats(const QString& stats);
    // Adds a small control (e.g. a tool button) at the right end of the header
    void addTool(QWidget* tool);

private:
    QLabel* dotLabel;
    QLabel* titleLabel;
    QLabel* addressLabel;
    QLabel* statsLabel;
    QHBoxLayout* layout;
};
