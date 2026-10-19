#pragma once

#include <QFrame>
#include <QLabel>

#include <Utils/MetaData.hpp>

class RobotWidget : public QFrame
{

public:
    explicit RobotWidget(QWidget* parent = nullptr);

    void setHost(const QString& host);
    void setMetaData(const RobotMetaData& metaData);
    void setOffline(const QString& reason);

    // Kept for the simple examples
    void setLatLon(double lat, double lon);
    void setFixState(int fix);

private:
    QLabel* statusDot;
    QLabel* hostLabel;
    QLabel* fixBadge;
    QLabel* latLonLabel;
    QLabel* speedLabel;

    static QString fixStateToString(int fix);
    static QColor fixStateToColor(int fix);
};
