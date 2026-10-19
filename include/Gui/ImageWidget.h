#pragma once

#include <QWidget>
#include <QImage>

// Shows an image scaled to fit (aspect ratio kept), or a placeholder text when there is no image
class ImageWidget : public QWidget {

public:
    ImageWidget(QWidget* parent = nullptr);

    void setImage(const QImage& image);
    void clearImage();
    void setPlaceholderText(const QString& text);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QImage m_image;
    QString m_placeholder = "No signal";
};
