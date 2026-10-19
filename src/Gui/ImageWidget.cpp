#include <Gui/ImageWidget.h>
#include <Gui/Theme.h>
#include <QPainter>
#include <QPainterPath>

ImageWidget::ImageWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(120, 120);
}

void ImageWidget::setImage(const QImage& image) {
    m_image = image;         // QImage is implicitly shared, the producer hands over its own copy
    update();                // Trigger repaint
}

void ImageWidget::clearImage() {
    m_image = QImage();
    update();
}

void ImageWidget::setPlaceholderText(const QString& text) {
    m_placeholder = text;
    update();
}

void ImageWidget::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);

    // Rounded dark background
    QPainterPath clip;
    clip.addRoundedRect(rect(), 8, 8);
    painter.setClipPath(clip);
    painter.fillRect(rect(), Theme::viewport);

    if (m_image.isNull()) {
        painter.setPen(Theme::viewportText);
        QFont font = painter.font();
        font.setPointSize(11);
        painter.setFont(font);
        painter.drawText(rect(), Qt::AlignCenter | Qt::TextWordWrap, m_placeholder);
        return;
    }

    // Fit the image inside the widget, keeping the aspect ratio
    QSize scaled = m_image.size().scaled(size(), Qt::KeepAspectRatio);
    QRect target(QPoint(0, 0), scaled);
    target.moveCenter(rect().center());
    painter.drawImage(target, m_image);
}
