#include "hik_camera_ui/camera_view.hpp"

CameraView::CameraView(QWidget* parent)
    : QOpenGLWidget(parent)
{
}

CameraView::~CameraView()
{
}

void CameraView::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
}

void CameraView::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

void CameraView::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT);

    if (pending_image_.isNull()) {
        return;
    }

    QPainter painter(this);
    QRect target = rect();
    QSize img_size = pending_image_.size();
    img_size.scale(target.size(), Qt::KeepAspectRatio);
    QRect centered(
        (target.width()  - img_size.width())  / 2,
        (target.height() - img_size.height()) / 2,
        img_size.width(),
        img_size.height()
    );
    painter.drawImage(centered, pending_image_);
}

void CameraView::updateImage(const QImage& image)
{
    pending_image_ = image;
    update();
}
