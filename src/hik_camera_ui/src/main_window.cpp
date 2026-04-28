#include "hik_camera_ui/main_window.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent)
{
    setWindowTitle("HIK Camera UI");
    resize(1280, 720);

    setupUI();

    ros_worker_ = new RosWorker(this);
    connect(ros_worker_, &RosWorker::imageReceived,
            this, &MainWindow::onImageReceived);
    ros_worker_->start();
}

MainWindow::~MainWindow()
{
    ros_worker_->stop();
    ros_worker_->wait();
}

void MainWindow::setupUI()
{
    QWidget* central = new QWidget(this);
    setCentralWidget(central);

    image_label_ = new QLabel(this);
    image_label_->setAlignment(Qt::AlignCenter);
    image_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    exposure_slider_ = new QSlider(Qt::Horizontal, this);
    exposure_slider_->setRange(1, 100000);
    exposure_slider_->setValue(10000);
    exposure_value_label_ = new QLabel("10000 us", this);

    gain_slider_ = new QSlider(Qt::Horizontal, this);
    gain_slider_->setRange(0, 100);
    gain_slider_->setValue(0);
    gain_value_label_ = new QLabel("0", this);

    frame_rate_slider_ = new QSlider(Qt::Horizontal, this);
    frame_rate_slider_->setRange(1, 90);
    frame_rate_slider_->setValue(90);
    frame_rate_value_label_ = new QLabel("90 fps", this);

    connect(exposure_slider_, &QSlider::valueChanged,
            this, &MainWindow::onExposureChanged);
    connect(gain_slider_, &QSlider::valueChanged,
            this, &MainWindow::onGainChanged);
    connect(frame_rate_slider_, &QSlider::valueChanged,
            this, &MainWindow::onFrameRateChanged);

    QVBoxLayout* slider_layout = new QVBoxLayout();
    slider_layout->addWidget(new QLabel("曝光", this));
    slider_layout->addWidget(exposure_slider_);
    slider_layout->addWidget(exposure_value_label_);
    slider_layout->addWidget(new QLabel("增益", this));
    slider_layout->addWidget(gain_slider_);
    slider_layout->addWidget(gain_value_label_);
    slider_layout->addWidget(new QLabel("帧率", this));
    slider_layout->addWidget(frame_rate_slider_);
    slider_layout->addWidget(frame_rate_value_label_);
    slider_layout->addStretch();

    QHBoxLayout* main_layout = new QHBoxLayout(central);
    main_layout->addWidget(image_label_, 3);
    main_layout->addLayout(slider_layout, 1);
}

void MainWindow::onImageReceived(QImage image)
{
    QPixmap pixmap = QPixmap::fromImage(image).scaled(
        image_label_->size(),
        Qt::KeepAspectRatio,
        Qt::SmoothTransformation
    );
    image_label_->setPixmap(pixmap);
}

void MainWindow::onExposureChanged(int value)
{
    exposure_value_label_->setText(QString::number(value) + " us");
    ros_worker_->setParameter("exposure_time", (double)value);
}

void MainWindow::onGainChanged(int value)
{
    gain_value_label_->setText(QString::number(value));
    ros_worker_->setParameter("gain", (double)value);
}

void MainWindow::onFrameRateChanged(int value)
{
    frame_rate_value_label_->setText(QString::number(value) + " fps");
    ros_worker_->setParameter("frame_rate", (double)value);
}