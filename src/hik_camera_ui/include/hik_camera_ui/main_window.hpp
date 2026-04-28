#pragma once

#include <QMainWindow>
#include <QLabel>
#include <QSlider>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QWidget>
#include "hik_camera_ui/ros_worker.hpp"
#include "hik_camera_ui/camera_view.hpp"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private slots:
    void onImageReceived(QImage image);
    void onExposureChanged(int value);
    void onGainChanged(int value);
    void onFrameRateChanged(int value);

private:
    void setupUI();

    CameraView* camera_view_;
    QLabel* exposure_value_label_;
    QLabel* gain_value_label_;
    QLabel* frame_rate_value_label_;
    QSlider* exposure_slider_;
    QSlider* gain_slider_;
    QSlider* frame_rate_slider_;
    RosWorker* ros_worker_;
};