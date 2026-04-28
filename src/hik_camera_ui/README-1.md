# hik_camera_ui — 海康工业相机 ROS2 Qt6 上位机

## 项目概述

本项目是一个 ROS2 + Qt6 混合应用，订阅海康工业相机驱动节点发布的图像话题，实时显示画面，并通过 ROS2 参数客户端动态调节相机曝光、增益、帧率参数。

---

## 目录结构

```
hik_camera_ui/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── hik_camera_ui/
│       ├── main_window.hpp     # Qt 主窗口声明
│       ├── ros_worker.hpp      # ROS2 线程封装声明
│       └── camera_view.hpp     # OpenGL 图像显示控件声明
└── src/
    ├── main.cpp                # 程序入口
    ├── main_window.cpp         # Qt 主窗口实现
    ├── ros_worker.cpp          # ROS2 线程封装实现
    └── camera_view.cpp         # OpenGL 图像显示控件实现
```

---

## 整体架构

```
┌─────────────────────────────────────────────────────┐
│                      主线程 (Qt)                     │
│                                                     │
│   MainWindow                                        │
│   ├── CameraView (QOpenGLWidget) → GPU 渲染图像      │
│   ├── QSlider × 3  → 曝光 / 增益 / 帧率             │
│   └── QLabel  × 3  → 显示当前参数数值               │
└──────────────────┬──────────────────────────────────┘
                   │ 信号槽 (Qt::QueuedConnection)
                   │ imageReceived(QImage)
┌──────────────────▼──────────────────────────────────┐
│                   子线程 (RosWorker)                  │
│                                                     │
│   rclcpp::spin_some(node_)  循环                    │
│   └── imageCallback()                               │
│       ├── 把 sensor_msgs::Image 转成 QImage          │
│       └── emit imageReceived(image.copy())           │
└─────────────────────────────────────────────────────┘
                   │
                   │ ROS2 话题订阅
                   │ /image_raw (BGR8, 1440×1080)
┌──────────────────▼──────────────────────────────────┐
│              hik_camera_node (相机驱动)               │
└─────────────────────────────────────────────────────┘
```

**核心设计原则：主线程只做 UI，子线程只做 ROS2。**
两者通过 Qt 信号槽通信，Qt 自动保证跨线程安全。

---

## 文件逐一讲解

### 1. CMakeLists.txt

```cmake
set(CMAKE_AUTOMOC ON)
```

Qt 的信号槽机制依赖一个叫 **moc（Meta-Object Compiler）** 的代码生成器。
它会扫描含 `Q_OBJECT` 宏的头文件，自动生成信号槽所需的元数据代码。
`AUTOMOC ON` 让 CMake 在构建时自动调用 moc，无需手动处理。

```cmake
find_package(Qt6 REQUIRED COMPONENTS Core Widgets OpenGL OpenGLWidgets)
```

加载四个 Qt6 模块：
- `Core`：Qt 基础，事件循环、信号槽、线程
- `Widgets`：窗口、按钮、滑块等 UI 控件
- `OpenGL`：OpenGL 上下文支持
- `OpenGLWidgets`：`QOpenGLWidget` 类，用于 GPU 渲染

```cmake
# Qt6 用 target_link_libraries
target_link_libraries(hik_camera_ui Qt6::Core Qt6::Widgets ...)

# ROS2 用 ament_target_dependencies
ament_target_dependencies(hik_camera_ui rclcpp sensor_msgs image_transport)
```

两套依赖系统并存：Qt6 是普通 CMake 包，ROS2 走 ament 体系，分开写互不干扰。

---

### 2. main.cpp — 程序入口

```cpp
rclcpp::init(argc, argv);   // 初始化 ROS2
QApplication app(argc, argv); // 初始化 Qt

MainWindow window;
window.show();

int result = app.exec();    // 启动 Qt 事件循环（阻塞在这里）
rclcpp::shutdown();         // Qt 退出后关闭 ROS2
return result;
```

**关键点：** `app.exec()` 是 Qt 的事件循环，它会阻塞在这里处理所有 UI 事件（鼠标点击、信号槽、定时器等），直到窗口关闭才返回。这和 PyQt5 的 `app.exec()` 完全一样。

ROS2 的 `spin` 不能在这里调用，否则会阻塞 Qt 事件循环——这就是为什么需要 RosWorker 子线程。

---

### 3. ros_worker.hpp / ros_worker.cpp — ROS2 线程封装

#### 为什么要单独开一个线程？

`rclcpp::spin()` 是一个阻塞调用，它会一直等待并处理 ROS2 消息。
如果在主线程调用，Qt 事件循环就无法运行，界面会卡死。

解决方案：把 `spin` 放进 `QThread` 子类的 `run()` 方法里，在独立线程运行。

#### 头文件关键声明

```cpp
class RosWorker : public QThread
{
    Q_OBJECT  // 必须有，才能使用信号槽

signals:
    void imageReceived(QImage image);  // 发给主线程的信号

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::AsyncParametersClient::SharedPtr param_client_;
    std::atomic<bool> running_;  // 原子布尔，线程安全的停止标志
};
```

`std::atomic<bool>` 是原子类型，多个线程同时读写它不会产生数据竞争。
主线程调用 `stop()` 写入 `false`，子线程读取它决定是否退出——这是标准的线程停止模式。

#### run() — 线程入口

```cpp
void RosWorker::run()
{
    running_ = true;
    while (running_ && rclcpp::ok()) {
        rclcpp::spin_some(node_);  // 处理一批待处理的消息，不阻塞
        msleep(1);                 // 让出 1ms，避免空转吃满 CPU
    }
}
```

使用 `spin_some` 而不是 `spin`：
- `spin` 永远阻塞，无法检查 `running_` 标志
- `spin_some` 处理完当前队列里的消息就返回，可以在循环里检查退出条件

#### imageCallback() — 图像回调

```cpp
void RosWorker::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    QImage image(
        msg->data.data(),   // 图像原始数据指针
        msg->width,
        msg->height,
        msg->step,          // 每行字节数（= width × 每像素字节数）
        QImage::Format_BGR888  // 相机输出 BGR 格式，对应此格式
    );

    emit imageReceived(image.copy());  // 必须 copy！
}
```

**为什么必须 `copy()`？**

`QImage` 构造时只是持有了指向 `msg->data` 的指针，并没有复制数据。
当 `imageCallback` 函数返回后，`msg` 的生命周期结束，数据被释放。
如果不 `copy()`，主线程收到信号准备渲染时，数据已经是野指针，程序崩溃。
`copy()` 会把图像数据深拷贝一份，生命周期由 `QImage` 对象自己管理。

#### setParameter() — 修改相机参数

```cpp
void RosWorker::setParameter(const std::string& name, double value)
{
    if (!param_client_->service_is_ready()) {
        return;  // 驱动节点未连接时直接忽略
    }
    param_client_->set_parameters({rclcpp::Parameter(name, value)});
}
```

`AsyncParametersClient` 是 ROS2 的异步参数客户端，调用后立即返回，不阻塞线程。
它向驱动节点发送参数修改请求，驱动节点收到后更新相机硬件寄存器。

---

### 4. camera_view.hpp / camera_view.cpp — OpenGL 图像显示

#### 为什么不用 QLabel？

最初用 `QLabel::setPixmap()` 显示图像，每帧都要在主线程做：
1. `QImage` → `QPixmap` 转换
2. `scaled()` 缩放（双线性插值，CPU 计算）

90fps 下这两步开销很大，导致 Qt 事件队列积压，画面越来越卡。

`QOpenGLWidget` 把渲染交给 GPU，CPU 只负责把图像数据传给显存，渲染由 GPU 完成，完全解决卡顿问题。

#### 三个必须重写的虚函数

```cpp
void CameraView::initializeGL()
{
    initializeOpenGLFunctions();        // 加载 OpenGL 函数指针
    glClearColor(0, 0, 0, 1);          // 背景色设为黑色
}

void CameraView::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);            // 窗口大小改变时更新视口
}

void CameraView::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT);      // 清空帧缓冲

    if (pending_image_.isNull()) return;

    QPainter painter(this);            // 在 OpenGL 上下文里用 Qt 画图
    // 计算保持宽高比的居中矩形
    QSize img_size = pending_image_.size();
    img_size.scale(rect().size(), Qt::KeepAspectRatio);
    QRect centered(...);
    painter.drawImage(centered, pending_image_);  // GPU 渲染
}
```

`initializeGL` → 程序启动时调用一次
`resizeGL` → 窗口大小改变时调用
`paintGL` → 每次需要重绘时调用（由 `update()` 触发）

#### updateImage() — 接收新帧

```cpp
void CameraView::updateImage(const QImage& image)
{
    pending_image_ = image;  // 保存最新帧
    update();                // 通知 Qt 下次事件循环时调用 paintGL()
}
```

`update()` 不会立即重绘，而是把重绘请求加入事件队列。
Qt 会在合适的时机（通常是下一帧）合并多个 `update()` 调用，只执行一次 `paintGL()`。
这样即使 ROS2 发来 90fps，Qt 也只会按屏幕刷新率渲染，不会过载。

---

### 5. main_window.hpp / main_window.cpp — 主窗口

#### 布局结构

```
QMainWindow
└── central (QWidget)
    └── QHBoxLayout (水平)
        ├── CameraView  (占 3/4 宽度)
        └── QVBoxLayout (占 1/4 宽度)
            ├── QLabel "曝光"
            ├── QSlider (1 ~ 100000 us)
            ├── QLabel  显示当前曝光值
            ├── QLabel "增益"
            ├── QSlider (0 ~ 100)
            ├── QLabel  显示当前增益值
            ├── QLabel "帧率"
            ├── QSlider (1 ~ 90 fps)
            └── QLabel  显示当前帧率值
```

#### 信号槽连接

```cpp
// RosWorker 子线程 → MainWindow 主线程（跨线程）
connect(ros_worker_, &RosWorker::imageReceived,
        this, &MainWindow::onImageReceived);

// 滑块 → MainWindow（同线程）
connect(exposure_slider_, &QSlider::valueChanged,
        this, &MainWindow::onExposureChanged);
```

跨线程的信号槽，Qt 默认使用 `Qt::QueuedConnection`：
发送方把信号数据打包放入接收方线程的事件队列，接收方线程的事件循环取出并执行槽函数。全程线程安全，不需要手动加锁。

#### 槽函数

```cpp
void MainWindow::onImageReceived(QImage image)
{
    camera_view_->updateImage(image);  // 转交给 OpenGL 控件渲染
}

void MainWindow::onExposureChanged(int value)
{
    exposure_value_label_->setText(QString::number(value) + " us");
    ros_worker_->setParameter("exposure_time", (double)value);
}
```

---

## 跨线程通信完整流程

```
相机硬件
  │ USB3.0
  ▼
hik_camera_node
  │ 发布 /image_raw
  ▼
RosWorker::imageCallback()   [子线程]
  │ emit imageReceived(image.copy())
  │
  │ Qt 信号队列（自动跨线程）
  ▼
MainWindow::onImageReceived()  [主线程]
  │
  ▼
CameraView::updateImage()
  │ update()
  ▼
CameraView::paintGL()          [主线程，GPU 渲染]
```

---

## 编译与运行

```bash
# 编译
cd ~/ros2_ws
colcon build --packages-select hik_camera_ui --cmake-args -DCMAKE_BUILD_TYPE=Debug

# 终端1：启动相机驱动
source ~/ros2_ws/install/setup.bash
ros2 run hik_camera hik_camera_node

# 终端2：启动上位机
source ~/ros2_ws/install/setup.bash
ros2 run hik_camera_ui hik_camera_ui
```

---

## 常见问题

**Q: 构建时报 `vtable for XXX` 链接错误**
A: moc 没有处理头文件。解决方法：把含 `Q_OBJECT` 的头文件加入 `add_executable` 列表，并清空 build 目录重建。

**Q: `ros2 node list` 没有输出**
A: 终端没有 source 环境。执行 `source ~/.bashrc` 或重开终端。

**Q: 滑块调节没有效果**
A: 参数客户端连接的节点名与实际节点名不符。用 `ros2 node list` 确认驱动节点名，在 `ros_worker.cpp` 构造函数里修正。

**Q: 画面卡顿**
A: 确认使用的是 `CameraView`（OpenGL 渲染）而不是 `QLabel`。检查 CMakeLists.txt 是否链接了 `Qt6::OpenGLWidgets`。
