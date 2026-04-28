#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include "hik_camera_ui/main_window.hpp"

int main(int argc, char* argv[] )
{
    //同时初始化ROS2和QT
    rclcpp::init(argc,argv);
    QApplication app(argc,argv);

    MainWindow window;
    window.show();
    

    int result = app.exec();
    return result;
}
