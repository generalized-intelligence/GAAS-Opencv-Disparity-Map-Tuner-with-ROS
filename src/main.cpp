#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{

    //initialize ROS related
    ros::init(argc, argv, "disparity_tuner");
    ros::NodeHandle nh;

    QApplication application(argc, argv);
    MainWindow w;

    w.show();

    return application.exec();
}
