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


//    while(ros::ok())
//    {
////        //ros::spin();
//        ros::spinOnce;
//        rate.sleep();
//    }

    return application.exec();
}
