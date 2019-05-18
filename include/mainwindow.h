#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>

#include "opencv2/core.hpp"
#include <opencv2/calib3d.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <QMainWindow>
#include <QFileDialog>

// newly added for ROS and cvbridge
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/duration.h>
#include <math.h>

#include <thread>


namespace Ui {
    class MainWindow;
}


using namespace std;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    explicit MainWindow(QWidget *parent = 0);

    ~MainWindow();

private slots:


    void on_pushButton_left_clicked();

    void on_pushButton_right_clicked();

    void on_horizontalSlider_num_of_disparity_sliderMoved(int position);

    void on_horizontalSlider_num_of_disparity_valueChanged(int value);

    void on_horizontalSlider_SAD_window_size_valueChanged(int value);

    void on_horizontalSlider_pre_filter_size_valueChanged(int value);

    void on_horizontalSlider_pre_filter_cap_valueChanged(int value);

    void on_horizontalSlider_min_disparity_valueChanged(int value);

    void on_horizontalSlider_texture_threshold_valueChanged(int value);

    void on_horizontalSlider_uniqueness_ratio_valueChanged(int value);

    void on_horizontalSlider_speckle_window_size_valueChanged(int value);

    void on_horizontalSlider_speckle_range_valueChanged(int value);

    void on_horizontalSlider_disp_12_max_diff_valueChanged(int value);

    void FetchLeftImage(const sensor_msgs::ImageConstPtr& msgLeft);

    void FetchRightImage(const sensor_msgs::ImageConstPtr& msgRight);

    void FetchImages(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight);

    void on_linetxt_left_textChanged(const QString &arg1);

    void on_linetxt_right_textChanged(const QString &arg1);

private:
    // the UI object, to access the UI elements created with Qt Designer
    Ui::MainWindow *ui;

    // the left and right pictures, converted to OpenCV Mat format
    cv::Mat left_image;
    cv::Mat right_image;

    // the object that holds the parameters for the block-matching algorithm
    cv::Ptr<cv::StereoBM> bmState;

    void compute_depth_map();  // compute depth map with OpenCV

    // functions to manage constraints on sliders
    void set_SADWindowSize();  // manage max value of SADWindowSize slider
    void set_num_of_disparity_slider_to_multiple_16(int position);
    QString left_txtline;
    QString right_txtline;

public:

    void ros_init();

    void show();

//    ros::Rate rate;

    ros::NodeHandle nh;
    ros::Subscriber left_sub, right_sub;

    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_bridge::CvImageConstPtr cv_ptrRight;

};

#endif // MAINWINDOW_H
