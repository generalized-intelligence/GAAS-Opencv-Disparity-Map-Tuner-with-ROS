#include "mainwindow.h"
#include "ui_mainwindow.h"



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    int RangeOfDisparity = 16;
    int SizeOfBlockWindow = 7;
    bmState = cv::StereoBM::create(RangeOfDisparity, SizeOfBlockWindow);

    // the default values used in OpenCV are defined here:
    // https://github.com/Itseez/opencv/blob/master/modules/calib3d/src/stereobm.cpp

    // we override the default values defined in the UI file with Qt Designer
    // to the ones defined above
    int preFilterSize = 41;
    int preFilterCap = 31;
    int minDisparity= -64;
    int textureThreshold = 10;
    int uniquenessRatio = 15;
    int speckleWindowSize = 0;
    int speckleRange = 0;
    int disp12MaxDiff = -1;

    bmState->setPreFilterSize (preFilterSize);  // must be an odd between 5 and 255
    bmState->setPreFilterCap (preFilterCap);  // must be within 1 and 63
    //bmState->setSADWindowSize (41);  // must be odd, be within 5..255 and be not larger than image width or height
    bmState->setMinDisparity (minDisparity);
    //bmState->setNumberOfDisparities(128);  // must be > 0 and divisible by 16
    bmState->setTextureThreshold(textureThreshold);  // must be non-negative
    bmState->setUniquenessRatio (uniquenessRatio);  // must be non-negative
    bmState->setSpeckleWindowSize(speckleWindowSize);
    bmState->setSpeckleRange (speckleRange);
    bmState->setDisp12MaxDiff(disp12MaxDiff);

    ui->horizontalSlider_pre_filter_size->setValue(preFilterSize);
    bmState->setPreFilterSize (preFilterSize);

    ui->horizontalSlider_pre_filter_cap->setValue(preFilterCap);
    bmState->setPreFilterCap(preFilterCap);

    //ui->horizontalSlider_SAD_window_size->setValue(bmState->state->SADWindowSize);
    ui->horizontalSlider_min_disparity->setValue(minDisparity);
    bmState->setMinDisparity(minDisparity);

    //ui->horizontalSlider_num_of_disparity->setValue(bmState->state->numberOfDisparities);
    ui->horizontalSlider_texture_threshold->setValue(textureThreshold);
    bmState->setTextureThreshold(textureThreshold);

    ui->horizontalSlider_uniqueness_ratio->setValue(uniquenessRatio);
    bmState->setUniquenessRatio (uniquenessRatio);

    ui->horizontalSlider_speckle_window_size->setValue(speckleWindowSize);
    bmState->setSpeckleWindowSize(speckleWindowSize);

    ui->horizontalSlider_speckle_range->setValue(speckleRange);
    bmState->setSpeckleRange (speckleRange);

    ui->horizontalSlider_disp_12_max_diff->setValue(disp12MaxDiff);
    bmState->setDisp12MaxDiff(disp12MaxDiff);
}

MainWindow::~MainWindow()
{
    delete ui;
}

//void MainWindow::ros_init()
//{
//    cout<<"subscribed to left image topic: "<<"/mynteye/left/image_rect"<<endl;
//    cout<<"sbuscribed to right image topic: "<<"/mynteye/right/image_rect"<<endl;
//
////    left_sub = nh.subscribe("/mynteye/left/image_rect", 1000, &MainWindow::FetchLeftImage, this);
////    left_sub = nh.subscribe("/mynteye/right/image_rect", 1000, &MainWindow::FetchRightImage, this);
//
//    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/mynteye/left/image_rect", 10);
//    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/mynteye/right/image_rect", 10);
//
//    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
//    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
//    sync.setMaxIntervalDuration(ros::Duration(0.01));
//    sync.registerCallback(boost::bind(&MainWindow::FetchImages, _1, _2));
//
//
//    cout<<"spining"<<endl;
//
//    ros::spin();
//
//    cout<<"after spining"<<endl;
//}


void MainWindow::ros_init()
{
    cout<<"subscribed to left image topic: "<<"/mynteye/left/image_rect"<<endl;
    cout<<"sbuscribed to right image topic: "<<"/mynteye/right/image_rect"<<endl;

//    left_sub = nh.subscribe("/mynteye/left/image_rect", 1000, &MainWindow::FetchLeftImage, this);
//    left_sub = nh.subscribe("/mynteye/right/image_rect", 1000, &MainWindow::FetchRightImage, this);

    string left_topic = "/mynteye/left/image_rect";
    string right_topic = "/mynteye/right/image_rect";

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, left_topic, 10);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, right_topic, 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.setMaxIntervalDuration(ros::Duration(0.01));
    sync.registerCallback(boost::bind(&MainWindow::FetchImages, this, _1, _2));

    cout<<"spining"<<endl;

    ros::spin();

    cout<<"after spining"<<endl;
}


void MainWindow::FetchLeftImage(const sensor_msgs::ImageConstPtr& msgLeft)
{
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    
    cout<<"subscribing images from left image topic."<<endl;

    left_image = cv_ptrLeft->image;


    // load image

    // QPixmap::fromImage(QImage((unsigned char*) mat.data, mat.cols, mat.rows, QImage::Format_RGB888));
    // some computation to resize the image if it is too big to fit in the GUI
    QPixmap left_pixmap = QPixmap::fromImage(QImage((unsigned char*) left_image.data, left_image.cols, left_image.rows, QImage::Format_RGB888));
    int max_width  = std::min(ui->label_image_left->maximumWidth(), left_image.cols);
    int max_height = std::min(ui->label_image_left->maximumHeight(), left_image.rows);
    ui->label_image_left->setPixmap(left_pixmap.scaled(max_width, max_height, Qt::KeepAspectRatio));

    set_SADWindowSize();  // the SAD window size parameter depends on the size of the image



    cout<<"fetching left image succeed, start doing disparity map generation"<<endl;

    if(!left_image.empty() && !right_image.empty())
    {
        compute_depth_map();
    }
}

void MainWindow::FetchRightImage(const sensor_msgs::ImageConstPtr& msgRight)
{
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    cout<<"subscribing images."<<endl;

    right_image = cv_ptrRight->image;


    QPixmap right_pixmap = QPixmap::fromImage(QImage((unsigned char*) right_image.data, right_image.cols, right_image.rows, QImage::Format_Grayscale8));

    int max_width  = std::min(ui->label_image_right->maximumWidth(), right_image.cols);
    int max_height = std::min(ui->label_image_right->maximumHeight(), right_image.rows);
    ui->label_image_right->setPixmap(right_pixmap.scaled(max_width, max_height, Qt::KeepAspectRatio));


    set_SADWindowSize();  // the SAD window size parameter depends on the size of the image
}


void MainWindow::FetchImages(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight)
{
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cout<<"subscribing images."<<endl;

    left_image = cv_ptrLeft->image;
    right_image = cv_ptrRight->image;

    // load images
    // some computation to resize the image if it is too big to fit in the GUI
    // QPixmap::fromImage(QImage((unsigned char*) mat.data, mat.cols, mat.rows, QImage::Format_RGB888));
    QPixmap left_pixmap = QPixmap::fromImage(QImage((unsigned char*) left_image.data, left_image.cols, left_image.rows, QImage::Format_Grayscale8));
    int max_width_left  = std::min(ui->label_image_left->maximumWidth(), left_image.cols);
    int max_height_left = std::min(ui->label_image_left->maximumHeight(), left_image.rows);
    ui->label_image_left->setPixmap(left_pixmap.scaled(max_width_left, max_height_left, Qt::KeepAspectRatio));

    QPixmap right_pixmap = QPixmap::fromImage(QImage((unsigned char*) right_image.data, right_image.cols, right_image.rows, QImage::Format_Grayscale8));
    int max_width_right  = std::min(ui->label_image_right->maximumWidth(), right_image.cols);
    int max_height_right = std::min(ui->label_image_right->maximumHeight(), right_image.rows);
    ui->label_image_right->setPixmap(right_pixmap.scaled(max_width_right, max_height_right, Qt::KeepAspectRatio));

    set_SADWindowSize();  // the SAD window size parameter depends on the size of the image

    compute_depth_map();
}


// method called when the button to change the left image is clicked
// we prompt the user to select an image, and we display it
void MainWindow::on_pushButton_left_clicked()
{
    // we prompt the user with a file dialog,
    // to select the picture file from the left camera
    QString filename = QFileDialog::getOpenFileName(this, "Select left picture file", QDir::homePath(), NULL);
    if (filename.isNull() || filename.isEmpty())
        return;

    ///// Qt display stuff

    // we load the picture from the file, to display it in a QLabel in the GUI
    QImage left_picture;
    left_picture.load(filename);

    // some computation to resize the image if it is too big to fit in the GUI
    QPixmap left_pixmap = QPixmap::fromImage(left_picture);
    int max_width  = std::min(ui->label_image_left->maximumWidth(), left_picture.width());
    int max_height = std::min(ui->label_image_left->maximumHeight(), left_picture.height());
    ui->label_image_left->setPixmap(left_pixmap.scaled(max_width, max_height, Qt::KeepAspectRatio));

    set_SADWindowSize();  // the SAD window size parameter depends on the size of the image

    ///// OpenCV disparity map computation

    // we convert filename from QString to std::string (needed by OpenCV)
    std::string filename_s = filename.toUtf8().constData();

    // we load the picture in the OpenCV Mat format, to compute depth map
    cv::Mat mat = cv::imread(filename_s, CV_LOAD_IMAGE_COLOR);
    cv::cvtColor(mat, mat, CV_BGR2GRAY);  // we convert to gray, needed to compute depth map
    this->left_image = mat;

    compute_depth_map();
}

// method called when the button to change the right image is clicked
// we prompt the user to select an image, and we display it
void MainWindow::on_pushButton_right_clicked()
{
    // we prompt the user with a file dialog,
    // to select the picture file from the left camera
    QString filename = QFileDialog::getOpenFileName(this, "Select right picture file", QDir::homePath(), NULL);
    if (filename.isNull() || filename.isEmpty())
        return;

    ///// Qt display stuff

    // we load the picture from the file, to display it in a QLabel in the GUI
    QImage right_picture;
    right_picture.load(filename);

    // some computation to resize the image if it is too big to fit in the GUI
    QPixmap right_pixmap = QPixmap::fromImage(right_picture);
    int max_width  = std::min(ui->label_image_right->maximumWidth(), right_picture.width());
    int max_height = std::min(ui->label_image_right->maximumHeight(), right_picture.height());
    ui->label_image_right->setPixmap(right_pixmap.scaled(max_width, max_height, Qt::KeepAspectRatio));

    set_SADWindowSize();  // the SAD window size parameter depends on the size of the image

    ///// OpenCV disparity map computation

    // we convert filename from QString to std::string (needed by OpenCV)
    std::string filename_s = filename.toUtf8().constData();

    // we load the picture in the OpenCV Mat format, to compute depth map
    cv::Mat mat = cv::imread(filename_s, CV_LOAD_IMAGE_COLOR);
    cv::cvtColor(mat, mat, CV_BGR2GRAY);  // we convert to gray, needed to compute depth map
    this->right_image = mat;

    compute_depth_map();
}

// we compute the depth map, if both left image and right image have been added
void MainWindow::compute_depth_map() {
    // we check that both images have been loaded
    if (this->left_image.empty() || this->right_image.empty())
        return;

    // we check that both images have the same size (else OpenCV throws an error)
    if (left_image.rows != right_image.rows || left_image.cols != right_image.cols) {
        ui->label_depth_map->setText("Can't compute depth map: left and right images should be the same size");
        return;
    }

    // we compute the depth map
    cv::Mat disparity_16S;  // 16 bits, signed
    bmState->compute(left_image, right_image, disparity_16S);
    //bmState(left_image, right_image, disparity_16S);

    // we convert the depth map to a QPixmap, to display it in the QUI
    // first, we need to convert the disparity map to a more regular grayscale format
    // then, we convert to RGB, and finally, we can convert to a QImage and then a QPixmap

    // we normalize the values, so that they all fit in the range [0, 255]
    cv::normalize(disparity_16S, disparity_16S, 0, 255, CV_MINMAX);

    // we convert the values from 16 bits signed to 8 bits unsigned
    cv::Mat disp(disparity_16S.rows, disparity_16S.cols, CV_8UC1);
    for (int i=0; i<disparity_16S.rows; i++)
        for (int j=0; j<disparity_16S.cols; j++)
            disp.at<unsigned char>(i,j) = (unsigned char)disparity_16S.at<short>(i,j);

    // we convert from gray to color
    cv::Mat disp_color;
    cv::cvtColor(disp, disp_color, CV_GRAY2RGB);

    // we finally can convert the image to a QPixmap and display it
    QImage disparity_image = QImage((unsigned char*) disp_color.data, disp_color.cols, disp_color.rows, QImage::Format_RGB888);
    QPixmap disparity_pixmap = QPixmap::fromImage(disparity_image);

    // some computation to resize the image if it is too big to fit in the GUI
    int max_width  = std::min(ui->label_depth_map->maximumWidth(),  disparity_image.width());
    int max_height = std::min(ui->label_depth_map->maximumHeight(), disparity_image.height());
    ui->label_depth_map->setPixmap(disparity_pixmap.scaled(max_width, max_height, Qt::KeepAspectRatio));

    ui->label_depth_map->setPixmap(disparity_pixmap);
}


/////////////////// Sliders management (callbacks and constraints) //////////////////////

///// pre filter size

// must be an odd number
void MainWindow::on_horizontalSlider_pre_filter_size_valueChanged(int value)
{
    if ((value % 2) == 0) {
        value -= 1;
        ui->horizontalSlider_pre_filter_size->setValue(value);
    }

    bmState->setPreFilterSize(value);
    compute_depth_map();
}

///// pre filter cap

void MainWindow::on_horizontalSlider_pre_filter_cap_valueChanged(int value)
{
    bmState->setPreFilterCap(value);
    compute_depth_map();
}

///// SAD window size

// the SAD Window size should always be smaller than the size of the images
// so when a new image is loaded, we set the maximum value for the slider
void MainWindow::set_SADWindowSize() {
    int value = 255;  // max value allowed

    // we check that the value is not bigger than the size of the pictures
    if (! left_image.empty())
        value = std::min(value, std::min(left_image.cols, left_image.rows));
    if (! right_image.empty())
        value = std::min(value, std::min(right_image.cols, right_image.rows));

    // we check that the value is >= 5
    value = std::max(value, 5);

    ui->horizontalSlider_SAD_window_size->setMaximum(value);
}

// must be an odd number
void MainWindow::on_horizontalSlider_SAD_window_size_valueChanged(int value)
{
    if ((value % 2) == 0) {
        value -= 1;
        ui->horizontalSlider_SAD_window_size->setValue(value);
    }

    //bmState->state->SADWindowSize = value;
    compute_depth_map();
}

///// Minimum disparity

void MainWindow::on_horizontalSlider_min_disparity_valueChanged(int value)
{
    bmState->setMinDisparity(value);
    compute_depth_map();
}

///// Number of disparities

// callback when slider for number of disparities is moved
// we must allow only values that are divisible by 16
void MainWindow::on_horizontalSlider_num_of_disparity_sliderMoved(int value)
{
    set_num_of_disparity_slider_to_multiple_16(value);
}

// callback when slider for number of disparities is changed
// (for the case when the slider is not moved (just a click), because then the callback above is not called)
// we must allow only values that are divisible by 16
void MainWindow::on_horizontalSlider_num_of_disparity_valueChanged(int value)
{
    set_num_of_disparity_slider_to_multiple_16(value);
}

void MainWindow::set_num_of_disparity_slider_to_multiple_16(int value) {
    if ((value % 16) != 0) {
        value -= (value % 16);
        ui->horizontalSlider_num_of_disparity->setValue(value);
    }

    //bmState->state->numberOfDisparities = value;
    compute_depth_map();
}

///// Texture threshold

void MainWindow::on_horizontalSlider_texture_threshold_valueChanged(int value)
{
    bmState->setTextureThreshold(value);
    compute_depth_map();
}

///// Uniqueness ratio

void MainWindow::on_horizontalSlider_uniqueness_ratio_valueChanged(int value)
{
    bmState->setUniquenessRatio(value);
    compute_depth_map();
}

///// Speckle window size

void MainWindow::on_horizontalSlider_speckle_window_size_valueChanged(int value)
{
    bmState->setSpeckleWindowSize(value);
    compute_depth_map();
}

///// Speckle range

void MainWindow::on_horizontalSlider_speckle_range_valueChanged(int value)
{
    bmState->setSpeckleRange(value);
    compute_depth_map();
}

///// Disparity 12 maximum difference

void MainWindow::on_horizontalSlider_disp_12_max_diff_valueChanged(int value)
{
    bmState->setDisp12MaxDiff(value);
    compute_depth_map();
}

//Data binding of linetxt_left&right to the QStrings
//Do whatever you would like to do
void MainWindow::on_linetxt_left_textChanged(const QString &arg1)
{
    this->left_txtline= ui->linetxt_left->text();
    cout<<left_txtline.toStdString()<<endl;
}

void MainWindow::on_linetxt_right_textChanged(const QString &arg1)
{
    this->right_txtline=ui->linetxt_right->text();
    cout<<right_txtline.toStdString()<<endl;
}


void MainWindow::show()
{
    QWidget::show();

    std::thread ros_thread(&MainWindow::ros_init, this);
    ros_thread.detach();
//    ros::spin();

}