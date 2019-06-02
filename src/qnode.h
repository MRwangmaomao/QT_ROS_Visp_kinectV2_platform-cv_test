/**
 * @file /include/test_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2018
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef test_gui_QNODE_HPP_
#define test_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string>
#include <QThread>
#include <QStringListModel>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <QImage>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind/bind.hpp>

#include <visp3/core/vpImage.h>
#include <visp3/imgproc/vpImgproc.h>
#include <visp3/blob/vpDot2.h>
#include <visp/vpImageIo.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/vision/vpPose.h>
//#include <visp3/gui/vpDisplayX.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace test_gui
{

/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread
{
    Q_OBJECT
public:
    QImage image;
    QImage vpimage;
    QImage depth;
    vpImage<unsigned char> orignal_I;

    QNode(int argc, char** argv);
    virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void run();

    void myCallback_img(const sensor_msgs::ImageConstPtr &msg);
    void myCallback_depth(const sensor_msgs::ImageConstPtr &msg);

    /*********************
    ** Logging
    **********************/
    enum LogLevel
    {
        Debug,
        Info,
        Warn,
        Error,
        Fatal
    };

    QStringListModel* loggingModel()
        {return &logging_model;}
    void log( const LogLevel &level, const std::string &msg);

    void RecvTopicCallback(const std_msgs::StringConstPtr &msg);


    QStringListModel* loggingModelLis()
        {return &logging_listen;}
    void log_listen(const LogLevel &level, const std::string &msg);

    void depthnearestFiltering(cv::Mat & depthSrc);

Q_SIGNALS:
    void loggingUpdated();
    void loggingListen();
    void rosShutdown();
    void loggingCamera();//发出设置相机图片信号
    void loggingDepthCamera();
private:
    int init_argc;
    char** init_argv;
    cv::Mat img;
    cv::Mat dph;
    cv::Mat vpimg;
    ros::Publisher chatter_publisher;
    ros::Subscriber chatter_subscriber;
    //image_transport::Subscriber img_qhd_subscriber;
    image_transport::Subscriber img_qhd_depth_subscriber;
    QStringListModel logging_model;
    QStringListModel logging_listen;
    bool vp_blob_init_done;
    vpImagePoint germ;
    vpDot2 blob; //2D
//    vpDisplayX d;
};

}  // namespace test_gui

#endif /* test_gui_QNODE_HPP_ */
