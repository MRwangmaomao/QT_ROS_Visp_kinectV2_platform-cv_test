/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2018
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <qnode.h>
#include <QString>
#include <QDebug>

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace test_gui
{

/*****************************************************************************
** Implementation
*****************************************************************************/
QNode::QNode(int argc, char** argv )
    : init_argc(argc), init_argv(argv)
{
    orignal_I.init(540,960);
}

QNode::~QNode()
{
    if(ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}



bool QNode::init()
{
    ros::init(init_argc,init_argv,"test_gui");
    if (!ros::master::check())
    {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub;
    image_sub = it.subscribe("/kinect2/qhd/image_color",1,&QNode::myCallback_img,this);
    image_transport::Subscriber depth_sub;
    depth_sub = it.subscribe("/kinect2/sd/image_depth",1,&QNode::myCallback_depth,this);
    start();
    ROS_INFO("I'm Starting!");

    return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url)
{
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"test_gui");
    if (!ros::master::check())
    {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub;
    image_sub = it.subscribe("/kinect2/qhd/image_color",1,&QNode::myCallback_img,this);
    image_transport::Subscriber depth_sub;
    depth_sub = it.subscribe("/kinect2/sd/image_depth",1,&QNode::myCallback_depth,this);
    start();
    return true;
}

void QNode::RecvTopicCallback(const std_msgs::StringConstPtr &msg)
{
     log_listen(Info, std::string("I heard: ")+msg->data.c_str());
}

void QNode::myCallback_img(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvCopy(msg);
        img = cv_ptrRGB->image;
        image = QImage(img.data,img.cols,img.rows,img.step[0],QImage::Format_RGB888);//change  to QImage format
        qDebug() << " " << img.rows << " " << img.cols;
        // cv mat to vpImage
        for (int x = 0; x < img.rows; x++) {
            for (int y = 0; y < img.cols; y++) {
               if(img.channels() == 3)
               {
                    orignal_I.bitmap[x*960 + y] = 0.299 * img.at<cv::Vec3b>(x,y)[0] + 0.587 * img.at<cv::Vec3b>(x,y)[1] + 0.114 * img.at<cv::Vec3b>(x,y)[2];
               }
            }
        }


        Q_EMIT loggingCamera();
        cv::waitKey(100);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void QNode::myCallback_depth(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImageConstPtr cv_ptrDEPTH;
    try
    {
        cv_ptrDEPTH = cv_bridge::toCvCopy(msg);
        dph = cv_ptrDEPTH->image;

       short max_depth;

       uchar p_depth_argb[424*512*4];
       int idx = 0;
        //find depth max
       for (int x = 0; x < dph.rows; x++) {
           for (int y = 0; y < dph.cols; y++) {
                if(dph.at<short>(x,y) > max_depth)
                    max_depth = dph.at<short>(x,y);
           }
       }

       for (int x = 0; x < dph.rows; x++) {
           for (int y = 0; y < dph.cols; y++) {
       //一定要使用1.0f相乘，转换成float类型，否则该工程的结果会有错误,因为这个要么是0，要么是1，0的概率要大很多
                   float fscale = 1.0f*(dph.at<short>(x,y))/max_depth;
                   if(dph.at<short>(x,y) != 0) {
                       p_depth_argb[idx++] = 255*(1-fscale);    //蓝色分量
                       p_depth_argb[idx++] = 0; //绿色分量
                       p_depth_argb[idx++] = 255*fscale
                               ;   //红色分量，越远越红
                       p_depth_argb[idx++] = 255*(1-fscale); //距离越近，越不透明
                   }
                   else {
                       p_depth_argb[idx++] = 0;
                       p_depth_argb[idx++] = 0;
                       p_depth_argb[idx++] = 0;
                       p_depth_argb[idx++] = 255;
                   }
            }
       }
        depth = QImage(p_depth_argb, dph.cols , dph.rows,  QImage::Format_ARGB32);//change  to QImage format
        Q_EMIT loggingDepthCamera();
        cv::waitKey(100);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void QNode::run()
{
    ros::Rate loop_rate(1);
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub;
    image_sub = it.subscribe("/kinect2/qhd/image_color",1,&QNode::myCallback_img,this);
    image_transport::Subscriber depth_sub;
    depth_sub = it.subscribe("/kinect2/sd/image_depth",1,&QNode::myCallback_depth,this);
    ros::spin();
    loop_rate.sleep();
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)

}

void QNode::log(const LogLevel &level, const std::string &msg)
{
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    switch (level)
    {
        case(Debug):
        {
            ROS_DEBUG_STREAM(msg);
            logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Info):
        {
            ROS_INFO_STREAM(msg);
            logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Warn):
        {
            ROS_WARN_STREAM(msg);
            logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Error):
        {
            ROS_ERROR_STREAM(msg);
            logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Fatal):
        {
            ROS_FATAL_STREAM(msg);
            logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
            break;
        }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_listen(const LogLevel &level, const std::string &msg)
{
    logging_listen.insertRows(logging_listen.rowCount(),1);
    std::stringstream logging_model_msg;
    switch (level)
    {
        case(Debug):
        {
            ROS_DEBUG_STREAM(msg);
            logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Info):
        {
            ROS_INFO_STREAM(msg);
            logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Warn):
        {
            ROS_WARN_STREAM(msg);
            logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Error):
        {
            ROS_ERROR_STREAM(msg);
            logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Fatal):
        {
            ROS_FATAL_STREAM(msg);
            logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
            break;
        }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_listen.setData(logging_listen.index(logging_listen.rowCount()-1),new_row);
    Q_EMIT loggingListen(); // used to readjust the scrollbar
}

}  // namespace test_gui
