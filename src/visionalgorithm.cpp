#include "visionalgorithm.h"

#include <QDebug>
#include <stdio.h>
#include <iostream>
#include <sstream>

/**
 * @brief visionalgorithm::visionalgorithm
 * @param vp_src
 * @param img_src
 */
visionalgorithm::visionalgorithm(vpImage<unsigned char> &vp_src, cv::Mat &img_src):I(vp_src),img(img_src)
{
    cam.initPersProjWithoutDistortion(571.831982, 568.547768, 436.7418039, 259.987286);
}

/**
 * @brief visionalgorithm::bar_code_detection
 */
void visionalgorithm::bar_code_detection()
{
    vpDetectorBase *detector = NULL;
    detector = new vpDetectorQRCode;

   bool status = detector->detect(I);
    std::ostringstream oss;
    oss << detector->getNbObjects() <<  " bar code detected";
   cv::Point p_start = cv::Point(10,30);
   //加上字符的起始点
   cv::putText(img, oss.str(), p_start, cv::FONT_HERSHEY_TRIPLEX, 0.6, cv::Scalar(255, 200, 200), 1, CV_AA);
   oss.clear();
   //在图像上加字符
   //第一个参数为要加字符的目标函数
   //第二个参数为要加的字符
   //第三个参数为字体
   //第四个参数为子的粗细
   //第五个参数为字符的颜色
       if (status) {
         for (size_t i = 0; i < detector->getNbObjects(); i++) {
           std::vector<vpImagePoint> p = detector->getPolygon(i);
           vpRect bbox = detector->getBBox(i);
//           vpDisplay::displayRectangle(I, bbox, vpColor::green);
           oss << "Message: \"" << detector->getNbObjects() << "\"";
           p_start.y = static_cast<int>(bbox.getTop());
           p_start.x = static_cast<int>(bbox.getLeft());
           cv::putText(img, oss.str(), p_start, cv::FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(255, 0, 0), 1, CV_AA);

           for (size_t j = 0; j < p.size(); j++) {
               p_start.x = static_cast<int>(p[j].get_j());
               p_start.y = static_cast<int>(p[j].get_i());

               cv::circle(img, p_start, 3, cv::Scalar(0,255,0),-1);
             std::ostringstream number;
             number << j;
             vpDisplay::displayText(I, p[j] + vpImagePoint(15, 5), number.str(), vpColor::blue);
           }
         }
       }

       delete detector;
}

/**
 * @brief visionalgorithm::detection_object_mbt
 */
void visionalgorithm::detection_object_mbt()
{

}

/**
 * @brief visionalgorithm::aprilTagDetection
 */
void visionalgorithm::aprilTagDetection()
{

    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    double tagSize = 0.053;
    float quad_decimate = 1.0;
    int nThreads = 1;
    bool display_tag = false;
    int color_id = -1;
    unsigned int thickness = 2;
    bool z_aligned = false;

    vpDetectorAprilTag detector(tagFamily);
    detector.setAprilTagQuadDecimate(quad_decimate);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setAprilTagNbThreads(nThreads);
    detector.setDisplayTag(display_tag, color_id < 0 ? vpColor::none : vpColor::getColor(color_id), thickness);
    detector.setZAlignedWithCameraAxis(z_aligned);

    double t = vpTime::measureTimeMs();
    std::vector<vpHomogeneousMatrix> cMo_vec;
    detector.detect(I, tagSize, cam, cMo_vec);
    t = vpTime::measureTimeMs() - t;
    std::stringstream ss;
    ss << "Detection time: " << t << " ms for " << detector.getNbObjects() << " tags";
    cv::Point p_start = cv::Point(10,30);
    //加上字符的起始点
    cv::putText(img, ss.str(), p_start, cv::FONT_HERSHEY_TRIPLEX, 0.6, cv::Scalar(255, 100, 200), 1, CV_AA);

    for (size_t i = 0; i < detector.getNbObjects(); i++) {
      std::vector<vpImagePoint> p = detector.getPolygon(i);
      vpRect bbox = detector.getBBox(i);
//      cv::Point p1(static_cast<int>(bbox.getTop()),static_cast<int>(bbox.getLeft()));
//      cv::Point p2(static_cast<int>(bbox.getLeft()),static_cast<int>(bbox.getRight()));
//      cv::Point p3(static_cast<int>(bbox.getRight()),static_cast<int>(bbox.getBottom()));
//      cv::Point p4(static_cast<int>(bbox.getBottom()),static_cast<int>(bbox.getTop()));
      std::string message = detector.getMessage(i);
      std::size_t tag_id_pos = message.find("id: ");
      if (tag_id_pos != std::string::npos) {
        int tag_id = atoi(message.substr(tag_id_pos + 4).c_str());
        ss.str("");
        ss << "Tag id: " << tag_id;
        p_start.y = (int)(bbox.getTop() - 10);
        p_start.x = (int)bbox.getLeft();
        cv::putText(img, ss.str(), p_start, cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(255, 0, 0), 1, CV_AA);
      }
      for (size_t j = 0; j < p.size(); j++) {
        p_start.x = static_cast<int>(p[j].get_j());
        p_start.y = static_cast<int>(p[j].get_i());
        cv::circle(img, p_start, 3, cv::Scalar(0,255,0),-1);
        vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
        std::ostringstream number;
        number << j;
        p_start.x = static_cast<int>(p[j].get_j())+5;
        p_start.y = static_cast<int>(p[j].get_i())+5;
        cv::putText(img, number.str(), p_start, cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(255, 0, 0), 1, CV_AA);

      }
    }

    for (size_t i = 0; i < cMo_vec.size(); i++) {
//      vpDisplay::displayFrame(I, cMo_vec[i], cam, tagSize / 2, vpColor::none, 3);
        std::cout << "R: " << cMo_vec[i].getRotationMatrix() << std::endl;
        std::cout << "T: " << cMo_vec[i].getTranslationVector() << std::endl;
    }
}
