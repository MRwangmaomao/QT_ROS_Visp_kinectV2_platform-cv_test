#ifndef VISIONALGORITHM_H
#define VISIONALGORITHM_H

#include <visp3/core/vpImage.h>
#include <visp3/imgproc/vpImgproc.h>
#include <visp3/blob/vpDot2.h>
#include <visp/vpImageIo.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/vision/vpPose.h>
#include <visp3/detection/vpDetectorQRCode.h>
#include <visp3/detection/vpDetectorDataMatrixCode.h>
//#include <visp3/gui/vpDisplayX.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <iostream>
#include <vector>

class visionalgorithm
{
public:
    visionalgorithm(vpImage<unsigned char> &src);
    void bar_code_detection();
    void detection_object_mbt();
    vpImage<unsigned char> I;
};

#endif // VISIONALGORITHM_H

