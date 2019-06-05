#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <visp3/core/vpImage.h>
#include <visp3/imgproc/vpImgproc.h>
#include <visp3/blob/vpDot2.h>
#include <visp/vpImageIo.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/vision/vpPose.h>
#include <visp3/detection/vpDetectorQRCode.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class calibration
{
public:
    calibration(vpImage<unsigned char> &src);
};

#endif // CALIBRATION_H
