#include "visionalgorithm.h"

#include <QDebug>

visionalgorithm::visionalgorithm(vpImage<unsigned char> &src):I(src)
{
}

void visionalgorithm::bar_code_detection()
{
    vpDetectorBase *detector = NULL;
    detector = new vpDetectorQRCode;

   bool status = detector->detect(I);
   qDebug() << detector->getNbObjects() << " bar code detected    " << "status:" << status;
//       vpDisplay::displayText(I, (int)I.getHeight() - 30, 10, legend.str(), vpColor::red);
//       if (status) {
//         for (size_t i = 0; i < detector->getNbObjects(); i++) {
//           std::vector<vpImagePoint> p = detector->getPolygon(i);
//           vpRect bbox = detector->getBBox(i);
//           vpDisplay::displayRectangle(I, bbox, vpColor::green);
//           vpDisplay::displayText(I, (int)(bbox.getTop() - 10), (int)bbox.getLeft(),
//                                  "Message: \"" + detector->getMessage(i) + "\"", vpColor::red);
//           for (size_t j = 0; j < p.size(); j++) {
//             vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
//             std::ostringstream number;
//             number << j;
//             vpDisplay::displayText(I, p[j] + vpImagePoint(15, 5), number.str(), vpColor::blue);
//           }
//         }
//         vpDisplay::displayText(I, (int)I.getHeight() - 15, 10, "A click to quit...", vpColor::red);
//         vpDisplay::flush(I);
//         vpDisplay::getClick(I);
//       }
       delete detector;
}

void visionalgorithm::detection_object_mbt()
{

}

