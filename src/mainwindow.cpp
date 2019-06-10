/*
 * author: Peirong Wang
 * data:2019.6.9
 * description:arm robot image process and grasp task
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QSettings>
#include <QMessageBox>
#include <QDebug>
#include <QGraphicsView>


/**
 * @brief MainWindow::MainWindow
 * @param argc
 * @param argv
 * @param parent
 */
MainWindow::MainWindow(int argc, char** argv, QWidget *parent):
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    qnode(argc, argv)
{
    ui->setupUi(this);

    ui->CBCalibration->clear();
    ui->CBdetection->clear();
    ui->CBtracking->clear();

    ui->CBCalibration->addItem("IndexParam");
    ui->CBCalibration->addItem("ExternParam");
    ui->CBtracking->addItem("Blob");
    ui->CBtracking->addItem("Model-Based");
    ui->CBdetection->addItem("QBar");
    ui->CBdetection->addItem("tBox");
    ui->CBdetection->addItem("AprilTag");

    ui->pushButtonCalibration->setStyleSheet("background-color: rgb(175,238,238)");
    ui->pushButtonTracking->setStyleSheet("background-color: rgb(255,255,255)");
    ui->pushButtonDetection->setStyleSheet("background-color: rgb(255,255,255)");

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    QObject::connect(&qnode,&QNode::loggingCamera,this,&MainWindow::updateLogcamera);

    QObject::connect(&qnode,&QNode::loggingDepthCamera,this,&MainWindow::updateLogDepthcamera);

    connect(ui->pushButton_grasp_end,&QPushButton::clicked,[=](){
        qnode.grasp_end();
    });

}

/**
 * @brief MainWindow::~MainWindow
 */
MainWindow::~MainWindow()
{
    delete ui;
}

/**
 * @brief MainWindow::displayCamera
 * @param image
 * @param vpimage
 */
void MainWindow::displayCamera(const QImage &image, const QImage &vpimage)
{
    qimage_mutex_.lock();
    qimage_ = image.copy();
    ui->label_image->setPixmap(QPixmap::fromImage(qimage_));
    ui->label_image->resize(ui->label_image->pixmap()->size());
    qvpimage_ = vpimage.copy();
    ui->label_Visp->setPixmap(QPixmap::fromImage(qvpimage_));
    ui->label_Visp->resize(ui->label_Visp->pixmap()->size());
    qimage_mutex_.unlock();
}

/**
 * @brief MainWindow::displayDepthCamera
 * @param depth
 */
void MainWindow::displayDepthCamera(const QImage &depth)
{
    qdepth_mutex_.lock();
    qdepth_ = depth.copy();

    ui->labelDepthImage->setPixmap(QPixmap::fromImage(qdepth_));
    ui->labelDepthImage->resize(ui->labelDepthImage->pixmap()->size());
    qdepth_mutex_.unlock();
}

/**
 * @brief MainWindow::updateLogcamera
 */
void MainWindow::updateLogcamera()
{
    displayCamera(qnode.image, qnode.vpimage);
}

/**
 * @brief MainWindow::updateLogDepthcamera
 */
void MainWindow::updateLogDepthcamera()
{
    displayDepthCamera(qnode.depth);
}

void MainWindow::on_pushButtonConnect_clicked()
{
    qnode.init();
}

void MainWindow::on_pushButtonKinect_clicked()
{
    qnode.saveimage();
    qDebug() << "save image";
}

void MainWindow::on_pushButtonCalibration_clicked()
{
    ui->pushButtonCalibration->setStyleSheet("background-color: rgb(175,238,238)");
    ui->pushButtonTracking->setStyleSheet("background-color: rgb(255,255,255)");
    ui->pushButtonDetection->setStyleSheet("background-color: rgb(255,255,255)");
    QString string=ui->CBCalibration->currentText();
    if(string == "IndexParam"){
        system("gnome-terminal -x zsh -c 'rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024 image:=/kinect2/qhd/image_color  limited:=true'&");
        qnode.setMode(qnode.Calib_Index);
    }
    else if(string == "ExternParam"){
        qnode.setMode(qnode.Calib_Extern);
    }
    else {

    }

}

/**
 * @brief MainWindow::on_pushButtonTracking_clicked
 */
void MainWindow::on_pushButtonTracking_clicked()
{
    ui->pushButtonTracking->setStyleSheet("background-color: rgb(175,238,238)");
    ui->pushButtonCalibration->setStyleSheet("background-color: rgb(255,255,255)");
    ui->pushButtonDetection->setStyleSheet("background-color: rgb(255,255,255)");
    QString string=ui->CBtracking->currentText();
    if(string == "Blob"){
        qnode.setMode(qnode.Track_Blob);
    }
    else if(string == "Model-Based"){
        qnode.setMode(qnode.Track_Model_Based);
    }
    else {

    }
}

/**
 * @brief MainWindow::on_pushButtonDetection_clicked
 */
void MainWindow::on_pushButtonDetection_clicked()
{
    ui->pushButtonDetection->setStyleSheet("background-color: rgb(175,238,238)");
    ui->pushButtonTracking->setStyleSheet("background-color: rgb(255,255,255)");
    ui->pushButtonCalibration->setStyleSheet("background-color: rgb(255,255,255)");
    QString string=ui->CBdetection->currentText();
    if(string == "QBar"){
        qnode.setMode(qnode.Detec_Qbar);
    }

    else if(string == "tBox"){
        qnode.setMode(qnode.Detec_TBox);
    }

    else if(string == "AprilTag"){
        qnode.setMode(qnode.Detec_april_tag);
    }

    else {

    }
}

/**
 * @brief MainWindow::on_pushButtonKinectV2_clicked
 */
void MainWindow::on_pushButtonKinectV2_clicked()
{
    system("gnome-terminal -x zsh -c 'source ~/catkin_kinect/devel/setup.zsh;roslaunch kinect2_bridge kinect2_bridge.launch limited:=true'&");
}

