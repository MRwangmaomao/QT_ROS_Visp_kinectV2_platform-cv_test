#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QSettings>
#include <QMessageBox>
#include <QDebug>
#include <QGraphicsView>

MainWindow::MainWindow(int argc, char** argv, QWidget *parent):
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    qnode(argc, argv)
{
    ui->setupUi(this);
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    QObject::connect(&qnode,&QNode::loggingCamera,this,&MainWindow::updateLogcamera);

    QObject::connect(&qnode,&QNode::loggingDepthCamera,this,&MainWindow::updateLogDepthcamera);


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::displayCamera(const QImage &image)
{
    qimage_mutex_.lock();
    qimage_ = image.copy();
    ui->label_image->setPixmap(QPixmap::fromImage(qimage_));
    ui->label_image->resize(ui->label_image->pixmap()->size());
    qimage_mutex_.unlock();
}

void MainWindow::displayDepthCamera(const QImage &depth)
{
    qdepth_mutex_.lock();
    qdepth_ = depth.copy();

    ui->labelDepthImage->setPixmap(QPixmap::fromImage(qdepth_));
    ui->labelDepthImage->resize(ui->labelDepthImage->pixmap()->size());
    qdepth_mutex_.unlock();
}

void MainWindow::updateLogcamera()
{
    displayCamera(qnode.image);
}

void MainWindow::updateLogDepthcamera()
{
    displayDepthCamera(qnode.depth);
//    qDebug()<<"I receive depth image!";
}

void MainWindow::on_pushButtonConnect_clicked()
{
    qnode.init();
}

void MainWindow::on_pushButtonKinect_clicked()
{
}
