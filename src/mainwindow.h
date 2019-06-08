#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qnode.h>
#include <QImage>//added
#include <QMutex>//added
#include <QtGui>
#include <QGraphicsPixmapItem>



using namespace test_gui;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char** argv, QWidget *parent = nullptr);
    ~MainWindow();

    void displayCamera(const QImage& image, const QImage& vpimage);//added
    void displayDepthCamera(const QImage &image);

    QGraphicsPixmapItem *g_depth_map;

private slots:
    void on_pushButtonConnect_clicked();

    void on_pushButtonKinect_clicked();
    void updateLogcamera();//added
    void updateLogDepthcamera();

    void on_pushButtonCalibration_clicked();

    void on_pushButtonTracking_clicked();

    void on_pushButtonDetection_clicked();

    void on_pushButtonKinectV2_clicked();

private:
    Ui::MainWindow *ui;
    QNode qnode;
    QImage qimage_;//added
    QImage qvpimage_;//added
    mutable QMutex qimage_mutex_;//added

    QImage qdepth_;//added
    mutable QMutex qdepth_mutex_;//added


};

#endif // MAINWINDOW_H
