#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <string>
#include <QThread>
#include <QStringListModel>
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <thread>

#include <QObject>
#include <QtNetwork/QUdpSocket>
#include <QtNetwork/QHostAddress>
#include <iostream>
#include <QDialog>
#include <QDebug>
#include <QString>

#include <sys/types.h>
#include <sys/wait.h>

#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#endif


namespace Ui {
using namespace cv;
using namespace std;
using namespace Qt;

class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    virtual ~MainWindow();
    bool init();
    void udp_write(QString text);
    void sendVideo(QByteArray img, uint16_t port);


    cv::Mat *Cam_img=NULL;
    cv::Mat Original;
    std::vector<uchar> usb1_buff;

    void run();
    bool ui_check = false;

Q_SIGNALS:
    void rosShutdown();
    void view_SIGNAL(void);

public Q_SLOTS:
    void udp_read();
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();

private:

    bool connection = false;
    Ui::MainWindow *ui;
    QUdpSocket *socket;
    QUdpSocket *m_pUdpSocket;
    QString text_data = 0;
    QByteArray buffer;
    quint8 image_cnt_past=0;
    quint8 image_cnt_now=0;

    QTimer *_5ms_Timer, *_1s_Timer;

    image_transport::Subscriber Cam_sub;
    void Cam_Callback(const sensor_msgs::ImageConstPtr& msg_img);
    int init_argc;
    char** init_argv;
    std::thread th;

private slots:
};

#endif // MAINWINDOW_H
