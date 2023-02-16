#include "mainwindow.h"
#include "ui_mainwindow.h"


#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <omp.h>



using namespace std;
using namespace Qt;

string STR_USB_CAM = "USB CAM";


QHostAddress RO = QHostAddress("192.168.188.100");
QHostAddress OP = QHostAddress("192.168.188.253");
//QHostAddress RO = QHostAddress("192.168.0.60");
//QHostAddress OP = QHostAddress("192.168.0.130");
//QHostAddress RO = QHostAddress("192.168.0.66");
//QHostAddress OP = QHostAddress("192.168.0.77");

uint16_t ROBOT_PORT = 9999; //RX, TX
uint16_t other_PORT = 8888;

bool isRecved = false;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ros::init(init_argc,init_argv,"udp_test_non_qt");
    if ( ! ros::master::check() ) {
        return;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    Cam_sub = it.subscribe("/usb_cam/image_raw",1, &MainWindow::Cam_Callback,this);

    socket = new QUdpSocket(this);
    m_pUdpSocket = new QUdpSocket(this);

    if(socket->bind(RO, ROBOT_PORT, QUdpSocket::ShareAddress)){
        qDebug()<<"text message socket bind success"<<endl;
        ui->connection->setText("OK");
        ui->ip->setText(RO.toString());
        connect(socket, SIGNAL(readyRead()), this, SLOT(udp_read()));
        connection = true;
    }

    m_pUdpSocket->bind(RO, other_PORT, QUdpSocket::ShareAddress);
    qDebug()<<"setup ready running..."<<endl;
    th=std::thread(&MainWindow::run,this);
    return;
}

MainWindow::~MainWindow(){
    if(ros::isStarted()) {
        ROS_INFO("exiting");
        ros::shutdown(); // explicitly needed since we use ros::start();
    }
    th.join();
    delete ui;
}

void MainWindow::udp_write(QString text){
    QByteArray packet;
    packet.append(text);
    //qDebug() << "Message from: udp_write";
    socket->writeDatagram(packet, OP, ROBOT_PORT);
}

void MainWindow::run(){
    ros::Rate loop_rate(33);
    while ( ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MainWindow::loop_test()
{
    isRecved=false;
    delete Cam_img;
    if(Cam_img!=NULL)Cam_img=NULL;
}

void MainWindow::Cam_Callback(const sensor_msgs::ImageConstPtr& msg_img){
if(Cam_img == NULL && !isRecved)
  {
    Cam_img = new cv::Mat(cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8)->image);
    if(Cam_img !=NULL)
    {
      isRecved =true;
      Original = Cam_img->clone();
      cv::resize(Original, Original, cv::Size(320, 240));

      vector<int> param = vector<int>(2);
      param[0]=CV_IMWRITE_JPEG_QUALITY;
      param[1]=95;

      imencode(".jpg",Original, usb1_buff, param);

      QByteArray array;
  for(auto val: usb1_buff)
  {
    array.push_back(val);
  }
  int sendok;
  sendok = m_pUdpSocket->writeDatagram(array.data(),array.size(),OP, other_PORT);
  while (sendok==-1) {
    sendok = m_pUdpSocket->writeDatagram(array.data(),array.size(),OP, other_PORT);
  }
    udp_write("image from" + RO.toString());
    usleep(1);
    loop_test();
    }
  }
  QImage usb_image((const unsigned char*)(Original.data), Original.cols, Original.rows, QImage::Format_RGB888);
  ui->robot_cam->setPixmap(QPixmap::fromImage(usb_image.rgbSwapped()));
}

void MainWindow::udp_read(){
    QByteArray buffer;
    buffer.resize(socket->pendingDatagramSize());
    socket->readDatagram(buffer.data(), buffer.size(), &OP, &ROBOT_PORT);
    ui->log->append("----------------------");
    ui->log->append(OP.toString());
    ui->log->append("Message port: 9999");
    ui->log->append("Message: "+buffer);
    ui->log->append("----------------------");
}

void MainWindow::sendVideo(QByteArray img, uint16_t port)
{
    //230400 byte video(rows*cols*RGB CHANNEL= 240*320*3=230400) send -> 3072byte, 75times
    int arr_cnt=0;
    int packet_size=0;
    for(uint8_t packet_sequence=1;packet_sequence<76;packet_sequence++)
    {

        QByteArray packet;
        packet.push_back((char)packet_sequence);
        for(int j=0;j<3072;j++)
        {
            packet.push_back(img.at(arr_cnt));//ffffffff
            arr_cnt++;
        }

        int i = m_pUdpSocket->writeDatagram(packet.data(),packet.size(), OP, other_PORT);
        if (i == 1) ROS_INFO("Image Transfer");
        //udp_write("image from" + RO.toString());
        usleep(1);

        packet_size = packet.size();
        packet.clear();
    }

    QByteArray array;
    
}

void MainWindow::on_pushButton_clicked(){
    text_data.clear();
    text_data.append("//"+ui->send_text->text());
    udp_write(text_data);
    ui->send_log->append("ROBOT : "+ui->send_text->text());
    ui->send_text->clear();
}

void MainWindow::on_pushButton_2_clicked(){
    ui->send_log->clear();
}


