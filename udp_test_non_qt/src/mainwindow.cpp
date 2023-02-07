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
#include <thread>
#include <omp.h>



using namespace std;
using namespace Qt;

string STR_USB_CAM = "USB CAM";

QHostAddress RO = QHostAddress("192.168.188.100");
QHostAddress OP = QHostAddress("192.168.188.253");
//QHostAddress RO = QHostAddress("192.168.0.35");
//QHostAddress OP = QHostAddress("192.168.0.35");

uint16_t ROBOT_PORT = 9999; //RX, TX
uint16_t other_PORT = 8888;

bool isRecved = false;

MainWindow::MainWindow(QWidget *parent) :
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
        connect(socket, SIGNAL(readyRead()), this, SLOT(udp_read()));
        connection = true;
        ui->connection->setText("OK!!!");
        ui->ip->setText(RO.toString());
    }

    m_pUdpSocket->bind(RO, other_PORT, QUdpSocket::ShareAddress);
    //show();
    run();
//    std::thread t1(&MainWindow::run, this);
//    std::thread t2(&MainWindow::show, this);
//    t1.join();
//    t2.join();
    return;
}

MainWindow::~MainWindow(){
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        delete ui;
        ros::waitForShutdown();
    }
}

void MainWindow::udp_write(QString text){
    QByteArray packet;
    packet.append(text);
    qDebug() << "Message from: udp_write";
    socket->writeDatagram(packet, OP, ROBOT_PORT);
}

void MainWindow::run(){
    ros::Rate loop_rate(33);
    while ( ros::ok() ) {
        ros::spinOnce();
        ROS_INFO("!");
        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    delete ui;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void MainWindow::loop_test()
{
    isRecved=false;
    delete Cam_img;
    if(Cam_img!=NULL)Cam_img=NULL;
}

void MainWindow::Cam_Callback(const sensor_msgs::ImageConstPtr& msg_img){
    if(Cam_img == NULL && !isRecved){
        Cam_img = new cv::Mat(cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8)->image);
        if(Cam_img !=NULL){
            Original = Cam_img->clone();
            cv::resize(Original, Original, cv::Size(320, 240));
            QImage raw_image((const unsigned char*)(Original.data), Original.cols, Original.rows, QImage::Format_RGB888);
            ui->robot_cam->setPixmap(QPixmap::fromImage(raw_image.rgbSwapped()));
            isRecved=true;
            QByteArray usb_cam_image_array;
            for(int i=0 ; i < Original.rows; i++){
                for(int j=0; j < Original.cols; j++){
                    usb_cam_image_array.push_back((unsigned char)(Original.at<cv::Vec3b>(i,j)[0]));
                    usb_cam_image_array.push_back((unsigned char)(Original.at<cv::Vec3b>(i,j)[1]));
                    usb_cam_image_array.push_back((unsigned char)(Original.at<cv::Vec3b>(i,j)[2]));
                }
            }
            sendVideo(usb_cam_image_array, other_PORT);
            loop_test();
        }
    }
}

void MainWindow::udp_read(){
    QByteArray buffer;
    buffer.resize(socket->pendingDatagramSize());
    socket->readDatagram(buffer.data(), buffer.size(), &OP, &ROBOT_PORT);
    ui->log->append("############");
    ui->log->append("ip : " + OP.toString());
    ui->log->append("port9999");
    ui->log->append(buffer);
    ui->log->append("############");
}

void MainWindow::on_pushButton_clicked(){
    switch(connection) {
    case true :
        text_data.clear();
        text_data.append("//" + ui->send_text->text());
        ui->send_text->clear();
        ui->send_log->append("Send" + text_data);
        udp_write(text_data);
        return;
    case false :
        ui->send_log->append("Connection Lost!" + text_data);
        return;
    }
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
        m_pUdpSocket->writeDatagram(packet.data(),packet.size(), OP, other_PORT);
        usleep(1);

        packet_size = packet.size();
        packet.clear();
    }
}

void MainWindow::on_pushButton_2_clicked(){
    ui->send_log->clear();
}


