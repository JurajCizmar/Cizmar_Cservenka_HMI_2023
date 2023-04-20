#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#ifdef _WIN32
#include<windows.h>
#endif
#include<iostream>
//#include<arpa/inet.h>
//#include<unistd.h>
//#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
//#include "ckobuki.h"
//#include "rplidar.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include "robot.h"

#include <QJoysticks.h>
namespace Ui {
class MainWindow;
}

///toto je trieda s oknom.. ktora sa spusti ked sa spusti aplikacia.. su tu vsetky gombiky a spustania...
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    bool useCamera1;
    bool useStopSwitch;
    //  cv::VideoCapture cap;

    int actIndex;
    //   cv::Mat frame[3];

    cv::Mat frame[3];
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    int processThisLidar(LaserMeasurement laserData);

    int processThisRobot(TKobukiData robotdata);

    int processThisCamera(cv::Mat cameraData);

    int processThisSkeleton(skeleton skeledata);

private slots:
    void on_pushButton_9_clicked();

    void on_pushButton_4_clicked();

    void getNewFrame();

    void skeletonTracker();

    void ramp();

    void rampBack();



private:

    //--skuste tu nic nevymazat... pridavajte co chcete, ale pri odoberani by sa mohol stat nejaky drobny problem, co bude vyhadzovat chyby
    Ui::MainWindow *ui;
     void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
     int updateLaserPicture;
     LaserMeasurement copyOfLaserData;
     std::string ipaddress;
     Robot robot;
     TKobukiData robotdata;
     int updateSkeletonPicture;
     skeleton skeleJoints;
     int datacounter;
     QTimer *timer;

     float rectHeight;
     float rectWidth;
     float frameHeight;
     float frameWidth;

     float diffHeight;
     float diffWidth;

     float rightIndexTipX;
     float rightIndexTipY;
     float leftIndexTipX;
     float leftIndexTipY;
     float rightWristY;
     float rightWristX;
     float leftWristX;
     float leftWristY;
     float rightMiddleTipX;
     float rightMiddleTipY;
     float leftMiddleTipX;
     float leftMiddleTipY;
     float rightPinkyTipX;
     float rightPinkyTipY;
     float rightPalmZ;
     float leftPalmZ;


     float rightIndexDistance;
     float leftIndexDistance;
     float rightMiddleDistance;
     float leftMiddleDistance;
     float rightPinkyDistance;

     int maxSpeed = 300;
     int maxSpeedBack = -300;
     int increment = 5;
     int actualSpeed = 0;

     QJoysticks *instance;

     double forwardspeed;//mm/s
     double rotationspeed;//omega/s
public slots:
     void setUiValues(double robotX,double robotY,double robotFi);
signals:
     void uiValuesChanged(double newrobotX,double newrobotY,double newrobotFi); ///toto nema telo


};

#endif // MAINWINDOW_H
