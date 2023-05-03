#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include<windows.h>
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
//#include "opencv2/core/utility.hpp"
//#include "opencv2/videoio.hpp"
//#include "opencv2/imgcodecs.hpp"
#include "robot.h"
#include <list>

#include <QJoysticks.h>

#define ROWS 46
#define COLS 57

#define M_PI           3.14159265358979323846  /* pi */

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
  //  cv::VideoCapture cap;

    int actIndex;

    cv::Mat frame[3];

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    int processThisLidar(LaserMeasurement laserData);

    int processThisRobot(TKobukiData robotdata);

int processThisCamera(cv::Mat cameraData);

private slots:

    void startup();

    void on_pushButton_4_clicked();

    void mousePressEvent(QMouseEvent *event);

    void mapping();

    void getOdometry(TKobukiData robotdata);

    void on_pushButton_11_clicked();

    void on_pushButton_12_clicked();

    void getIndex();

    void insert_map();

    void on_pushButton_14_clicked();

    void on_pushButton_15_clicked();

    void on_pushButton_16_clicked();

    void on_pushButton_17_clicked();

    void on_pushButton_18_clicked();

    void regulation();

    void ramp();

    void advance_to_next_point();

    void setRotation(double rotation);

    void expandMapWithValue(int value);

private:
     JOYINFO joystickInfo;
    Ui::MainWindow *ui;
     void paintEvent(QPaintEvent *event); // Q_DECL_OVERRIDE;
     int updateLaserPicture;
     LaserMeasurement copyOfLaserData;
     std::string ipaddress;
     Robot robot;
     TKobukiData robotdata;
     int datacounter;
     QTimer *timer;
     QJoysticks *instance;

     QPoint mouseMapPoint;
     QPoint cellSize;
     QPoint clickedPos;

     QRect rect;
     QRect miniMap;

     double tickToMeter = 0.000085292090497737556558;
     int left_encoder = 0, right_encoder = 0, left_encoder_last = 0, right_encoder_last = 0;
     float x_pos, y_pos, angle;
     float l_lk, l_rk, l;
     bool start = true;
     int start_left, start_right;
     float start_angle;

     bool mapovanie = false;
     int threshold;
     int map[120][120] = {{0}};
     int finalMap[ROWS][COLS] = {{0}};
     bool boolMap[ROWS][COLS] = {{false}};

     int minimapSize;
     int x, y;
     float mapLength, mapHeight, cellWidth, cellHeight;

     bool zahajenie, cielovyBod, misiaBod, prechodovyBod;
     bool zaznam;

     int increment;
     int actualSpeed = 0;
     int maxSpeed = 300;
     double Pdist, Pangle;
     double distance_error, angle_error, diff;
     bool regulacia;
     int counter;

     std::vector<float> body;
     float bodX, bodY;

     double forwardspeed;//mm/s
     double rotationspeed;//omega/s

public slots:
     //void setUiValues(double robotX,double robotY,double robotFi);
//signals:
//     void uiValuesChanged(double newrobotX, double newrobotY, double newrobotFi); ///toto nema telo


};

#endif // MAINWINDOW_H
