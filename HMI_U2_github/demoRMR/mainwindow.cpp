#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#include <QMouseEvent>
#include <opencv2/opencv.hpp>
#include <QTimer>
#include <QImage>

#include <QDir>
#include <sstream>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow){

    // tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    ipaddress = "127.0.0.1";
    // cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    datacounter = 0;
    // timer = new QTimer(this);
    // connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
    actIndex = -1;
    datacounter = 0;
    x_pos = 0.0;
    y_pos = 0.0;
    angle = 0;
    threshold = 200;
    minimapSize = 200;
    zahajenie = false;
    regulacia = false;
    cielovyBod = false;
    misiaBod = false;
    prechodovyBod = false;
    zaznam = false;
    increment = 5;
    Pdist = 500;
    Pangle = 15;
    positionCounter = 0;
    bodX = 0;
    bodY = 0;
    actualFrameCounter = 0;
    mission_counter = 0;
//    missionLogFile.open("missionlog.txt");

    MainWindow::startup();
    MainWindow::insert_map();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setBrush(Qt::black);
    QPen pero;
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::green);
    QRect geometry = ui->frame_2->geometry();
    QFont font = painter.font();

    rect = ui->frame->geometry();
    rect.translate(0,15);
    miniMap = ui->frame_2->geometry();
    miniMap.setHeight(geometry.width());
    miniMap.setWidth(geometry.width());
    miniMap.translate(0,50);

    mapLength = rect.bottomRight().x() - rect.topLeft().x();
    mapHeight = rect.bottomRight().y() - rect.topLeft().y();
    cellWidth = mapLength/COLS;
    cellHeight = mapHeight/ROWS;
    font.setPixelSize(80);
    painter.setFont(font);

    painter.drawRect(rect);

    QRect hlavicka(0, 0, 0.5*cellHeight, 1.5*cellWidth);

//    hlavicka.translate(int((x_pos*10*cellWidth) + (5 * cellWidth) + rect.topLeft().x()), int((41 * cellHeight) - (y_pos*10*cellHeight) + rect.topLeft().y())-20);

    if(actIndex > -1)
    {
        if (zahajenie){
            std::stringstream ss;
            ss << "D:/School/Ing/1st year/LS/HMI/Zadanie/Cizmar_Cservenka_HMI_2023/HMI_U2_github/demoRMR/frames/image_" << actualFrameCounter << ".jpg";
            std::string filename = ss.str();
            cv::imwrite(filename, frame[actIndex]);
            actualFrameCounter++;
        }

        //std::cout<<actIndex<<std::endl;
        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );

        for (int i = 0; i < ROWS; i++){
            for (int j = 0; j < COLS; j++){

                if (finalMap[i][j] == 1){
                    painter.fillRect(int((j*cellWidth)+rect.topLeft().x()), int((i*cellHeight)+rect.topLeft().y()),int(cellWidth), int(cellHeight), QColor(0, 255, 0, 175));

                } else if(finalMap[i][j] == 3){
                    if (boolMap[i][j] == true){
                        painter.fillRect(int((j*cellWidth)+rect.topLeft().x()), int((i*cellHeight)+rect.topLeft().y()),int(cellWidth), int(cellHeight), QColor(255, 255, 255, 255));
                    }
                    else {
                        painter.fillRect(int((j*cellWidth)+rect.topLeft().x()), int((i*cellHeight)+rect.topLeft().y()),int(cellWidth), int(cellHeight), QColor(0, 0, 255, 175));
                    }
                } else if(finalMap[i][j] == 5){
                    painter.fillRect(int((j*cellWidth)+rect.topLeft().x()), int((i*cellHeight)+rect.topLeft().y()),int(cellWidth), int(cellHeight), QColor(255, 0, 0, 175));

                } else if(finalMap[i][j] == 4) {
                    painter.fillRect(int((j*cellWidth)+rect.topLeft().x()), int((i*cellHeight)+rect.topLeft().y()),int(cellWidth), int(cellHeight), QColor(255, 255, 0, 175));

                } else if (finalMap[i][j] == 8){
                    painter.fillRect(int((j*cellWidth)+rect.topLeft().x()), int((i*cellHeight)+rect.topLeft().y()), int(cellWidth), int(cellHeight), QColor(255, 255, 255, 255));
                }
            }
        }

        pero.setStyle(Qt::SolidLine);
        pero.setWidth(6);
        pero.setBrush(Qt::red);
        painter.setPen(pero);
        painter.drawEllipse(int((x_pos*10*cellWidth) + (5 * cellWidth) + rect.topLeft().x())-(1.75*cellWidth), int((41 * cellHeight) - (y_pos*10*cellHeight) + rect.topLeft().y())-(1.75*cellHeight), 3.5*cellWidth, 3.5*cellHeight);
        painter.drawImage(miniMap, image.rgbSwapped());

        for(int k = 0; k < copyOfLaserData.numberOfScans/*360*/; k++)
        {
            int D = copyOfLaserData.Data[k].scanDistance - 175;
            if (!alarm && D < 50) alarm = true;
        }

        if (alarm){
            showTextString = "! MISIA !\n! ZLYHALA !";
            MainWindow::paintTextOnScreen(&painter, &rect);
            MainWindow::on_pushButton_4_clicked();

        } else if (executingMission){
            showTextString = "VYKONAVAM\nMISIU";
            MainWindow::paintTextOnScreen(&painter, &rect);

        } else if (boolMissionExecuted){
            showTextString = "! MISIA ! \n! SPLNENA !";
            MainWindow::paintTextOnScreen(&painter, &rect);
        }


        QPoint pointik = hlavicka.center();
        pointik.setX(int((x_pos*10*cellWidth) + (5 * cellWidth) + rect.topLeft().x()));
        pointik.setY(int((41 * cellHeight) - (y_pos*10*cellHeight) + rect.topLeft().y()));
        painter.translate(pointik);
        painter.rotate(-angle-90);
        hlavicka.translate(-hlavicka.width()/2, 0);
        painter.fillRect(hlavicka, QColor(255, 0, 0, 255));
    }
}

int MainWindow::processThisRobot(TKobukiData robotdata)
{
    MainWindow::getOdometry(robotdata);

//    if (mapovanie){
//        MainWindow::mapping();
//    }
    if (zahajenie && regulacia){
        frames.push_back(frame[actIndex]);
        MainWindow::regulation();
    }



    return 0;
}

int MainWindow::processThisLidar(LaserMeasurement laserData)
{
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    updateLaserPicture=1;
    update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

    return 0;
}

int MainWindow::processThisCamera(cv::Mat cameraData)
{
    cameraData.copyTo(frame[(actIndex+1)%3]);
    actIndex=(actIndex+1)%3;
    updateLaserPicture=1;
    return 0;
}

void MainWindow::startup() //start button
{
    forwardspeed=0;
    rotationspeed=0;
//    std::cout << "frames: " << frames.size() << std::endl;
    robot.setLaserParameters("127.0.0.1",52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar,this,std::placeholders::_1));
    robot.setRobotParameters("127.0.0.1",53000,5300,std::bind(&MainWindow::processThisRobot,this,std::placeholders::_1));
    robot.setCameraParameters("http://127.0.0.1:8889/stream.mjpg",std::bind(&MainWindow::processThisCamera,this,std::placeholders::_1));
    robot.robotStart();

    instance = QJoysticks::getInstance();

    connect(
        instance, &QJoysticks::axisChanged,
        [this]( const int js, const int axis, const qreal value) { if(/*js==0 &&*/ axis==1){forwardspeed=-value*300;}
            if(/*js==0 &&*/ axis==0){rotationspeed=-value*(3.14159/2.0);}}
    );
}

void MainWindow::on_pushButton_4_clicked() //stop
{
    robot.setTranslationSpeed(0);
    robot.setRotationSpeed(0);
    mapovanie = false;
    zahajenie = false;
    cielovyBod = false;
    misiaBod = false;
    prechodovyBod = false;
    zaznam = false;
    regulacia = false;
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {

        clickedPos = event->pos();
        mouseMapPoint.setX(clickedPos.x() - rect.topLeft().x());
        mouseMapPoint.setY(clickedPos.y() - rect.topLeft().y());

        MainWindow::getIndex();

        if(rect.contains(clickedPos.x(),clickedPos.y())){
//            std::cout << "clicked: " << mouseMapPoint.x() << " " << mouseMapPoint.y() << std::endl;
            std::cout << "array index: " << x << " " << y << std::endl;
        }
        if(rect.contains(clickedPos.x(),clickedPos.y()) && prechodovyBod && finalMap[y][x] == 0){
            finalMap[y][x] = 3;
            body.push_back((x-5)/10.0);
            body.push_back((41-y)/10.0);
//            std::cout << " pushback: " << (x-5)/10.0 << " " << (41-y)/10.0 << std::endl;

        } else if(rect.contains(clickedPos.x(),clickedPos.y()) && misiaBod && finalMap[y][x] == 0){
            finalMap[y][x] = 4;
            MainWindow::expandMapWithValue(4);

            body.push_back((x-5)/10.0);
            body.push_back((41-y)/10.0);
//            std::cout << " pushback: " << (x-5)/10.0 << " " << (41-y)/10.0 << std::endl;

        } else if(rect.contains(clickedPos.x(),clickedPos.y()) && cielovyBod && finalMap[y][x] == 0){
            finalMap[y][x] = 5;
            MainWindow::expandMapWithValue(5);

            body.push_back((x-5)/10.0);
            body.push_back((41-y)/10.0);
//            std::cout << " pushback: " << (x-5)/10.0 << " " << (41-y)/10.0 << std::endl;
        }

    } else if (event->button() == Qt::RightButton) {

//        QPoint clickPos = event->pos();
//        std::cout << "odometry: " << x_pos << " " << y_pos << std::endl
    }

    // Call the base class implementation to handle any other cases
    QWidget::mousePressEvent(event);
}

void MainWindow::advance_to_next_point()
{
    if (boolMap[y][x] == false)        boolMap[y+1][x] = true;
    if (boolMap[y+1][x] == false)      boolMap[y+1][x] = true;
    if (boolMap[y-1][x] == false)      boolMap[y-1][x] = true;
    if (boolMap[y][x+1] == false)      boolMap[y][x+1] = true;
    if (boolMap[y][x-1] == false)      boolMap[y][x-1] = true;
    if (boolMap[y-1][x-1] == false)    boolMap[y-1][x-1] = true;
    if (boolMap[y+1][x+1] == false)    boolMap[y+1][x+1] = true;
    if (boolMap[y-1][x+1] == false)    boolMap[y-1][x+1] = true;
    if (boolMap[y+1][x-1] == false)    boolMap[y+1][x-1] = true;
    bodX = body.at(positionCounter*2);
    bodY = body.at((positionCounter*2)+1);
    positionCounter++;
}

void MainWindow::expandMapWithValue(int value)
{
    if (finalMap[y+1][x] == 0)      finalMap[y+1][x] = value;
    if (finalMap[y-1][x] == 0)      finalMap[y-1][x] = value;
    if (finalMap[y][x+1] == 0)      finalMap[y][x+1] = value;
    if (finalMap[y][x-1] == 0)      finalMap[y][x-1] = value;
    if (finalMap[y-1][x-1] == 0)    finalMap[y-1][x-1] = value;
    if (finalMap[y+1][x+1] == 0)    finalMap[y+1][x+1] = value;
    if (finalMap[y-1][x+1] == 0)    finalMap[y-1][x+1] = value;
    if (finalMap[y+1][x-1] == 0)    finalMap[y+1][x-1] = value;
}

void MainWindow::regulation()
{
    // vypocet regulacie a regulacnej odchylky
    x = int((x_pos*10) + 5);
    y = int(41 -(y_pos*10));

    if (finalMap[y][x] == 0 || finalMap[y][x] == 3 ) finalMap[y][x] = 8;

    std::cout << "      som na: " << x << " " << y << std::endl;
    std::cout << " moja poloha: " << x_pos << " " << y_pos << std::endl;
    std::cout << "     idem na: " << bodX << " " << bodY << std::endl;

    angle_error = double(atan2((bodY-y_pos),(bodX-x_pos)));
    angle_error *= 180/PI;
    angle_error -= angle;

    if (angle_error > 180){
        angle_error -= 360;
    }

    double rot = Pangle * (angle_error * M_PI/180);

    if (rot > M_PI/2){
        rot = M_PI/2;

    } else if (rot < -M_PI/2){
        rot = -M_PI/2;
    }

    distance_error = std::sqrt(pow((x_pos - bodX),2) + pow((y_pos - bodY),2));
//    std::cout<<"dist error: "<<distance_error<<" angle error: "<<angle_error<<" rot: "<< rot << std::endl;
//    std::cout<<" x pos: "<<x_pos<<" y pos: "<<y_pos<<endl;


    if (distance_error < 0.02 && finalMap[y][x] == 5) {

        MainWindow::missionExecuted();
        std::cout << " !!! MISIA USPESNE VYKONANA !!!" << std::endl;

    } else if (distance_error < 0.02 && finalMap[y][x] == 4) {

        MainWindow::executeOrder();

    } else if (angle_error < -8 && distance_error >= 0.02){

        MainWindow::setRotation(rot);

    } else if (angle_error > 8 && distance_error >= 0.02){

        MainWindow::setRotation(rot);

    } else if (angle_error <= 8 && angle_error >= 4){

        MainWindow::setRotation(2*rot);

    } else if (angle_error >= -8 && angle_error <= -4){

        MainWindow::setRotation(2*rot);

    } else if(distance_error >= 0.02){

        MainWindow::ramp();

    } else if(distance_error < 0.02){

        MainWindow::advance_to_next_point();
    }
}

void MainWindow::executeOrder()
{

    if(mission_counter < 500){
        executingMission = true;
        MainWindow::setRotation(M_PI/4);
        mission_counter++;

    } else {
        executingMission = false;
        MainWindow::advance_to_next_point();
    }
}

void MainWindow::setRotation(double rotation)
{
    actualSpeed = 0;
    robot.setRotationSpeed(rotation);
}

void MainWindow::ramp()
{
    maxSpeed = int(distance_error * Pdist);
    if (maxSpeed > 300)
        maxSpeed = 300;

    if (actualSpeed < maxSpeed){
        actualSpeed += increment;
        robot.setTranslationSpeed(actualSpeed);

    } else if(actualSpeed >= maxSpeed){
        actualSpeed -= increment;
        robot.setTranslationSpeed(actualSpeed);
    }
    else{
        actualSpeed = maxSpeed;
        robot.setTranslationSpeed(maxSpeed);
    }
}

void MainWindow::missionExecuted()
{
    MainWindow::on_pushButton_4_clicked(); // stop
    MainWindow::on_pushButton_14_clicked(); // Uloz mapu do txt
    boolMissionExecuted = true;
//    missionLogFile.close();
    MainWindow::createVideo();
}

void MainWindow::createVideo()
{
    QString dirPath =        "D:/School/Ing/1st year/LS/HMI/Zadanie/Cizmar_Cservenka_HMI_2023/HMI_U2_github/demoRMR/frames"; // Replace with your image directory path
    QString outputFilePath = "D:/School/Ing/1st year/LS/HMI/Zadanie/Cizmar_Cservenka_HMI_2023/HMI_U2_github/demoRMR/video.avi"; // Replace with your desired output video file path

    int width = 863; // Replace with your desired video frame width
    int height = 480; // Replace with your desired video frame height

    // Read images from directory
    QStringList filters;
    filters << "*.jpg"; // Replace with the file extensions of your saved images
    QDir dir(dirPath);
    QStringList imagePaths = dir.entryList(filters, QDir::Files);
    int numFrames = imagePaths.size();
    std::vector<cv::Mat> video;
    for (const QString& imagePath : imagePaths) {
        QString filePath = dirPath + "/" + imagePath;
        cv::Mat image = cv::imread(filePath.toStdString());
        if (image.empty()) {
            std::cout << "Could not read image" << std::endl;
            return;
        }
        cv::resize(image, image, cv::Size(width, height)); // Resize image to desired video frame size
        video.push_back(image);
    }

    // Create video writer
    cv::VideoWriter videoWriter(outputFilePath.toStdString(), cv::VideoWriter::fourcc('M','J','P','G'), 5, cv::Size(width, height), true);
    if (!videoWriter.isOpened()) {
        std::cout << "Error opening video file for writing" << std::endl;
        return;
    }

    // Write frames to video
    for (int i = 0; i < video.size(); i++) {
        videoWriter.write(video[i]);
    }

    // Release video writer
    videoWriter.release();
    std::cout << " video ulozene " << std::endl;
}

void MainWindow::getIndex()
{
    x = int(mouseMapPoint.x()/cellWidth);
    y = int(mouseMapPoint.y()/cellHeight);
}

void MainWindow::getOdometry(TKobukiData robotdata)
{
    //Pociatocna poloha robota
    if(start){
        start_left = robotdata.EncoderLeft;
        start_right = robotdata.EncoderRight;
        start_angle = robotdata.GyroAngle/100.0;
        start = false;
    }
    // Nacita pociatocne hodnoty encoderov
    left_encoder = robotdata.EncoderLeft - start_left;
    right_encoder = robotdata.EncoderRight - start_right;

    // Ochrana proti preteceniu
    if(left_encoder_last - left_encoder > 50000){
        left_encoder_last = - (65535 - left_encoder_last);
    }
    else if(left_encoder_last - left_encoder < -60000){
        left_encoder_last = 65535 + left_encoder_last;
    }
    if(right_encoder_last - right_encoder > 60000){
        right_encoder_last = - (65535 - right_encoder_last);
    }
    else if(right_encoder_last - right_encoder < -60000){
        right_encoder_last = 65535 + right_encoder_last;
    }

    // Vypocet uhlu otocenia robota a nastavenia pociatocneho uhla
    angle = robotdata.GyroAngle / 100.0 - start_angle;

    if(angle > 180.0){
        angle = angle - 360.0;
    }
    else if(angle < -180.0){
        angle = 360.0 + angle;
    }

    l_lk = tickToMeter * (left_encoder - left_encoder_last);
    l_rk = tickToMeter * (right_encoder - right_encoder_last);
    left_encoder_last = left_encoder;
    right_encoder_last = right_encoder;
    l = (l_lk + l_rk)/2;

    x_pos = x_pos + (l * cos(angle*3.14159/180.0));
    y_pos = y_pos + (l * sin(angle*3.14159/180.0));
}

void MainWindow::mapping()
{
    for(int k = 0; k < copyOfLaserData.numberOfScans/*360*/; k++)
    {
        int d_i = copyOfLaserData.Data[k].scanDistance; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
        if(!(d_i <= 700 && d_i >= 640) ){
            if (d_i <= 3000 && d_i >= 130){
                int x_gi = int((x_pos*1000 + d_i * cos(angle*PI/180+(360-copyOfLaserData.Data[k].scanAngle)*PI/180)));
                int y_gi = int((y_pos*1000 + d_i * sin(angle*PI/180+(360-copyOfLaserData.Data[k].scanAngle)*PI/180)));
                int map_x = int(60 + x_gi/100);
                int map_y = int(60 + y_gi/100);

                map[map_x][map_y] += 1;
            }
        }
    }
}

void MainWindow::on_pushButton_11_clicked() // mapovanie
{
    mapovanie = true;
    std::cout<<" mapovanie" << std::endl;
}

void MainWindow::on_pushButton_12_clicked() // uloz mapu
{
    mapovanie = false;

    ofstream outfile("mapovanie.txt");

    if (outfile.is_open()){

        for(int i = 0; i < 120; i++){
            for(int j = 0; j < 120; j++){
                if (map[i][j] == 0){
                    outfile << ' ';
                }
                else if(map[i][j] >= threshold){
                    outfile << '1';

                } else {
                    outfile << ' ';
                }
            }
            outfile << '\n';
        }

    } else {
        cout << " Unable to open file";
    }
    outfile.close();
    std::cout<<" mapa zapisana" << std::endl;;
}

void MainWindow::insert_map() // insert mapu
{
    std::cout << " ~~~~ vkladam mapu do pola ~~~~ " << std::endl;

    char c;
    int i = 0, j = 0;
    ifstream infile("mapa.txt");

    if (!infile.is_open()){
        std::cout << "nepodarilo sa otvorit insert mapu" << std::endl;

    } else {
        while (infile.get(c)) {
            if (c == '\n') {
                i++;
                j = 0;

            } else if(c == '1'){
                if (i >= 0 && i < ROWS && j >= 0 && j < COLS){
                    finalMap[i][j] = 1;
                    j++;
                }

            } else {
                j++;
            }
        }
    }

    std::cout << " ~~~~ mapa uspesne vlozena ~~~~ " << std::endl;
    infile.close();
}

void MainWindow::on_pushButton_14_clicked() // Uloz mapu do txt
{
    std::cout << " ~~~~ Ukladam mapu ~~~~ " << std::endl;
    ofstream outfile("maplog.txt");
    if(!outfile.is_open()){
        std::cout << " nepodarilo sa otvorit maplog " << std::endl;

    } else {
        for(int i = 0; i < ROWS; i++)
        {
            for(int j = 0; j < COLS; j++)
            {
                if (finalMap[i][j] != 0){
                    outfile << finalMap[i][j];
                } else {
                    outfile << ' ';
                }
            }
            outfile << '\n';
        }
        std::cout << " ~~~~ Mapa uspesne ulozena  ~~~~ " << std::endl;
    }
    outfile.close();
}

void MainWindow::on_pushButton_15_clicked() // Prechodovy bod
{
    if (!zahajenie){
        cielovyBod = false;
        misiaBod = false;
        prechodovyBod = true;
    }
    std::cout << " ~~~~~~~~~~ AKTIVNY PRECHODOVY BOD ~~~~~~~~~~ " << std::endl;
}

void MainWindow::on_pushButton_16_clicked() // Misia bod
{
    if (!zahajenie){
        cielovyBod = false;
        misiaBod = true;
        prechodovyBod = false;
    }
    std::cout << " ~~~~~~~~~~ AKTIVNY BOD MISIE ~~~~~~~~~~ " << std::endl;
}

void MainWindow::on_pushButton_18_clicked() // Cielovy bod
{
    if (!zahajenie){
        cielovyBod = true;
        misiaBod = false;
        prechodovyBod = false;
    }
    std::cout << " ~~~~~~~~~~ AKTIVNY CIELOVY BOD ~~~~~~~~~~ " << std::endl;
}

void MainWindow::on_pushButton_17_clicked() // Zahajenie misie
{
    zahajenie = true;
    cielovyBod = false;
    misiaBod = false;
    prechodovyBod = false;
    regulacia = true;
    if (!alarm ) {
        std::cout << " ~~~~~~~~~~ ZAHAJUJEM VYKONANIE MISIE ~~~~~~~~~~ " << std::endl;
    }
}

void MainWindow::paintTextOnScreen(QPainter* painter, QRect *rect){

    QRect alarmRect(0, 0, rect->bottomRight().x()-200, rect->bottomRight().y()-200);
    alarmRect.translate((rect->bottomRight().x()/2)-(rect->bottomRight().x()/3), (rect->bottomRight().y()/2 - (rect->bottomRight().y()/6)) );
    painter->fillRect(rect->topLeft().x()+100, rect->topLeft().y()+100, rect->bottomRight().x()-200, rect->bottomRight().y()-300, QColor(192, 192, 192, 175));
    painter->drawText(alarmRect, Qt::TextWordWrap, showTextString);
}

void MainWindow::on_Preplanovat_clicked() // preplanovat
{








}

