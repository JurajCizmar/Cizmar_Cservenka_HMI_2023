#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>

/// HMI Juraj Čižmár + Béla Cservenka LS 2023


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    ipaddress = "127.0.0.1";//192.168.1.11 toto je na niektory realny robot.. na lokal budete davat "127.0.0.1"
  //  cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    datacounter = 0;
  //  timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
    actIndex = -1;
    useCamera1 = true;
    useStopSwitch=true;
    f = 681.743;
    ZD = -145;
    Z = 210;
    YD = 115;
    alpha = 0;
    datacounter = 0;
    max = 600;
    mid = 400;
    min = 200; 
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    ///prekreslujem obrazovku len vtedy, ked viem ze mam nove data. paintevent sa
    /// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
    painter.setBrush(Qt::black);//cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
    QPen pero;
    QPen pen;
    //pero.setStyle(Qt::SolidLine);//styl pera - plna ciara
    //pero.setWidth(0);//hrubka pera -3pixely
    pero.setColor(Qt::green);//farba je zelena
    QRect rect;
    rect = ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit
    rect.translate(0,15);
    QRect miniMap(0, 0, minimapSize, minimapSize);
    painter.drawRect(rect);
    QFont font = painter.font();
    font.setPixelSize(70);
    painter.setFont(font);

    rectHeight = rect.bottomLeft().y() - rect.topLeft().y();
    rectWidth = rect.topRight().x() - rect.topLeft().x();

    if(rectHeight > rectWidth)
    {
           if(rectWidth/rectHeight > 1.777)
           {
                rect.setWidth(rectHeight*1.777);
           }
           else if(rectWidth/rectHeight < 1.777)
           {
               rect.setHeight(rectWidth/1.777);
           }
    } else if(rectHeight < rectWidth)
    {
        if(rectWidth/rectHeight > 1.777)
        {
            rect.setWidth(rectHeight*1.777);
        }
        else if(rectWidth/rectHeight < 1.777)
        {
            rect.setHeight(rectWidth/1.777);
        }
    }

    if(useCamera1 == true && actIndex > -1)/// ak zobrazujem data z kamery a aspon niektory frame vo vectore je naplneny
    {
        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888);
        QPainter image_painter(&image);
        if(updateLaserPicture == 1) ///ak mam nove data z lidaru
        {
            updateLaserPicture = 0;

            for(int k = 0; k < copyOfLaserData.numberOfScans/*360*/; k++)
            {
                D = copyOfLaserData.Data[k].scanDistance;
                scanAngle = 360.0-copyOfLaserData.Data[k].scanAngle;

                X = D * cos(scanAngle * PI/180.0);
                Y = D * sin(scanAngle * PI/180.0);
                float xobr = (frame[actIndex].cols/2.0) - ((f * Y)/(X + ZD));
                float yobr = (frame[actIndex].rows/2.0) + ((f * (-Z + YD))/(X + ZD));

                D -= 200; //200 pre simulator, 150 pre realny

                if(scanAngle <= 32.0 || scanAngle >= (360.0-32.0))
                {
                    alpha = int((255-(D/16)));
                    image_painter.fillRect((int)xobr-10, (int)yobr-20, 20, 40, QColor(0, 50, 255 , alpha));
                }
            }
        }
        minimapSize = int(rect.bottomRight().x()/4.5);
        miniMap.translate(rect.bottomRight().x()-minimapSize, rect.bottomRight().y()-minimapSize);
        painter.drawImage(rect, image.rgbSwapped());
        painter.setBrush(QColor(192, 192, 192, 127));
        painter.drawRect(miniMap);

        painter.setBrush(Qt::transparent);
        pen.setStyle(Qt::SolidLine);
        pen.setWidth(6);
        pen.setBrush(Qt::red);
        painter.setPen(pen);
        painter.drawEllipse(QPoint(miniMap.bottomRight().x()-(minimapSize/2), miniMap.bottomRight().y()-(minimapSize/2)), 2, 2);

        painter.drawEllipse(rect.bottomRight().x()/2 -21, rect.bottomRight().y()-96, 42, 42);
        painter.fillRect(rect.bottomRight().x()/2-3, rect.bottomRight().y()-96, 6, 20, Qt::red);


        pen.setBrush(Qt::blue);
        pen.setWidth(3);
        painter.setPen(pen);


        for(int k = 0; k < copyOfLaserData.numberOfScans/*360*/; k++)
        {
            int dist = copyOfLaserData.Data[k].scanDistance/50; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
            int xp = miniMap.width()-(miniMap.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0)) + miniMap.topLeft().x(); //prepocet do obrazovky
            int yp = miniMap.height()-(miniMap.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+miniMap.topLeft().y();//prepocet do obrazovky
            if(miniMap.contains(xp,yp)){//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                painter.drawEllipse(QPoint(xp, yp),2,2);
            }
        }

        pen.setWidth(6);
        painter.setPen(pen);

        for(int k = 0; k < copyOfLaserData.numberOfScans/*360*/; k++){
            D = copyOfLaserData.Data[k].scanDistance;
            D -= 150;
            scanAngle = 360.0-copyOfLaserData.Data[k].scanAngle;

            if (scanAngle > 22.5 && scanAngle <= 67.5){

                if (D <= max && D > mid){
                    draw_robot_arc(&painter, &pen, &rect, 120, 30, 1);

                } else if(D <= mid && D > min){
                    draw_robot_arc(&painter, &pen, &rect, 120, 30, 2);

                } else if(D <= min && D >= 50){
                    draw_robot_arc(&painter, &pen, &rect, 120, 30, 3);
                }

            } else if (scanAngle > 67.5 && scanAngle <= 112.5){

                if (D <= max && D > mid){
                    draw_robot_arc(&painter, &pen, &rect, 165, 30, 1);

                } else if(D <= mid && D > min){
                    draw_robot_arc(&painter, &pen, &rect, 165, 30, 2);

                } else if(D <= min && D >= 50){
                    draw_robot_arc(&painter, &pen, &rect, 165, 30, 3);
                }

            } else if (scanAngle > 112.5 && scanAngle <= 157.5){

                if (D <= max && D > mid){
                    draw_robot_arc(&painter, &pen, &rect, 210, 30, 1);

                } else if(D <= mid && D > min){
                    draw_robot_arc(&painter, &pen, &rect, 210, 30, 2);

                } else if(D <= min && D >= 50){
                    draw_robot_arc(&painter, &pen, &rect, 210, 30, 3);
                }

            } else if (scanAngle > 157.5 && scanAngle <= 202.5){

                if (D <= max && D > mid){
                    draw_robot_arc(&painter, &pen, &rect, 255, 30, 1);

                } else if(D <= mid && D > min){
                    draw_robot_arc(&painter, &pen, &rect, 255, 30, 2);

                } else if(D <= min && D >= 50){
                    draw_robot_arc(&painter, &pen, &rect, 255, 30, 3);
                }

            } else if (scanAngle > 202.5 && scanAngle <= 247.5){

                if (D <= max && D > mid){
                    draw_robot_arc(&painter, &pen, &rect, 300, 30, 1);

                } else if(D <= mid && D > min){
                    draw_robot_arc(&painter, &pen, &rect, 300, 30, 2);

                } else if(D <= min && D >= 50){
                    draw_robot_arc(&painter, &pen, &rect, 300, 30, 3);
                }

            } else if (scanAngle > 247.5 && scanAngle <= 292.5){

                if (D <= max && D > mid){
                    draw_robot_arc(&painter, &pen, &rect, 345, 30, 1);

                } else if(D <= mid && D > min){
                    draw_robot_arc(&painter, &pen, &rect, 345, 30, 2);

                } else if(D <= min && D >= 50){
                    draw_robot_arc(&painter, &pen, &rect, 345, 30, 3);
                }

            } else if (scanAngle > 292.5 && scanAngle <= 337.5){

                if (D <= max && D > mid){
                    draw_robot_arc(&painter, &pen, &rect, 30, 30, 1);

                } else if(D <= mid && D > min){
                    draw_robot_arc(&painter, &pen, &rect, 30, 30, 2);

                } else if(D <= min && D >= 50){
                    draw_robot_arc(&painter, &pen, &rect, 30, 30, 3);
                }

            } else {

                if (D <= max && D > mid){
                    draw_robot_arc(&painter, &pen, &rect, 75, 30, 1);

                } else if(D <= mid && D > min){
                    draw_robot_arc(&painter, &pen, &rect, 75, 30, 2);

                } else if(D <= min && D >= 50){
                    draw_robot_arc(&painter, &pen, &rect, 75, 30, 3);
                }
            }
        }

    }
    else
    {
        if(updateLaserPicture == 1) ///ak mam nove data z lidaru
        {
            updateLaserPicture = 0;
            pero.setColor(Qt::green);
            painter.setPen(pero);
            //teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
            //std::cout<<copyOfLaserData.numberOfScans<<std::endl;
            for(int k = 0; k < copyOfLaserData.numberOfScans/*360*/; k++)
            {
                int dist = copyOfLaserData.Data[k].scanDistance/15; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
                int xp = rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x(); //prepocet do obrazovky
                int yp = rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();//prepocet do obrazovky
                if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                    painter.drawEllipse(QPoint(xp, yp),2,2);
            }
        }
    }
    /*if(updateSkeletonPicture == 1 )
    {
        painter.setPen(Qt::red);
        for(int i = 0; i < 75; i++)
        {
            int xp=rect.width()-rect.width() * skeleJoints.joints[i].x+rect.topLeft().x();
            int yp= (rect.height() *skeleJoints.joints[i].y)+rect.topLeft().y();
            if(rect.contains(xp,yp))
                painter.drawEllipse(QPoint(xp, yp),2,2);
        }
    }*/
}


/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked
void  MainWindow::setUiValues(double robotX, double robotY, double robotFi)
{
//     ui->lineEdit_2->setText(QString::number(robotX));
//     ui->lineEdit_3->setText(QString::number(robotY));
//     ui->lineEdit_4->setText(QString::number(robotFi));
}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{
    ///tu mozete robit s datami z robota
    /// ale nic vypoctovo narocne - to iste vlakno ktore cita data z robota
    ///teraz tu posielam rychlosti na zaklade toho co setne joystick a vypisujeme data z robota(kazdy 5ty krat. ale mozete skusit aj castejsie). vyratajte si polohu. a vypiste spravnu
    /// tuto joystick cast mozete vklude vymazat,alebo znasilnit na vas regulator alebo ake mate pohnutky... kazdopadne, aktualne to blokuje gombiky cize tak

///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

    if(useStopSwitch == true){
        skeletonTracker();
    }

    if(datacounter % 5  == 0)
    {

        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
                //ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
                //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
                //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
                /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
                /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
                /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        ///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
        //emit uiValuesChanged(0,0,0);
        ///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }
    datacounter++;

    if (rampa) MainWindow::ramp();
    return 0;

}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z lidaru
int MainWindow::processThisLidar(LaserMeasurement laserData)
{
    memcpy( &copyOfLaserData, &laserData, sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    updateLaserPicture = 1;
    update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

    return 0;
}

///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z kamery
int MainWindow::processThisCamera(cv::Mat cameraData)
{
    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    actIndex = (actIndex+1)%3;//aktualizujem kde je nova fotka
    updateLaserPicture=1;
    return 0;
}

///toto je calback na data zo skeleton trackera, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z trackera
int MainWindow::processThisSkeleton(skeleton skeledata)
{

    memcpy(&skeleJoints,&skeledata,sizeof(skeleton));

    updateSkeletonPicture=1;
    return 0;
}

void MainWindow::on_pushButton_9_clicked() //start button
{

    forwardspeed=0;
    rotationspeed=0;
    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));

    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robot.setLaserParameters(ipaddress,52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar,this,std::placeholders::_1));
    robot.setRobotParameters(ipaddress,53000,5300,std::bind(&MainWindow::processThisRobot,this,std::placeholders::_1));
    //---simulator ma port 8889, realny robot 8000
    robot.setCameraParameters("http://"+ipaddress+":8889/stream.mjpg",std::bind(&MainWindow::processThisCamera,this,std::placeholders::_1));
    robot.setSkeletonParameters("127.0.0.1",23432,23432,std::bind(&MainWindow::processThisSkeleton,this,std::placeholders::_1));
    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robot.robotStart();


    //ziskanie joystickov
    instance = QJoysticks::getInstance();


    /// prepojenie joysticku s jeho callbackom... zas cez lambdu. neviem ci som to niekde spominal,ale lambdy su super. okrem toho mam este rad ternarne operatory a spolocneske hry ale to tiez nikoho nezaujima
    /// co vas vlastne zaujima? citanie komentov asi nie, inak by ste citali toto a ze tu je blbosti
    connect(
        instance, &QJoysticks::axisChanged,
        [this]( const int js, const int axis, const qreal value) { if(/*js==0 &&*/ axis ==1){forwardspeed = -value*300;}
            if(/*js==0 &&*/ axis == 0){rotationspeed = -value*(3.14159/2.0);}}
    );
}

void MainWindow::on_pushButton_4_clicked() //stop
{   
    if(useStopSwitch==true)
    {
        useStopSwitch=false;

        ui->pushButton_4->setText("STOPPED");
        ui->pushButton_4->setStyleSheet("background-color: red");
        robot.setTranslationSpeed(0);
        robot.setRotationSpeed(0);
        actualSpeed=0;
        rampa = false;
    }
    else
    {
        useStopSwitch=true;

        ui->pushButton_4->setText("RUNNING");
        ui->pushButton_4->setStyleSheet("background-color: yellow");
        robot.setTranslationSpeed(0);
        robot.setRotationSpeed(0);
        actualSpeed=0;
        rampa = false;
    }
}

void MainWindow::skeletonTracker()
{
    rightIndexTipX = skeleJoints.joints[jointnames(right_index_tip)].x;
    rightIndexTipY = skeleJoints.joints[jointnames(right_index_tip)].y;
    leftIndexTipX = skeleJoints.joints[jointnames(left_index_tip)].x;
    leftIndexTipY = skeleJoints.joints[jointnames(left_index_tip)].y;
    rightWristX = skeleJoints.joints[jointnames(right_wrist)].x;
    rightWristY = skeleJoints.joints[jointnames(right_wrist)].y;
    leftWristX = skeleJoints.joints[jointnames(left_wrist)].x;
    leftWristY = skeleJoints.joints[jointnames(left_wrist)].y;
    rightMiddleTipX = skeleJoints.joints[jointnames(right_middle_tip)].x;
    rightMiddleTipY = skeleJoints.joints[jointnames(right_middle_tip)].y;
    leftMiddleTipX = skeleJoints.joints[jointnames(left_middle_tip)].x;
    leftMiddleTipY = skeleJoints.joints[jointnames(left_middle_tip)].y;
    rightPinkyTipX = skeleJoints.joints[jointnames(right_pink_tip)].x;
    rightPinkyTipY = skeleJoints.joints[jointnames(right_pink_tip)].y;

    rightIndexDistance = sqrt(pow((rightIndexTipX - rightWristX),2) + pow((rightIndexTipY - rightWristY),2));
    leftIndexDistance = sqrt(pow((leftIndexTipX - leftWristX),2) + pow((leftIndexTipY - leftWristY),2));
    rightMiddleDistance = sqrt(pow((rightMiddleTipX - rightWristX),2) + pow((rightMiddleTipY - rightWristY),2));
    leftMiddleDistance = sqrt(pow((leftMiddleTipX - leftWristX),2) + pow((leftMiddleTipY - leftWristY),2));
    rightPinkyDistance = sqrt(pow((rightPinkyTipX - rightWristX),2) + pow((rightPinkyTipY - rightWristY),2));

    //std::cout << "right index distace " <<rightIndexDistance<<endl;
    //std::cout << "pinky distace " <<rightPinkyDistance<<endl;

    //riadenie gestami

    //dopredu = oba ukazovaky su vystrete
    if((rightIndexDistance >= 0.4) && (leftIndexDistance >= 0.4) &&
       (leftMiddleDistance < 0.4) && (rightMiddleDistance < 0.4))
    {
        MainWindow::ramp();
    }
    //toc sa doprava = pravy ukazovak hore a lavy dole
    else if((rightIndexDistance >= 0.4) && (leftIndexDistance < 0.4) &&
            (leftMiddleDistance < 0.4) && (rightMiddleDistance < 0.4))
    {
        robot.setRotationSpeed(-3.14159/2);
    }
    //toc sa dolava = lavy ukazovak hore a pravy dole
    else if((leftIndexDistance >= 0.4) && (rightIndexDistance < 0.4) &&
            (rightMiddleDistance < 0.4) && (leftMiddleDistance < 0.4))
    {
        robot.setRotationSpeed(3.14159/2);
    }
    //toc sa doprava po kruznici = pravy ukazovak a pravy stredny prst
    else if((rightIndexDistance >= 0.4) && (rightMiddleDistance >= 0.4) &&
            (leftIndexDistance < 0.4) && (leftMiddleDistance < 0.4))
    {
        robot.setArcSpeed(100,-100);
    }
    //toc sa dolava po kruznici = lavy ukazovak a lavy stredny prst
    else if((leftIndexDistance >= 0.4) && (leftMiddleDistance >= 0.4) &&
            (rightIndexDistance < 0.4) && (rightMiddleDistance < 0.4))
    {
        robot.setArcSpeed(100, 100);
    }
    //dozadu
    else if((rightPinkyDistance >= 0.4) && (rightIndexDistance < 0.4) &&
            (rightMiddleDistance < 0.4))
    {
        MainWindow::rampBack();
    }

    //zastavenie (pre zastavenie z hociakeho ineho gesta)
    else if((rightIndexDistance < 0.4) && (leftIndexDistance < 0.4) &&
            (rightMiddleDistance < 0.4) && (leftMiddleDistance < 0.4) &&
            (rightPinkyDistance < 0.4))
    {
        robot.setTranslationSpeed(0);
        robot.setRotationSpeed(0);
        actualSpeed = 0;
    }
}

void MainWindow::getNewFrame()
{

}

void MainWindow::ramp(){
    if (actualSpeed < maxSpeed){
        actualSpeed += increment;
        robot.setTranslationSpeed(actualSpeed);
    }
    else{
        robot.setTranslationSpeed(maxSpeed);
    }
}

void MainWindow::rampBack(){
    //std::cout << "ramp back" <<endl;
    if (actualSpeed > maxSpeedBack){
        actualSpeed -= increment;
        robot.setTranslationSpeed(actualSpeed);
    }
    else{
        robot.setTranslationSpeed(maxSpeedBack);
    }
    //std::cout << "actual speed" <<actualSpeed<<endl;
}

void MainWindow::draw_robot_arc(QPainter *painter, QPen *pen, QRect *rect, double startAngle, double arcLength, int emergency){

    if (emergency == 1){
        pen->setBrush(QColor(255, 255, 0));
        painter->setPen(*pen);
        painter->drawArc(rect->bottomRight().x()/2-35, rect->bottomRight().y()-110, 70, 70, startAngle*16, arcLength*16);

    } else if (emergency == 2){
        pen->setBrush(QColor(255, 255, 0));
        painter->setPen(*pen);
        painter->drawArc(rect->bottomRight().x()/2-35, rect->bottomRight().y()-110, 70, 70, startAngle*16, arcLength*16);
        pen->setBrush(QColor(255, 165, 0));
        painter->setPen(*pen);
        painter->drawArc(rect->bottomRight().x()/2-50, rect->bottomRight().y()-125, 100, 100, startAngle*16, arcLength*16);

    } else if (emergency == 3){

        pen->setBrush(QColor(255, 255, 0));
        painter->setPen(*pen);
        painter->drawArc(rect->bottomRight().x()/2-35, rect->bottomRight().y()-110, 70, 70, startAngle*16, arcLength*16);
        pen->setBrush(QColor(255, 165, 0));
        painter->setPen(*pen);
        painter->drawArc(rect->bottomRight().x()/2-50, rect->bottomRight().y()-125, 100, 100, startAngle*16, arcLength*16);
        pen->setBrush(QColor(255, 0, 0));
        painter->setPen(*pen);
        painter->drawArc(rect->bottomRight().x()/2-65, rect->bottomRight().y()-140, 130, 130, startAngle*16, arcLength*16);

        painter->fillRect(rect->topLeft().x()+100, rect->topLeft().y()+100, rect->bottomRight().x()-200, rect->bottomRight().y()-300, QColor(192, 192, 192, 35));
        painter->drawText((rect->bottomRight().x()/2)-(rect->bottomRight().x()/8), (rect->bottomRight().y()/2), alarmString);
    }
}
