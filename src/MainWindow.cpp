#include "MainWindow.h"
#include <QImage>
#include <QPixmap>
#include <opencv2/aruco.hpp>

MainWindow::MainWindow(){
    resize(960,720);
    view=new QLabel(this);
    setCentralWidget(view);

    cap.open(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH,640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,480);

    timer=new QTimer(this);
    connect(timer,&QTimer::timeout,this,&MainWindow::updateFrame);
    timer->start(30);
}

void MainWindow::updateFrame(){
    cv::Mat frame;
    cap>>frame;
    if(frame.empty()) return;

    static auto dict=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
    static cv::aruco::DetectorParameters p;
    static cv::aruco::ArucoDetector detector(dict,p);

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    detector.detectMarkers(frame,corners,ids);
    if(!ids.empty())
        cv::aruco::drawDetectedMarkers(frame,corners,ids);

    cv::cvtColor(frame,frame,cv::COLOR_BGR2RGB);
    QImage img(frame.data,frame.cols,frame.rows,frame.step,QImage::Format_RGB888);
    view->setPixmap(QPixmap::fromImage(img));
}
