#pragma once
#include <QMainWindow>
#include <QLabel>
#include <QTimer>
#include <opencv2/opencv.hpp>

class MainWindow: public QMainWindow{
    Q_OBJECT
public:
    MainWindow();
private slots:
    void updateFrame();
private:
    QLabel *view;
    QTimer *timer;
    cv::VideoCapture cap;
};
