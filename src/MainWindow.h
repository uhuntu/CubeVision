#pragma once
#include <QMainWindow>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <opencv2/opencv.hpp>
#include <vector>

class MainWindow: public QMainWindow{
    Q_OBJECT
public:
    MainWindow();
private slots:
    void updateFrame();
private:
    void finishCalibration();
    bool loadCalibration();
    bool saveCalibration() const;
    QString calibrationFilePath() const;

    QComboBox *dictionarySelector;
    QPushButton *calibrationModeButton;
    QPushButton *captureCalibrationButton;
    QPushButton *finishCalibrationButton;
    QLabel *calibrationStatus;
    QLabel *view;
    QTimer *timer;
    cv::VideoCapture cap;
    bool captureCalibrationFrame=false;
    bool cameraCalibrated=false;
    cv::Mat cameraMatrix;
    cv::Mat distortionCoefficients;
    cv::Size calibrationImageSize;
    std::vector<std::vector<cv::Point2f>> calibrationSamples;
    std::vector<std::vector<int>> calibrationSampleIds;
};
