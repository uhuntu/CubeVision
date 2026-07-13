#include "MainWindow.h"
#include <QDir>
#include <QImage>
#include <QPixmap>
#include <QStandardPaths>
#include <QToolBar>
#include <opencv2/aruco.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>

namespace {
constexpr int BoardColumns=8;
constexpr int BoardRows=11;
constexpr float BoardSquareSizeMeters=0.020F;
constexpr float BoardMarkerSizeMeters=0.015F;
constexpr int MinimumCalibrationSamples=10;
constexpr int MinimumCornersPerSample=12;
constexpr float Marker5x5SizeMeters=0.015F;
constexpr float Marker4x4SizeMeters=0.010F;
constexpr int CaptureWidth=1920;
constexpr int CaptureHeight=1080;
constexpr int CaptureFramesPerSecond=30;

const cv::aruco::CharucoBoard& calibrationBoard(){
    static const cv::aruco::CharucoBoard board(
        cv::Size(BoardColumns,BoardRows),
        BoardSquareSizeMeters,
        BoardMarkerSizeMeters,
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100));
    return board;
}
}

MainWindow::MainWindow(){
    resize(960,720);

    auto *toolbar=addToolBar("Detection");
    toolbar->setMovable(false);
    toolbar->addWidget(new QLabel("ArUco dictionary: ",toolbar));

    dictionarySelector=new QComboBox(toolbar);
    dictionarySelector->addItem("5x5 (DICT_5X5_100)");
    dictionarySelector->addItem("4x4 (DICT_4X4_100)");
    dictionarySelector->setCurrentIndex(1);
    toolbar->addWidget(dictionarySelector);
    toolbar->addSeparator();

    calibrationModeButton=new QPushButton("Calibration mode",toolbar);
    calibrationModeButton->setCheckable(true);
    toolbar->addWidget(calibrationModeButton);

    captureCalibrationButton=new QPushButton("Capture",toolbar);
    captureCalibrationButton->setEnabled(false);
    toolbar->addWidget(captureCalibrationButton);

    finishCalibrationButton=new QPushButton("Calibrate",toolbar);
    finishCalibrationButton->setEnabled(false);
    toolbar->addWidget(finishCalibrationButton);

    calibrationStatus=new QLabel(toolbar);
    toolbar->addWidget(calibrationStatus);

    connect(calibrationModeButton,&QPushButton::toggled,this,[this](bool enabled){
        captureCalibrationButton->setEnabled(enabled);
        finishCalibrationButton->setEnabled(enabled);
        captureCalibrationFrame=false;
        if(enabled)
            calibrationStatus->setText(" Show the 8x11 ChArUco board");
        else
            calibrationStatus->setText(cameraCalibrated ? " Calibrated" : " Not calibrated");
    });
    connect(captureCalibrationButton,&QPushButton::clicked,this,[this]{
        captureCalibrationFrame=true;
        calibrationStatus->setText(" Waiting for board...");
    });
    connect(finishCalibrationButton,&QPushButton::clicked,this,&MainWindow::finishCalibration);

    view=new QLabel(this);
    view->setAlignment(Qt::AlignCenter);
    setCentralWidget(view);

#ifdef __linux__
    // Camera indices are not stable when USB devices are unplugged. Select the
    // first available V4L2 capture device instead of assuming /dev/video0.
    for(int cameraIndex=0;cameraIndex<10 && !cap.isOpened();++cameraIndex)
        cap.open(cameraIndex,cv::CAP_V4L2);
#else
    cap.open(0);
#endif
    cap.set(cv::CAP_PROP_FOURCC,cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH,CaptureWidth);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,CaptureHeight);
    cap.set(cv::CAP_PROP_FPS,CaptureFramesPerSecond);
    cap.set(cv::CAP_PROP_BUFFERSIZE,1);

    cameraCalibrated=loadCalibration();
    calibrationStatus->setText(cameraCalibrated ? " Calibrated" : " Not calibrated");

    timer=new QTimer(this);
    connect(timer,&QTimer::timeout,this,&MainWindow::updateFrame);
    timer->start(30);
}

QString MainWindow::calibrationFilePath() const{
    const QString directory=QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
    QDir().mkpath(directory);
    return QDir(directory).filePath("camera.yml");
}

bool MainWindow::loadCalibration(){
    cv::FileStorage file(calibrationFilePath().toStdString(),cv::FileStorage::READ);
    if(!file.isOpened())
        return false;

    int width=0;
    int height=0;
    file["camera_matrix"]>>cameraMatrix;
    file["distortion_coefficients"]>>distortionCoefficients;
    file["image_width"]>>width;
    file["image_height"]>>height;
    calibrationImageSize=cv::Size(width,height);

    return !cameraMatrix.empty() && !distortionCoefficients.empty()
        && width>0 && height>0;
}

bool MainWindow::saveCalibration() const{
    cv::FileStorage file(calibrationFilePath().toStdString(),cv::FileStorage::WRITE);
    if(!file.isOpened())
        return false;

    file<<"camera_matrix"<<cameraMatrix;
    file<<"distortion_coefficients"<<distortionCoefficients;
    file<<"image_width"<<calibrationImageSize.width;
    file<<"image_height"<<calibrationImageSize.height;
    return true;
}

void MainWindow::finishCalibration(){
    if(static_cast<int>(calibrationSamples.size())<MinimumCalibrationSamples){
        calibrationStatus->setText(
            QString(" Need %1 samples (%2 captured)")
                .arg(MinimumCalibrationSamples)
                .arg(calibrationSamples.size()));
        return;
    }

    const auto &allBoardPoints=calibrationBoard().getChessboardCorners();
    std::vector<std::vector<cv::Point3f>> objectPoints;
    objectPoints.reserve(calibrationSampleIds.size());
    for(const auto &sampleIds:calibrationSampleIds){
        std::vector<cv::Point3f> samplePoints;
        samplePoints.reserve(sampleIds.size());
        for(const int id:sampleIds)
            samplePoints.push_back(allBoardPoints.at(id));
        objectPoints.push_back(std::move(samplePoints));
    }
    std::vector<cv::Mat> rotationVectors;
    std::vector<cv::Mat> translationVectors;
    cameraMatrix=cv::Mat::eye(3,3,CV_64F);
    distortionCoefficients=cv::Mat::zeros(8,1,CV_64F);

    const double error=cv::calibrateCamera(objectPoints,
                                           calibrationSamples,
                                           calibrationImageSize,
                                           cameraMatrix,
                                           distortionCoefficients,
                                           rotationVectors,
                                           translationVectors);
    cameraCalibrated=std::isfinite(error) && saveCalibration();
    if(cameraCalibrated){
        calibrationStatus->setText(QString(" Calibrated (RMS %1 px)").arg(error,0,'f',3));
        calibrationModeButton->setChecked(false);
        calibrationSamples.clear();
        calibrationSampleIds.clear();
    }else{
        calibrationStatus->setText(" Calibration failed");
    }
}

void MainWindow::updateFrame(){
    cv::Mat frame;
    cap>>frame;
    if(frame.empty())
        return;

    if(calibrationModeButton->isChecked()){
        static const cv::aruco::CharucoDetector detector(calibrationBoard());
        std::vector<cv::Point2f> charucoCorners;
        std::vector<int> charucoIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        std::vector<int> markerIds;
        detector.detectBoard(frame,charucoCorners,charucoIds,markerCorners,markerIds);

        if(!markerIds.empty())
            cv::aruco::drawDetectedMarkers(frame,markerCorners);
        if(!charucoIds.empty())
            cv::aruco::drawDetectedCornersCharuco(
                frame,charucoCorners,cv::noArray(),cv::Scalar(255,0,0));

        if(captureCalibrationFrame
            && static_cast<int>(charucoIds.size())>=MinimumCornersPerSample){
            calibrationSamples.push_back(charucoCorners);
            calibrationSampleIds.push_back(charucoIds);
            calibrationImageSize=frame.size();
            captureCalibrationFrame=false;
            calibrationStatus->setText(
                QString(" %1/%2 samples")
                    .arg(calibrationSamples.size())
                    .arg(MinimumCalibrationSamples));
        }else if(captureCalibrationFrame && !charucoIds.empty()){
            calibrationStatus->setText(
                QString(" Need %1 visible corners (%2 found)")
                    .arg(MinimumCornersPerSample)
                    .arg(charucoIds.size()));
        }else if(!captureCalibrationFrame){
            calibrationStatus->setText(
                QString(" %1 corners visible | %2/%3 samples")
                    .arg(charucoIds.size())
                    .arg(calibrationSamples.size())
                    .arg(MinimumCalibrationSamples));
        }
    }else{
        static cv::aruco::DetectorParameters parameters;
        static cv::aruco::ArucoDetector detector5x5(
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100),parameters);
        static cv::aruco::ArucoDetector detector4x4(
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100),parameters);

        const bool use5x5=dictionarySelector->currentIndex()==0;
        const auto &detector=use5x5 ? detector5x5 : detector4x4;
        const float markerSize=use5x5 ? Marker5x5SizeMeters : Marker4x4SizeMeters;

        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        detector.detectMarkers(frame,corners,ids);
        if(!ids.empty()){
            cv::aruco::drawDetectedMarkers(frame,corners,ids);

            if(cameraCalibrated){
                cv::Mat scaledCameraMatrix=cameraMatrix.clone();
                if(frame.size()!=calibrationImageSize){
                    const double scaleX=static_cast<double>(frame.cols)/calibrationImageSize.width;
                    const double scaleY=static_cast<double>(frame.rows)/calibrationImageSize.height;
                    scaledCameraMatrix.at<double>(0,0)*=scaleX;
                    scaledCameraMatrix.at<double>(0,2)*=scaleX;
                    scaledCameraMatrix.at<double>(1,1)*=scaleY;
                    scaledCameraMatrix.at<double>(1,2)*=scaleY;
                }

                const float halfSize=markerSize*0.5F;
                const std::vector<cv::Point3f> markerPoints={
                    {-halfSize, halfSize,0.0F},
                    { halfSize, halfSize,0.0F},
                    { halfSize,-halfSize,0.0F},
                    {-halfSize,-halfSize,0.0F}
                };
                const std::vector<cv::Point3f> normalPoints={
                    {0.0F,0.0F,0.0F},
                    {0.0F,0.0F,markerSize*1.5F}
                };

                for(const auto &markerCorners:corners){
                    cv::Vec3d rotationVector;
                    cv::Vec3d translationVector;
                    if(!cv::solvePnP(markerPoints,
                                     markerCorners,
                                     scaledCameraMatrix,
                                     distortionCoefficients,
                                     rotationVector,
                                     translationVector,
                                     false,
                                     cv::SOLVEPNP_IPPE_SQUARE))
                        continue;

                    std::vector<cv::Point2f> projectedNormal;
                    cv::projectPoints(normalPoints,
                                      rotationVector,
                                      translationVector,
                                      scaledCameraMatrix,
                                      distortionCoefficients,
                                      projectedNormal);
                    cv::arrowedLine(frame,
                                    projectedNormal[0],
                                    projectedNormal[1],
                                    cv::Scalar(0,0,255),
                                    3,
                                    cv::LINE_AA,
                                    0,
                                    0.18);
                    cv::circle(frame,
                               projectedNormal[0],
                               5,
                               cv::Scalar(0,0,255),
                               cv::FILLED,
                               cv::LINE_AA);
                }
            }
        }
    }

    cv::cvtColor(frame,frame,cv::COLOR_BGR2RGB);
    QImage image(frame.data,frame.cols,frame.rows,frame.step,QImage::Format_RGB888);
    view->setPixmap(QPixmap::fromImage(image).scaled(
        view->size(),Qt::KeepAspectRatio,Qt::SmoothTransformation));
}
