#pragma once
#include <QMainWindow>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QProcess>
#include <QStringList>
#include <QTimer>
#include <opencv2/opencv.hpp>
#include <array>
#include <vector>

class CubeWidget;

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
    void captureCubeFace(const std::vector<std::vector<cv::Point2f>> &corners,
                         const std::vector<int> &ids);
    void commitCubeFace(int face,const std::array<int,9> &markers);
    void resetPendingCubeScan();
    void processAutomaticCubeScan(
        const std::vector<std::vector<cv::Point2f>> &corners,
        const std::vector<int> &ids);
    void updateCubeScanStatus(const QString &message=QString());
    bool cubeScanIsValid() const;
    bool normalizeCubeFaceOrientations();
    QByteArray cubeSolverInput() const;
    void startCubeSolver();
    void finishCubeSolver(int exitCode,QProcess::ExitStatus exitStatus);
    void updateSolutionStep();
    void verifySolutionMove();
    void processAutomaticMoveVerification(const std::vector<int> &ids);

    QComboBox *dictionarySelector;
    QPushButton *calibrationModeButton;
    QPushButton *captureCalibrationButton;
    QPushButton *finishCalibrationButton;
    QPushButton *toggleCalibrationButton;
    QPushButton *captureCubeFaceButton;
    QPushButton *resetCubeScanButton;
    QComboBox *solveMethodSelector;
    QPushButton *solveCubeButton;
    QPushButton *previousMoveButton;
    QPushButton *nextMoveButton;
    QLabel *calibrationStatus;
    QLabel *cubeScanStatus;
    QLabel *solutionStatus;
    QLabel *view;
    CubeWidget *cubeWidget;
    QTimer *timer;
    cv::VideoCapture cap;
    bool captureCalibrationFrame=false;
    bool cameraCalibrated=false;
    bool calibrationEnabled=false;
    cv::Mat cameraMatrix;
    cv::Mat distortionCoefficients;
    cv::Size calibrationImageSize;
    std::vector<std::vector<cv::Point2f>> calibrationSamples;
    std::vector<std::vector<int>> calibrationSampleIds;
    bool automaticCubeScan=false;
    int missedCubeScanFrames=0;
    int pendingCubeScanFace=-1;
    std::array<int,9> pendingCubeScanIds;
    std::array<int,9> pendingCubeScanCounts;
    std::vector<cv::Point2f> pendingCubeScanCenterCorners;
    std::array<std::array<int,9>,6> scannedCubeFaces;
    QProcess *cubeSolver=nullptr;
    QStringList solutionMoves;
    int solutionMoveIndex=0;
    bool crossSolutionActive=false;
    std::vector<int> lastMarkerIds;
    int stableVerificationFrames=0;
    int missedVerificationFrames=0;
    std::vector<int> stableVerificationIds;
};
