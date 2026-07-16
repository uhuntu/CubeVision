#include "MainWindow.h"
#include "CubeWidget.h"
#include <QDir>
#include <QCoreApplication>
#include <QFileInfo>
#include <QHash>
#include <QHBoxLayout>
#include <QImage>
#include <QPixmap>
#include <QRegularExpression>
#include <QStandardPaths>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWidget>
#include <opencv2/aruco.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include <algorithm>
#include <cmath>
#include <numeric>

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

cv::Scalar cubeFaceColor(const int markerId){
    // Marker IDs in the CubeNet layout encode the face as id modulo six.
    switch(markerId%6){
    case 0: return {0,255,255};   // Up: yellow
    case 1: return {255,255,255}; // Down: white
    case 2: return {255,0,0};     // Left: blue
    case 3: return {0,190,0};     // Right: green
    case 4: return {0,0,255};     // Front: red
    default:return {0,128,255};   // Back: orange
    }
}

const char* cubeFaceName(const int markerId){
    static constexpr const char *names[]={"U","D","L","R","F","B"};
    return names[markerId%6];
}

char cubeColorName(const int markerId){
    static constexpr char names[]={'Y','W','B','G','R','O'};
    return names[markerId%6];
}

using CubeFaces=std::array<std::array<int,9>,6>;

std::array<int,9> rotateFaceClockwise(const std::array<int,9> &face){
    std::array<int,9> rotated;
    for(int row=0;row<3;++row)
        for(int column=0;column<3;++column)
            rotated[row*3+column]=face[(2-column)*3+row];
    return rotated;
}

bool sameFaceMarkers(std::array<int,9> left,std::array<int,9> right){
    // Marker IDs are unique.  For move verification the set of stickers on an
    // adjacent face identifies the turn, while ignoring the arbitrary in-plane
    // orientation introduced when the scanned faces are normalized.
    std::sort(left.begin(),left.end());
    std::sort(right.begin(),right.end());
    return left==right;
}

bool sameCubieColors(std::array<int,3> left,std::array<int,3> right){
    std::sort(left.begin(),left.end());
    std::sort(right.begin(),right.end());
    return left==right;
}

bool sameCubieColors(std::array<int,2> left,std::array<int,2> right){
    std::sort(left.begin(),left.end());
    std::sort(right.begin(),right.end());
    return left==right;
}

int permutationParity(const int *permutation,const int size){
    int inversions=0;
    for(int left=0;left<size;++left)
        for(int right=left+1;right<size;++right)
            inversions+=permutation[left]>permutation[right] ? 1 : 0;
    return inversions%2;
}

bool cubeStateIsSolvable(const CubeFaces &faces){
    // Face indexes: U=0, D=1, L=2, R=3, F=4, B=5. Facelet indexes
    // are row-major as viewed directly from outside the cube.
    struct Facelet{ int face; int position; };
    static constexpr Facelet cornerFacelets[8][3]={
        {{0,8},{3,0},{4,2}}, // URF
        {{0,6},{4,0},{2,2}}, // UFL
        {{0,0},{2,0},{5,2}}, // ULB
        {{0,2},{5,0},{3,2}}, // UBR
        {{1,2},{4,8},{3,6}}, // DFR
        {{1,0},{2,8},{4,6}}, // DLF
        {{1,6},{5,8},{2,6}}, // DBL
        {{1,8},{3,8},{5,6}}  // DRB
    };
    static constexpr int cornerColors[8][3]={
        {0,3,4},{0,4,2},{0,2,5},{0,5,3},
        {1,4,3},{1,2,4},{1,5,2},{1,3,5}
    };
    static constexpr Facelet edgeFacelets[12][2]={
        {{0,5},{3,1}},{{0,7},{4,1}},{{0,3},{2,1}},{{0,1},{5,1}},
        {{1,5},{3,7}},{{1,1},{4,7}},{{1,3},{2,7}},{{1,7},{5,7}},
        {{4,5},{3,3}},{{4,3},{2,5}},{{5,5},{2,3}},{{5,3},{3,5}}
    };
    static constexpr int edgeColors[12][2]={
        {0,3},{0,4},{0,2},{0,5},{1,3},{1,4},
        {1,2},{1,5},{4,3},{4,2},{5,2},{5,3}
    };
    auto color=[&faces](const Facelet facelet){
        const int id=faces[facelet.face][facelet.position];
        return id>=0 ? id%6 : -1;
    };

    int cornerPermutation[8];
    int cornerOrientationSum=0;
    std::array<bool,8> cornerSeen{};
    for(int position=0;position<8;++position){
        const std::array<int,3> colors={color(cornerFacelets[position][0]),
                                        color(cornerFacelets[position][1]),
                                        color(cornerFacelets[position][2])};
        int cubie=-1;
        for(int candidate=0;candidate<8;++candidate){
            if(sameCubieColors(colors,{cornerColors[candidate][0],
                                       cornerColors[candidate][1],
                                       cornerColors[candidate][2]})){
                cubie=candidate;
                break;
            }
        }
        if(cubie<0 || cornerSeen[cubie])
            return false;
        cornerSeen[cubie]=true;
        cornerPermutation[position]=cubie;

        int orientation=0;
        while(orientation<3 && colors[orientation]!=0 && colors[orientation]!=1)
            ++orientation;
        if(orientation==3)
            return false;
        cornerOrientationSum+=orientation;
    }
    if(cornerOrientationSum%3!=0)
        return false;

    int edgePermutation[12];
    int edgeOrientationSum=0;
    std::array<bool,12> edgeSeen{};
    for(int position=0;position<12;++position){
        const std::array<int,2> colors={color(edgeFacelets[position][0]),
                                        color(edgeFacelets[position][1])};
        int cubie=-1;
        int orientation=0;
        for(int candidate=0;candidate<12;++candidate){
            if(!sameCubieColors(colors,{edgeColors[candidate][0],edgeColors[candidate][1]}))
                continue;
            cubie=candidate;
            orientation=colors[0]==edgeColors[candidate][0] ? 0 : 1;
            break;
        }
        if(cubie<0 || edgeSeen[cubie])
            return false;
        edgeSeen[cubie]=true;
        edgePermutation[position]=cubie;
        edgeOrientationSum+=orientation;
    }
    if(edgeOrientationSum%2!=0)
        return false;
    return permutationParity(cornerPermutation,8)==permutationParity(edgePermutation,12);
}

struct CubeVector{ int x; int y; int z; };
struct FaceletGeometry{ CubeVector position; CubeVector normal; };

FaceletGeometry faceletGeometry(const int face,const int row,const int column){
    switch(face){
    case 0: return {{column-1, 1,row-1},{ 0, 1, 0}}; // U
    case 1: return {{column-1,-1,1-row},{ 0,-1, 0}}; // D
    case 2: return {{-1,1-row,column-1},{-1, 0, 0}}; // L
    case 3: return {{ 1,1-row,1-column},{ 1, 0, 0}}; // R
    case 4: return {{column-1,1-row, 1},{ 0, 0, 1}}; // F
    default:return {{1-column,1-row,-1},{ 0, 0,-1}}; // B
    }
}

std::pair<int,int> faceletIndex(const FaceletGeometry &geometry){
    const auto &position=geometry.position;
    const auto &normal=geometry.normal;
    if(normal.y==1)  return {0,(position.z+1)*3+position.x+1};
    if(normal.y==-1) return {1,(1-position.z)*3+position.x+1};
    if(normal.x==-1) return {2,(1-position.y)*3+position.z+1};
    if(normal.x==1)  return {3,(1-position.y)*3+1-position.z};
    if(normal.z==1)  return {4,(1-position.y)*3+position.x+1};
    return {5,(1-position.y)*3+1-position.x};
}

CubeVector rotateClockwise(const CubeVector value,const CubeVector normal){
    const int parallel=value.x*normal.x+value.y*normal.y+value.z*normal.z;
    const CubeVector cross={normal.y*value.z-normal.z*value.y,
                            normal.z*value.x-normal.x*value.z,
                            normal.x*value.y-normal.y*value.x};
    return {normal.x*parallel-cross.x,
            normal.y*parallel-cross.y,
            normal.z*parallel-cross.z};
}

int faceIndex(const QChar face){
    if(face=='U') return 0;
    if(face=='D') return 1;
    if(face=='L') return 2;
    if(face=='R') return 3;
    if(face=='F') return 4;
    return 5;
}

int verificationFaceForMove(const int movingFace){
    // The face being turned cannot verify its own movement when its center
    // marker defines the image axes: that marker rotates with the whole layer,
    // making the nine-marker layout appear unchanged.  Observe an adjacent
    // face whose center stays fixed instead.
    return movingFace==4 || movingFace==5 ? 0 : 4; // F/B -> U; others -> F
}

CubeFaces applyCubeMove(const CubeFaces &faces,const QChar face,const int turns){
    CubeFaces result=faces;
    const int movingFace=faceIndex(face);
    const CubeVector axis=faceletGeometry(movingFace,1,1).normal;
    for(int sourceFace=0;sourceFace<6;++sourceFace){
        for(int sourcePosition=0;sourcePosition<9;++sourcePosition){
            FaceletGeometry geometry=faceletGeometry(
                sourceFace,sourcePosition/3,sourcePosition%3);
            const int layer=geometry.position.x*axis.x
                +geometry.position.y*axis.y+geometry.position.z*axis.z;
            if(layer!=1)
                continue;
            for(int turn=0;turn<turns;++turn){
                geometry.position=rotateClockwise(geometry.position,axis);
                geometry.normal=rotateClockwise(geometry.normal,axis);
            }
            const auto [destinationFace,destinationPosition]=faceletIndex(geometry);
            result[destinationFace][destinationPosition]=faces[sourceFace][sourcePosition];
        }
    }
    return result;
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

    toggleCalibrationButton=new QPushButton("Disable calibration",toolbar);
    toggleCalibrationButton->setEnabled(false);
    toolbar->addWidget(toggleCalibrationButton);

    calibrationStatus=new QLabel(toolbar);
    toolbar->addWidget(calibrationStatus);
    toolbar->addSeparator();

    captureCubeFaceButton=new QPushButton("Auto scan",toolbar);
    captureCubeFaceButton->setCheckable(true);
    toolbar->addWidget(captureCubeFaceButton);

    resetCubeScanButton=new QPushButton("Reset scan",toolbar);
    toolbar->addWidget(resetCubeScanButton);

    addToolBarBreak();
    auto *coachToolbar=addToolBar("Solve Coach");
    coachToolbar->setMovable(false);
    coachToolbar->addWidget(new QLabel("Solve Coach: ",coachToolbar));

    solveCubeButton=new QPushButton("Solve",coachToolbar);
    solveCubeButton->setEnabled(false);
    coachToolbar->addWidget(solveCubeButton);

    previousMoveButton=new QPushButton("Previous",coachToolbar);
    previousMoveButton->setEnabled(false);
    previousMoveButton->setVisible(false);
    coachToolbar->addWidget(previousMoveButton);

    nextMoveButton=new QPushButton("Verify move",coachToolbar);
    nextMoveButton->setEnabled(false);
    coachToolbar->addWidget(nextMoveButton);

    connect(calibrationModeButton,&QPushButton::toggled,this,[this](bool enabled){
        captureCalibrationButton->setEnabled(enabled);
        finishCalibrationButton->setEnabled(enabled);
        captureCalibrationFrame=false;
        if(enabled)
            calibrationStatus->setText(" Show the 8x11 ChArUco board");
        else
            calibrationStatus->setText(!cameraCalibrated ? " Not calibrated"
                : calibrationEnabled ? " Calibrated" : " Calibration disabled");
    });
    connect(captureCalibrationButton,&QPushButton::clicked,this,[this]{
        captureCalibrationFrame=true;
        calibrationStatus->setText(" Waiting for board...");
    });
    connect(finishCalibrationButton,&QPushButton::clicked,this,&MainWindow::finishCalibration);
    connect(toggleCalibrationButton,&QPushButton::clicked,this,[this]{
        if(!cameraCalibrated)
            return;
        calibrationEnabled=!calibrationEnabled;
        toggleCalibrationButton->setText(
            calibrationEnabled ? "Disable calibration" : "Enable calibration");
        calibrationStatus->setText(
            calibrationEnabled ? " Calibrated" : " Calibration disabled");
    });
    connect(captureCubeFaceButton,&QPushButton::toggled,this,[this](const bool enabled){
        automaticCubeScan=enabled;
        stableCubeScanFrames=0;
        stableCubeScanIds.clear();
        captureCubeFaceButton->setText(enabled ? "Stop auto scan" : "Auto scan");
        if(enabled){
            calibrationModeButton->setChecked(false);
            dictionarySelector->setCurrentIndex(1);
            updateCubeScanStatus("Auto scan active - show one complete face");
        }else{
            updateCubeScanStatus("Auto scan stopped");
        }
    });
    connect(resetCubeScanButton,&QPushButton::clicked,this,[this]{
        for(auto &face:scannedCubeFaces)
            face.fill(-1);
        stableCubeScanFrames=0;
        stableCubeScanIds.clear();
        solutionMoves.clear();
        solutionMoveIndex=0;
        solutionStatus->clear();
        cubeWidget->clearCubeState();
        solveCubeButton->setEnabled(false);
        previousMoveButton->setEnabled(false);
        nextMoveButton->setEnabled(false);
        updateCubeScanStatus(automaticCubeScan
            ? "Cube scan reset - show one complete face"
            : "Cube scan reset");
    });
    connect(solveCubeButton,&QPushButton::clicked,this,&MainWindow::startCubeSolver);
    connect(nextMoveButton,&QPushButton::clicked,this,&MainWindow::verifySolutionMove);

    auto *central=new QWidget(this);
    auto *layout=new QVBoxLayout(central);
    layout->setContentsMargins(0,0,0,0);
    layout->setSpacing(4);

    view=new QLabel(central);
    view->setAlignment(Qt::AlignCenter);

    auto *visualLayout=new QHBoxLayout;
    visualLayout->setContentsMargins(0,0,0,0);
    visualLayout->setSpacing(4);
    visualLayout->addWidget(view,3);

    cubeWidget=new CubeWidget(central);
    visualLayout->addWidget(cubeWidget,1);
    layout->addLayout(visualLayout,1);

    cubeScanStatus=new QLabel(central);
    cubeScanStatus->setAlignment(Qt::AlignCenter);
    cubeScanStatus->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(cubeScanStatus);

    solutionStatus=new QLabel(central);
    solutionStatus->setAlignment(Qt::AlignCenter);
    QFont solutionFont=solutionStatus->font();
    solutionFont.setPointSize(solutionFont.pointSize()+4);
    solutionFont.setBold(true);
    solutionStatus->setFont(solutionFont);
    layout->addWidget(solutionStatus);
    setCentralWidget(central);

    for(auto &face:scannedCubeFaces)
        face.fill(-1);
    cubeWidget->setCubeState(scannedCubeFaces);
    updateCubeScanStatus();

    cubeSolver=new QProcess(this);
    cubeSolver->setProcessChannelMode(QProcess::MergedChannels);
    connect(cubeSolver,&QProcess::finished,this,&MainWindow::finishCubeSolver);

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
    calibrationEnabled=cameraCalibrated;
    toggleCalibrationButton->setEnabled(cameraCalibrated);
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

bool MainWindow::cubeScanIsValid() const{
    std::array<int,54> occurrences{};
    for(const auto &face:scannedCubeFaces){
        for(const int id:face){
            if(id<0 || id>=static_cast<int>(occurrences.size()))
                return false;
            ++occurrences[id];
        }
    }
    return std::all_of(occurrences.begin(),occurrences.end(),
                       [](const int count){ return count==1; });
}

bool MainWindow::normalizeCubeFaceOrientations(){
    if(!cubeScanIsValid())
        return false;
    const CubeFaces original=scannedCubeFaces;
    constexpr int combinations=4*4*4*4*4*4;
    for(int combination=0;combination<combinations;++combination){
        CubeFaces candidate=original;
        int rotations=combination;
        for(auto &face:candidate){
            const int turns=rotations%4;
            rotations/=4;
            for(int turn=0;turn<turns;++turn)
                face=rotateFaceClockwise(face);
        }
        if(cubeStateIsSolvable(candidate)){
            scannedCubeFaces=std::move(candidate);
            return true;
        }
    }
    return false;
}

void MainWindow::updateCubeScanStatus(const QString &message){
    static constexpr const char *faceNames[]={"U","D","L","R","F","B"};
    int scannedCount=0;
    QString state;
    for(std::size_t face=0;face<scannedCubeFaces.size();++face){
        if(face>0)
            state+=face==3 ? "\n" : "   ";
        state+=QString::fromLatin1(faceNames[face])+": ";
        if(scannedCubeFaces[face][0]<0){
            state+="---/---/---";
            continue;
        }

        ++scannedCount;
        for(std::size_t position=0;position<scannedCubeFaces[face].size();++position){
            if(position>0 && position%3==0)
                state+='/';
            state+=QChar(cubeColorName(scannedCubeFaces[face][position]));
        }
    }

    QString heading=message;
    if(heading.isEmpty()){
        if(scannedCount==6){
            heading=cubeScanIsValid()
                ? "All six faces scanned - cube state ready"
                : "Scan conflict: every marker ID 0-53 must appear exactly once";
        }else{
            heading=QString("Cube scan: %1/6 faces").arg(scannedCount);
        }
    }
    cubeScanStatus->setText(heading+'\n'+state);
    solveCubeButton->setEnabled(cubeScanIsValid()
        && (!cubeSolver || cubeSolver->state()==QProcess::NotRunning));
}

QByteArray MainWindow::cubeSolverInput() const{
    QByteArray input;
    auto appendFace=[this,&input](const int face){
        for(const int id:scannedCubeFaces[face])
            input.append(cubeFaceName(id));
    };
    auto appendRow=[this,&input](const int face,const int row){
        for(int column=0;column<3;++column)
            input.append(cubeFaceName(scannedCubeFaces[face][row*3+column]));
    };

    // DikCube accepts this unfolded net, matching the orientation printed in
    // CubeVision's marker PDF:
    //       U
    //     L F R B
    //       D
    appendFace(0);
    for(int row=0;row<3;++row){
        appendRow(2,row);
        appendRow(4,row);
        appendRow(3,row);
        appendRow(5,row);
    }
    appendFace(1);
    input.append('\n');
    return input;
}

void MainWindow::startCubeSolver(){
    if(!cubeScanIsValid() || !cubeSolver || cubeSolver->state()!=QProcess::NotRunning)
        return;

    QString program=QDir(QCoreApplication::applicationDirPath()).filePath("CubeSolver");
#ifdef Q_OS_WIN
    program+=".exe";
#endif
    if(!QFileInfo::exists(program)){
        solutionStatus->setText("Cube solver executable is missing");
        return;
    }

    solutionMoves.clear();
    solutionMoveIndex=0;
    solutionStatus->setText("Checking cube state and calculating a solution...");
    solveCubeButton->setEnabled(false);
    previousMoveButton->setEnabled(false);
    nextMoveButton->setEnabled(false);
    cubeSolver->start(program,{"-p","-m"});
    if(!cubeSolver->waitForStarted(1000)){
        solutionStatus->setText("Could not start the cube solver");
        solveCubeButton->setEnabled(true);
        return;
    }
    cubeSolver->write(cubeSolverInput());
    cubeSolver->closeWriteChannel();
}

void MainWindow::finishCubeSolver(const int exitCode,const QProcess::ExitStatus exitStatus){
    const QString output=QString::fromLocal8Bit(cubeSolver->readAll());
    if(exitStatus!=QProcess::NormalExit || exitCode!=0){
        QString reason="Cube state is not physically solvable; rescan each face orientation";
        const QStringList lines=output.split('\n',Qt::SkipEmptyParts);
        for(auto iterator=lines.crbegin();iterator!=lines.crend();++iterator){
            if(iterator->contains("cube is",Qt::CaseInsensitive)){
                reason=*iterator;
                break;
            }
        }
        solutionStatus->setText(reason);
        cubeWidget->setActiveMove(QString());
        solveCubeButton->setEnabled(true);
        return;
    }

    QRegularExpression expression("Solution \\([^\\r\\n]*\\):([^\\r\\n]*)");
    auto matches=expression.globalMatch(output);
    QString encodedMoves;
    while(matches.hasNext())
        encodedMoves=matches.next().captured(1).trimmed();

    if(encodedMoves.isEmpty()){
        solutionStatus->setText("Cube is already solved");
        cubeWidget->setActiveMove(QString());
        solveCubeButton->setEnabled(true);
        return;
    }

    // The input net rotates the solver's coordinate system. Translate its
    // move letters back to CubeVision's U/D/L/R/F/B labels.
    const QHash<QChar,QChar> faceMap={{'F','D'},{'R','R'},{'U','F'},
                                     {'B','U'},{'L','L'},{'D','B'}};
    const QRegularExpression moveExpression("([FRUBLD])([123])");
    auto moveMatches=moveExpression.globalMatch(encodedMoves);
    while(moveMatches.hasNext()){
        const auto match=moveMatches.next();
        QString move(faceMap.value(match.captured(1).front()));
        const QChar turns=match.captured(2).front();
        if(turns=='2')
            move+='2';
        else if(turns=='3')
            move+='\'';
        solutionMoves.push_back(move);
    }

    if(solutionMoves.isEmpty()){
        solutionStatus->setText("Solver returned an unreadable solution");
        cubeWidget->setActiveMove(QString());
        solveCubeButton->setEnabled(true);
        return;
    }
    solutionMoveIndex=0;
    solutionStatus->setToolTip("Full solution: "+solutionMoves.join(' '));
    updateSolutionStep();
}

void MainWindow::updateSolutionStep(){
    if(solutionMoves.isEmpty()){
        previousMoveButton->setEnabled(false);
        nextMoveButton->setEnabled(false);
        return;
    }
    if(solutionMoveIndex>=solutionMoves.size()){
        solutionStatus->setText("Solution complete - your cube should now be solved");
        cubeWidget->setActiveMove(QString());
        previousMoveButton->setEnabled(false);
        nextMoveButton->setEnabled(false);
        solveCubeButton->setEnabled(true);
        return;
    }

    const QString move=solutionMoves[solutionMoveIndex];
    cubeWidget->setActiveMove(move);
    const QHash<QChar,QString> faceNames={{'U',"Up (yellow)"},{'D',"Down (white)"},
                                         {'L',"Left (blue)"},{'R',"Right (green)"},
                                         {'F',"Front (red)"},{'B',"Back (orange)"}};
    QString direction="clockwise 90 degrees";
    if(move.endsWith('\''))
        direction="counterclockwise 90 degrees";
    else if(move.endsWith('2'))
        direction="180 degrees";
    const int verificationFace=verificationFaceForMove(faceIndex(move.front()));
    const QChar verificationLetter=QChar::fromLatin1(cubeFaceName(verificationFace)[0]);
    solutionStatus->setText(
        QString("Step %1/%2:  %3  - turn %4 %5; then show %6 to verify")
            .arg(solutionMoveIndex+1)
            .arg(solutionMoves.size())
            .arg(move)
            .arg(faceNames.value(move.front()))
            .arg(direction)
            .arg(faceNames.value(verificationLetter)));
    previousMoveButton->setEnabled(false);
    nextMoveButton->setEnabled(true);
    solveCubeButton->setEnabled(true);
}

void MainWindow::verifySolutionMove(){
    if(solutionMoveIndex<0 || solutionMoveIndex>=solutionMoves.size())
        return;
    if(lastMarkerIds.size()!=9){
        solutionStatus->setText(
            QString("Verification needs exactly 9 visible markers (%1 found)")
                .arg(lastMarkerIds.size()));
        return;
    }

    const QString move=solutionMoves[solutionMoveIndex];
    const int movingFace=faceIndex(move.front());
    const int verificationFace=verificationFaceForMove(movingFace);

    std::size_t centerIndex=lastMarkerIds.size();
    for(std::size_t index=0;index<lastMarkerIds.size();++index){
        if(lastMarkerIds[index]==verificationFace){
            centerIndex=index;
            break;
        }
    }
    if(centerIndex==lastMarkerIds.size()){
        solutionStatus->setText(
            QString("Show the %1 face to verify move %2")
                .arg(cubeFaceName(verificationFace)).arg(move));
        return;
    }

    std::array<int,9> observed;
    for(std::size_t position=0;position<observed.size();++position)
        observed[position]=lastMarkerIds[position];

    int expectedTurns=1;
    if(move.endsWith('2'))
        expectedTurns=2;
    else if(move.endsWith('\''))
        expectedTurns=3;
    const CubeFaces expected=applyCubeMove(scannedCubeFaces,move.front(),expectedTurns);
    if(sameFaceMarkers(observed,expected[verificationFace])){
        scannedCubeFaces=expected;
        cubeWidget->setCubeState(scannedCubeFaces);
        ++solutionMoveIndex;
        updateSolutionStep();
        return;
    }
    if(sameFaceMarkers(observed,scannedCubeFaces[verificationFace])){
        solutionStatus->setText("The adjacent stickers have not moved yet; perform "
                                +move+" and verify again");
        return;
    }

    for(int detectedTurns=1;detectedTurns<=3;++detectedTurns){
        if(!sameFaceMarkers(
               observed,
               applyCubeMove(scannedCubeFaces,move.front(),detectedTurns)[verificationFace]))
            continue;
        const QString detected=detectedTurns==1 ? QString(move.front())
            : detectedTurns==2 ? QString(move.front())+'2'
            : QString(move.front())+'\'';
        solutionStatus->setText(
            QString("Wrong turn detected: %1; expected %2. Undo it and try again")
                .arg(detected).arg(move));
        return;
    }
    solutionStatus->setText("Face does not match the expected state; undo the last action and retry");
}

void MainWindow::processAutomaticCubeScan(
    const std::vector<std::vector<cv::Point2f>> &corners,
    const std::vector<int> &ids){
    constexpr int RequiredStableFrames=8;
    if(!automaticCubeScan)
        return;
    if(ids.size()!=9){
        stableCubeScanFrames=0;
        stableCubeScanIds.clear();
        updateCubeScanStatus(
            QString("Auto scan: show exactly one face (%1/9 markers visible)")
                .arg(ids.size()));
        return;
    }

    int center=-1;
    int centerCount=0;
    for(const int id:ids){
        if(id>=0 && id<6){
            center=id;
            ++centerCount;
        }
    }
    if(centerCount!=1){
        stableCubeScanFrames=0;
        stableCubeScanIds.clear();
        updateCubeScanStatus("Auto scan: aim one face directly at the camera");
        return;
    }
    if(scannedCubeFaces[center][0]>=0){
        stableCubeScanFrames=0;
        stableCubeScanIds.clear();
        updateCubeScanStatus(
            QString("%1 face already scanned - show a different face")
                .arg(cubeFaceName(center)));
        return;
    }

    std::vector<int> signature=ids;
    std::sort(signature.begin(),signature.end());
    if(signature!=stableCubeScanIds){
        stableCubeScanIds=std::move(signature);
        stableCubeScanFrames=1;
    }else{
        ++stableCubeScanFrames;
    }
    if(stableCubeScanFrames<RequiredStableFrames){
        updateCubeScanStatus(
            QString("Hold %1 face steady... %2/%3")
                .arg(cubeFaceName(center))
                .arg(stableCubeScanFrames)
                .arg(RequiredStableFrames));
        return;
    }

    stableCubeScanFrames=0;
    stableCubeScanIds.clear();
    captureCubeFace(corners,ids);
}

void MainWindow::captureCubeFace(
    const std::vector<std::vector<cv::Point2f>> &corners,
    const std::vector<int> &ids){
    if(ids.size()!=9){
        updateCubeScanStatus(
            QString("Show only one complete face: exactly 9 markers required (%1 found)")
                .arg(ids.size()));
        return;
    }

    auto markerCenter=[&corners](const std::size_t index){
        cv::Point2f center;
        for(const cv::Point2f &corner:corners[index])
            center+=corner;
        return center*(1.0F/static_cast<float>(corners[index].size()));
    };

    // The face being presented is identified by its center sticker (IDs 0-5).
    // If an adjacent center is visible, the largest one belongs to the face
    // aimed most directly at the camera.
    std::size_t centerIndex=ids.size();
    double largestCenterArea=0.0;
    for(std::size_t index=0;index<ids.size();++index){
        if(ids[index]<0 || ids[index]>=6)
            continue;
        const double area=std::abs(cv::contourArea(corners[index]));
        if(area>largestCenterArea){
            largestCenterArea=area;
            centerIndex=index;
        }
    }
    if(centerIndex==ids.size()){
        updateCubeScanStatus("No center marker found (expected an ID from 0 to 5)");
        return;
    }

    const cv::Point2f faceCenter=markerCenter(centerIndex);
    std::vector<std::size_t> nearest(ids.size());
    std::iota(nearest.begin(),nearest.end(),0);
    std::sort(nearest.begin(),nearest.end(),[&](const std::size_t left,const std::size_t right){
        return cv::norm(markerCenter(left)-faceCenter)<cv::norm(markerCenter(right)-faceCenter);
    });
    nearest.resize(9);

    int selectedCenters=0;
    for(const std::size_t index:nearest){
        if(ids[index]<0 || ids[index]>=54){
            updateCubeScanStatus("A detected marker is outside the CubeNet ID range 0-53");
            return;
        }
        selectedCenters+=ids[index]<6 ? 1 : 0;
    }
    if(selectedCenters!=1){
        updateCubeScanStatus("Show one face more directly; markers from another center are too close");
        return;
    }

    const auto &centerCorners=corners[centerIndex];
    const cv::Point2f xAxis=(centerCorners[1]+centerCorners[2])
        -(centerCorners[0]+centerCorners[3]);
    const cv::Point2f yAxis=(centerCorners[2]+centerCorners[3])
        -(centerCorners[0]+centerCorners[1]);
    if(cv::norm(xAxis)<1.0 || cv::norm(yAxis)<1.0){
        updateCubeScanStatus("Center marker is too small to determine face orientation");
        return;
    }

    struct ProjectedMarker{
        std::size_t index;
        float x;
        float y;
    };
    std::vector<ProjectedMarker> projected;
    projected.reserve(9);
    for(const std::size_t index:nearest){
        const cv::Point2f offset=markerCenter(index)-faceCenter;
        projected.push_back({index,
                             offset.dot(xAxis)/xAxis.dot(xAxis),
                             offset.dot(yAxis)/yAxis.dot(yAxis)});
    }
    std::sort(projected.begin(),projected.end(),[](const auto &left,const auto &right){
        return left.y<right.y;
    });
    for(std::size_t row=0;row<3;++row){
        const auto begin=projected.begin()+static_cast<std::ptrdiff_t>(row*3);
        std::sort(begin,begin+3,[](const auto &left,const auto &right){
            return left.x<right.x;
        });
    }

    if(projected[4].index!=centerIndex){
        updateCubeScanStatus("Face is too tilted; keep all nine stickers square to the camera");
        return;
    }

    const int face=ids[centerIndex];
    for(std::size_t position=0;position<projected.size();++position)
        scannedCubeFaces[face][position]=ids[projected[position].index];
    cubeWidget->setCubeState(scannedCubeFaces);
    if(cubeScanIsValid()){
        captureCubeFaceButton->setChecked(false);
        if(normalizeCubeFaceOrientations()){
            cubeWidget->setCubeState(scannedCubeFaces);
            updateCubeScanStatus("All faces scanned and oriented - calculating solution");
            startCubeSolver();
        }else{
            updateCubeScanStatus("Markers form an impossible cube; reset and rescan carefully");
        }
    }else{
        updateCubeScanStatus(QString("Scanned %1 face").arg(cubeFaceName(face)));
    }
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
        calibrationEnabled=true;
        calibrationStatus->setText(QString(" Calibrated (RMS %1 px)").arg(error,0,'f',3));
        toggleCalibrationButton->setEnabled(true);
        toggleCalibrationButton->setText("Disable calibration");
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
        if(!use5x5)
            lastMarkerIds=ids;
        if(!use5x5)
            processAutomaticCubeScan(corners,ids);
        if(!ids.empty()){
            if(use5x5){
                cv::aruco::drawDetectedMarkers(frame,corners,ids);
            }else{
                cv::Mat tint=frame.clone();
                for(std::size_t index=0;index<ids.size();++index){
                    std::vector<cv::Point> polygon;
                    polygon.reserve(corners[index].size());
                    for(const cv::Point2f &corner:corners[index])
                        polygon.emplace_back(cvRound(corner.x),cvRound(corner.y));
                    cv::fillConvexPoly(tint,polygon,cubeFaceColor(ids[index]),cv::LINE_AA);
                }
                cv::addWeighted(tint,0.30,frame,0.70,0.0,frame);

                for(std::size_t index=0;index<ids.size();++index){
                    const cv::Scalar color=cubeFaceColor(ids[index]);
                    for(std::size_t corner=0;corner<corners[index].size();++corner)
                        cv::line(frame,
                                 corners[index][corner],
                                 corners[index][(corner+1)%corners[index].size()],
                                 color,3,cv::LINE_AA);

                    const std::string label=std::string(cubeFaceName(ids[index]))
                        +" "+std::to_string(ids[index]);
                    const cv::Point2f labelPosition=corners[index][0]+cv::Point2f(0.0F,-8.0F);
                    cv::putText(frame,label,labelPosition,cv::FONT_HERSHEY_SIMPLEX,
                                0.65,cv::Scalar(0,0,0),4,cv::LINE_AA);
                    cv::putText(frame,label,labelPosition,cv::FONT_HERSHEY_SIMPLEX,
                                0.65,color,2,cv::LINE_AA);
                }
            }

            if(cameraCalibrated && calibrationEnabled){
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
