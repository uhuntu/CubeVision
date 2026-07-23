#include "MainWindow.h"
#include "CrossSolver.h"
#include "CubeState.h"
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
#include <limits>
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

struct FaceMarkerSelection{
    std::size_t centerIndex=0;
    std::vector<std::size_t> indices;

    bool isValid() const{
        return indices.size()==9;
    }
};

struct FaceGridObservation{
    int face=-1;
    std::array<int,9> ids;
    std::vector<std::size_t> indices;

    FaceGridObservation(){
        ids.fill(-1);
    }

    int visibleCount() const{
        return static_cast<int>(std::count_if(ids.begin(),ids.end(),
                                              [](const int id){ return id>=0; }));
    }

    bool isValid() const{
        return face>=0 && visibleCount()>=5;
    }
};

cv::Point2f markerCenter(
    const std::vector<std::vector<cv::Point2f>> &corners,
    const std::size_t index){
    cv::Point2f center;
    for(const cv::Point2f &corner:corners[index])
        center+=corner;
    return center*(1.0F/static_cast<float>(corners[index].size()));
}

FaceMarkerSelection selectFaceMarkers(
    const std::vector<std::vector<cv::Point2f>> &corners,
    const std::vector<int> &ids){
    FaceMarkerSelection selection;
    if(ids.size()<9 || corners.size()!=ids.size())
        return selection;

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
    if(centerIndex==ids.size())
        return selection;

    const cv::Point2f faceCenter=markerCenter(corners,centerIndex);
    selection.indices.resize(ids.size());
    std::iota(selection.indices.begin(),selection.indices.end(),0);
    std::sort(selection.indices.begin(),selection.indices.end(),
              [&corners,&faceCenter](const std::size_t left,const std::size_t right){
        return cv::norm(markerCenter(corners,left)-faceCenter)
            <cv::norm(markerCenter(corners,right)-faceCenter);
    });
    selection.indices.resize(9);

    const int selectedCenters=static_cast<int>(std::count_if(
        selection.indices.begin(),selection.indices.end(),
        [&ids](const std::size_t index){
            return ids[index]>=0 && ids[index]<6;
        }));
    if(selectedCenters!=1){
        selection.indices.clear();
        return selection;
    }

    selection.centerIndex=centerIndex;
    return selection;
}

FaceGridObservation orderCompleteFaceGrid(
    const std::vector<std::vector<cv::Point2f>> &corners,
    const std::vector<int> &ids){
    FaceGridObservation observation;
    const FaceMarkerSelection selection=selectFaceMarkers(corners,ids);
    if(!selection.isValid())
        return observation;

    const std::size_t centerIndex=selection.centerIndex;
    const auto &centerCorners=corners[centerIndex];
    const cv::Point2f faceCenter=markerCenter(corners,centerIndex);
    const cv::Point2f xAxis=(centerCorners[1]+centerCorners[2])
        -(centerCorners[0]+centerCorners[3]);
    const cv::Point2f yAxis=(centerCorners[2]+centerCorners[3])
        -(centerCorners[0]+centerCorners[1]);
    if(cv::norm(xAxis)<1.0 || cv::norm(yAxis)<1.0)
        return observation;

    struct ProjectedMarker{
        std::size_t index;
        float x;
        float y;
    };
    std::vector<ProjectedMarker> projected;
    projected.reserve(9);
    for(const std::size_t index:selection.indices){
        if(ids[index]<0 || ids[index]>=54)
            return observation;
        const cv::Point2f offset=markerCenter(corners,index)-faceCenter;
        projected.push_back({index,
                             offset.dot(xAxis)/xAxis.dot(xAxis),
                             offset.dot(yAxis)/yAxis.dot(yAxis)});
    }

    // Perspective changes distances but preserves the ordering of the three
    // rows and the three markers within each row.  Ordering avoids rejecting
    // a complete tilted face because an outer marker crossed a fixed limit.
    std::sort(projected.begin(),projected.end(),[](const auto &left,const auto &right){
        return left.y<right.y;
    });
    for(std::size_t row=0;row<3;++row){
        const auto begin=projected.begin()+static_cast<std::ptrdiff_t>(row*3);
        std::sort(begin,begin+3,[](const auto &left,const auto &right){
            return left.x<right.x;
        });
    }
    if(projected[4].index!=centerIndex)
        return observation;

    observation.face=ids[centerIndex];
    for(std::size_t position=0;position<projected.size();++position){
        observation.ids[position]=ids[projected[position].index];
        observation.indices.push_back(projected[position].index);
    }
    return observation;
}

FaceGridObservation observeFaceGrid(
    const std::vector<std::vector<cv::Point2f>> &corners,
    const std::vector<int> &ids){
    FaceGridObservation observation;
    if(corners.size()!=ids.size() || ids.empty())
        return observation;

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
    if(centerIndex==ids.size())
        return observation;

    const auto &centerCorners=corners[centerIndex];
    const cv::Point2f faceCenter=markerCenter(corners,centerIndex);
    const cv::Point2f xAxis=(centerCorners[1]+centerCorners[2])
        -(centerCorners[0]+centerCorners[3]);
    const cv::Point2f yAxis=(centerCorners[2]+centerCorners[3])
        -(centerCorners[0]+centerCorners[1]);
    if(cv::norm(xAxis)<1.0 || cv::norm(yAxis)<1.0)
        return observation;

    // Adjacent sticker centers are approximately half a center-marker axis
    // from the origin.  Classifying in this normalized coordinate system
    // lets observations from moving camera frames fill the same 3x3 cells.
    constexpr float CellCenterDistance=0.55F;
    constexpr float CellBoundary=0.25F;
    constexpr float MaximumFaceCoordinate=0.90F;
    std::array<float,9> bestScores;
    bestScores.fill(std::numeric_limits<float>::max());

    for(std::size_t index=0;index<ids.size();++index){
        if(ids[index]<0 || ids[index]>=54)
            continue;
        if(ids[index]<6 && index!=centerIndex)
            continue;

        const cv::Point2f offset=markerCenter(corners,index)-faceCenter;
        const float x=offset.dot(xAxis)/xAxis.dot(xAxis);
        const float y=offset.dot(yAxis)/yAxis.dot(yAxis);
        if(std::abs(x)>MaximumFaceCoordinate || std::abs(y)>MaximumFaceCoordinate)
            continue;

        const int column=x<-CellBoundary ? 0 : x>CellBoundary ? 2 : 1;
        const int row=y<-CellBoundary ? 0 : y>CellBoundary ? 2 : 1;
        const int position=row*3+column;
        if((position==4)!=(index==centerIndex))
            continue;

        const float expectedX=(column-1)*CellCenterDistance;
        const float expectedY=(row-1)*CellCenterDistance;
        const float score=std::abs(x-expectedX)+std::abs(y-expectedY);
        if(score>=bestScores[position])
            continue;
        bestScores[position]=score;
        observation.ids[position]=ids[index];
    }

    observation.face=ids[centerIndex];
    for(std::size_t position=0;position<observation.ids.size();++position){
        if(observation.ids[position]<0)
            continue;
        for(std::size_t index=0;index<ids.size();++index){
            if(ids[index]==observation.ids[position]){
                observation.indices.push_back(index);
                break;
            }
        }
    }
    return observation;
}

void mergeCubeMarkerDetections(
    std::vector<std::vector<cv::Point2f>> &corners,
    std::vector<int> &ids,
    const std::vector<std::vector<cv::Point2f>> &recoveredCorners,
    const std::vector<int> &recoveredIds){
    for(std::size_t recovered=0;recovered<recoveredIds.size();++recovered){
        const int id=recoveredIds[recovered];
        if(id<0 || id>=54)
            continue;
        const auto existing=std::find(ids.begin(),ids.end(),id);
        if(existing==ids.end()){
            ids.push_back(id);
            corners.push_back(recoveredCorners[recovered]);
            continue;
        }

        const std::size_t index=
            static_cast<std::size_t>(std::distance(ids.begin(),existing));
        if(std::abs(cv::contourArea(recoveredCorners[recovered]))
            >std::abs(cv::contourArea(corners[index])))
            corners[index]=recoveredCorners[recovered];
    }
}

void detectCubeMarkers(
    const cv::Mat &frame,
    const cv::aruco::ArucoDetector &detector,
    std::vector<std::vector<cv::Point2f>> &corners,
    std::vector<int> &ids){
    detector.detectMarkers(frame,corners,ids);

    std::vector<std::vector<cv::Point2f>> validCorners;
    std::vector<int> validIds;
    validCorners.reserve(corners.size());
    validIds.reserve(ids.size());
    for(std::size_t index=0;index<ids.size();++index){
        if(ids[index]<0 || ids[index]>=54)
            continue;
        validIds.push_back(ids[index]);
        validCorners.push_back(corners[index]);
    }
    corners=std::move(validCorners);
    ids=std::move(validIds);

    const auto centerVisible=[&ids]{
        return std::any_of(ids.begin(),ids.end(),[](const int id){
            return id>=0 && id<6;
        });
    };
    if(ids.size()>=9 && centerVisible())
        return;

    // Recovery is deliberately lower-frequency than the normal detector.
    // Running multiple full-resolution ArUco passes every camera frame makes
    // the UI lag precisely when a marker is difficult to read.
    static int recoveryFrame=0;
    constexpr int RecoveryInterval=4;
    if(++recoveryFrame%RecoveryInterval!=0)
        return;

    cv::Mat gray;
    cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
    cv::Rect recoveryRegion(0,0,frame.cols,frame.rows);
    double recoveryScale=1.0;
    if(corners.size()>=4){
        std::vector<cv::Point2f> detectedPoints;
        for(const auto &markerCorners:corners)
            detectedPoints.insert(detectedPoints.end(),
                                  markerCorners.begin(),markerCorners.end());
        const cv::Rect detectedBounds=cv::boundingRect(detectedPoints);
        const int padding=cvRound(
            std::max(detectedBounds.width,detectedBounds.height)*0.65);
        const cv::Rect expanded(detectedBounds.x-padding,
                                detectedBounds.y-padding,
                                detectedBounds.width+padding*2,
                                detectedBounds.height+padding*2);
        recoveryRegion=expanded&cv::Rect(0,0,frame.cols,frame.rows);
        recoveryScale=2.0;
    }
    if(recoveryRegion.width<20 || recoveryRegion.height<20)
        return;

    cv::Mat recoveryImage;
    cv::resize(gray(recoveryRegion),recoveryImage,cv::Size(),
               recoveryScale,recoveryScale,cv::INTER_CUBIC);
    static const cv::Ptr<cv::CLAHE> clahe=cv::createCLAHE(2.5,cv::Size(8,8));
    cv::Mat enhanced;
    clahe->apply(recoveryImage,enhanced);

    static const cv::aruco::ArucoDetector recoveryDetector=[]{
        cv::aruco::DetectorParameters recovery;
        recovery.adaptiveThreshWinSizeMax=53;
        recovery.adaptiveThreshWinSizeStep=5;
        recovery.minMarkerPerimeterRate=0.008;
        recovery.minCornerDistanceRate=0.02;
        recovery.minMarkerDistanceRate=0.03;
        recovery.cornerRefinementMethod=cv::aruco::CORNER_REFINE_SUBPIX;
        recovery.perspectiveRemovePixelPerCell=8;
        recovery.minOtsuStdDev=3.0;
        recovery.errorCorrectionRate=0.8;
        return cv::aruco::ArucoDetector(
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100),
            recovery);
    }();
    std::vector<std::vector<cv::Point2f>> recoveredCorners;
    std::vector<int> recoveredIds;
    recoveryDetector.detectMarkers(enhanced,recoveredCorners,recoveredIds);
    for(auto &markerCorners:recoveredCorners){
        for(cv::Point2f &point:markerCorners){
            point.x=static_cast<float>(point.x/recoveryScale+recoveryRegion.x);
            point.y=static_cast<float>(point.y/recoveryScale+recoveryRegion.y);
        }
    }
    mergeCubeMarkerDetections(corners,ids,recoveredCorners,recoveredIds);
}

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

bool cubeMarkerCubiesAreValid(const CubeFaces &faces){
    // Exact sticker locations from the printed CubeNet.  Unlike colors, these
    // IDs distinguish every sticker and therefore remove the ambiguity when
    // choosing among the four possible rotations of each scanned face.
    static constexpr CubeFaces solvedMarkers={{
        {{42,48, 6,36, 0,12,30,24,18}}, // U
        {{43,49, 7,37, 1,13,31,25,19}}, // D
        {{44,50, 8,38, 2,14,32,26,20}}, // L
        {{45,51, 9,39, 3,15,33,27,21}}, // R
        {{46,52,10,40, 4,16,34,28,22}}, // F
        {{47,53,11,41, 5,17,35,29,23}}  // B
    }};
    struct Facelet{ int face; int position; };
    static constexpr Facelet cornerFacelets[8][3]={
        {{0,8},{3,0},{4,2}},{{0,6},{4,0},{2,2}},
        {{0,0},{2,0},{5,2}},{{0,2},{5,0},{3,2}},
        {{1,2},{4,8},{3,6}},{{1,0},{2,8},{4,6}},
        {{1,6},{5,8},{2,6}},{{1,8},{3,8},{5,6}}
    };
    static constexpr Facelet edgeFacelets[12][2]={
        {{0,5},{3,1}},{{0,7},{4,1}},{{0,3},{2,1}},{{0,1},{5,1}},
        {{1,5},{3,7}},{{1,1},{4,7}},{{1,3},{2,7}},{{1,7},{5,7}},
        {{4,5},{3,3}},{{4,3},{2,5}},{{5,5},{2,3}},{{5,3},{3,5}}
    };
    auto marker=[&faces](const Facelet facelet){
        return faces[facelet.face][facelet.position];
    };
    auto solvedMarker=[](const Facelet facelet){
        return solvedMarkers[facelet.face][facelet.position];
    };

    std::array<bool,8> cornersSeen{};
    for(const auto &position:cornerFacelets){
        std::array<int,3> observed={marker(position[0]),marker(position[1]),
                                    marker(position[2])};
        std::sort(observed.begin(),observed.end());
        int cubie=-1;
        for(int candidate=0;candidate<8;++candidate){
            std::array<int,3> expected={solvedMarker(cornerFacelets[candidate][0]),
                                        solvedMarker(cornerFacelets[candidate][1]),
                                        solvedMarker(cornerFacelets[candidate][2])};
            std::sort(expected.begin(),expected.end());
            if(observed==expected){
                cubie=candidate;
                break;
            }
        }
        if(cubie<0 || cornersSeen[cubie])
            return false;
        cornersSeen[cubie]=true;
    }

    std::array<bool,12> edgesSeen{};
    for(const auto &position:edgeFacelets){
        std::array<int,2> observed={marker(position[0]),marker(position[1])};
        std::sort(observed.begin(),observed.end());
        int cubie=-1;
        for(int candidate=0;candidate<12;++candidate){
            std::array<int,2> expected={solvedMarker(edgeFacelets[candidate][0]),
                                        solvedMarker(edgeFacelets[candidate][1])};
            std::sort(expected.begin(),expected.end());
            if(observed==expected){
                cubie=candidate;
                break;
            }
        }
        if(cubie<0 || edgesSeen[cubie])
            return false;
        edgesSeen[cubie]=true;
    }
    for(int face=0;face<6;++face)
        if(faces[face][4]!=face)
            return false;
    return true;
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

    solveMethodSelector=new QComboBox(coachToolbar);
    solveMethodSelector->addItem("White Cross");
    solveMethodSelector->addItem("Quick Solve");
    solveMethodSelector->setToolTip(
        "White Cross stops after the four white edges align with the side centers");
    coachToolbar->addWidget(solveMethodSelector);

    solveCubeButton=new QPushButton("Solve",coachToolbar);
    solveCubeButton->setEnabled(false);
    coachToolbar->addWidget(solveCubeButton);

    previousMoveButton=new QPushButton("Previous",coachToolbar);
    previousMoveButton->setEnabled(false);
    previousMoveButton->setVisible(false);
    coachToolbar->addWidget(previousMoveButton);

    nextMoveButton=new QPushButton("Verify now",coachToolbar);
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
        resetPendingCubeScan();
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
        resetPendingCubeScan();
        solutionMoves.clear();
        solutionMoveIndex=0;
        crossSolutionActive=false;
        solveMethodSelector->setEnabled(true);
        stableVerificationFrames=0;
        missedVerificationFrames=0;
        stableVerificationIds.clear();
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
    resetPendingCubeScan();
    cubeWidget->setCubeState(scannedCubeFaces);
    updateCubeScanStatus();
    captureCubeFaceButton->setChecked(true);

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
        if(cubeMarkerCubiesAreValid(candidate) && cubeStateIsSolvable(candidate)){
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

    solutionMoves.clear();
    solutionMoveIndex=0;
    stableVerificationFrames=0;
    missedVerificationFrames=0;
    stableVerificationIds.clear();
    solveCubeButton->setEnabled(false);
    solveMethodSelector->setEnabled(false);
    previousMoveButton->setEnabled(false);
    nextMoveButton->setEnabled(false);

    if(solveMethodSelector->currentIndex()==0){
        crossSolutionActive=true;
        solutionStatus->setText("Calculating a shortest white-cross solution...");
        const auto solution=solveWhiteCross(scannedCubeFaces);
        if(!solution){
            solutionStatus->setText("Could not identify all four white cross edges");
            solveCubeButton->setEnabled(true);
            solveMethodSelector->setEnabled(true);
            return;
        }
        if(solution->isEmpty()){
            solutionStatus->setText("White cross is already complete");
            cubeWidget->setActiveMove(QString());
            solveCubeButton->setEnabled(true);
            solveMethodSelector->setEnabled(true);
            return;
        }
        solutionMoves=*solution;
        solutionStatus->setToolTip("White cross solution: "+solutionMoves.join(' '));
        updateSolutionStep();
        return;
    }

    crossSolutionActive=false;
    QString program=QDir(QCoreApplication::applicationDirPath()).filePath("CubeSolver");
#ifdef Q_OS_WIN
    program+=".exe";
#endif
    if(!QFileInfo::exists(program)){
        solutionStatus->setText("Cube solver executable is missing");
        solveCubeButton->setEnabled(true);
        solveMethodSelector->setEnabled(true);
        return;
    }

    solutionStatus->setText("Checking cube state and calculating a solution...");
    cubeSolver->start(program,{"-p","-m"});
    if(!cubeSolver->waitForStarted(1000)){
        solutionStatus->setText("Could not start the cube solver");
        solveCubeButton->setEnabled(true);
        solveMethodSelector->setEnabled(true);
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
        solveMethodSelector->setEnabled(true);
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
        solveMethodSelector->setEnabled(true);
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
        solveMethodSelector->setEnabled(true);
        return;
    }
    solutionMoveIndex=0;
    solutionStatus->setToolTip("Full solution: "+solutionMoves.join(' '));
    updateSolutionStep();
}

void MainWindow::updateSolutionStep(){
    stableVerificationFrames=0;
    missedVerificationFrames=0;
    stableVerificationIds.clear();
    nextMoveButton->setText("Verify now");
    if(solutionMoves.isEmpty()){
        previousMoveButton->setEnabled(false);
        nextMoveButton->setEnabled(false);
        return;
    }
    if(solutionMoveIndex>=solutionMoves.size()){
        solutionStatus->setText(crossSolutionActive
            ? "White cross complete - all four edges align with their side centers"
            : "Solution complete - your cube should now be solved");
        cubeWidget->setActiveMove(QString());
        previousMoveButton->setEnabled(false);
        nextMoveButton->setEnabled(false);
        solveCubeButton->setEnabled(true);
        solveMethodSelector->setEnabled(true);
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
        QString("Step %1/%2:  %3  - turn %4 %5; then show %6 (auto verify)")
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

void MainWindow::processAutomaticMoveVerification(const std::vector<int> &ids){
    constexpr int RequiredStableObservations=5;
    constexpr int AllowedMissedFrames=6;
    if(solutionMoveIndex<0 || solutionMoveIndex>=solutionMoves.size()){
        stableVerificationFrames=0;
        missedVerificationFrames=0;
        stableVerificationIds.clear();
        return;
    }

    auto missObservation=[this]{
        if(++missedVerificationFrames<=AllowedMissedFrames)
            return;
        stableVerificationFrames=0;
        stableVerificationIds.clear();
        nextMoveButton->setText("Verify now");
    };
    if(ids.size()!=9){
        missObservation();
        return;
    }

    const QString move=solutionMoves[solutionMoveIndex];
    const int verificationFace=verificationFaceForMove(faceIndex(move.front()));
    if(std::find(ids.begin(),ids.end(),verificationFace)==ids.end()){
        missObservation();
        return;
    }
    missedVerificationFrames=0;

    std::array<int,9> observed;
    std::copy(ids.begin(),ids.end(),observed.begin());
    if(sameFaceMarkers(observed,scannedCubeFaces[verificationFace])){
        // The requested face is visible but the move has not happened yet.
        // Keep the step instruction on screen instead of repeatedly reporting
        // an unchanged state.
        stableVerificationFrames=0;
        stableVerificationIds.clear();
        nextMoveButton->setText("Verify now");
        return;
    }

    std::vector<int> signature=ids;
    std::sort(signature.begin(),signature.end());
    if(signature!=stableVerificationIds){
        stableVerificationIds=std::move(signature);
        stableVerificationFrames=1;
        nextMoveButton->setText(
            QString("Auto verify %1/%2")
                .arg(stableVerificationFrames).arg(RequiredStableObservations));
        return;
    }
    ++stableVerificationFrames;
    nextMoveButton->setText(
        QString("Auto verify %1/%2")
            .arg(stableVerificationFrames).arg(RequiredStableObservations));
    if(stableVerificationFrames<RequiredStableObservations)
        return;

    stableVerificationFrames=0;
    missedVerificationFrames=0;
    stableVerificationIds.clear();
    nextMoveButton->setText("Verify now");
    verifySolutionMove();
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

void MainWindow::resetPendingCubeScan(){
    missedCubeScanFrames=0;
    pendingCubeScanFace=-1;
    pendingCubeScanIds.fill(-1);
    pendingCubeScanCounts.fill(0);
    pendingCubeScanCenterCorners.clear();
}

void MainWindow::processAutomaticCubeScan(
    const std::vector<std::vector<cv::Point2f>> &corners,
    const std::vector<int> &ids){
    constexpr int RequiredObservationsPerMarker=2;
    constexpr int AllowedMissedFrames=12;
    if(!automaticCubeScan)
        return;

    FaceGridObservation observation=orderCompleteFaceGrid(corners,ids);
    if(!observation.isValid())
        observation=observeFaceGrid(corners,ids);
    const bool centerObserved=observation.face>=0;
    std::vector<cv::Point2f> observedCenterCorners;
    if(centerObserved){
        const auto center=std::find(ids.begin(),ids.end(),observation.face);
        if(center!=ids.end())
            observedCenterCorners=corners[
                static_cast<std::size_t>(std::distance(ids.begin(),center))];
    }else if(pendingCubeScanFace>=0 && pendingCubeScanCenterCorners.size()==4){
        // Reuse a recent center pose while the cube is held steady.  The
        // synthetic center anchors the grid only; it is removed from the
        // observation so a missing center is never falsely confirmed.
        auto anchoredCorners=corners;
        auto anchoredIds=ids;
        anchoredCorners.push_back(pendingCubeScanCenterCorners);
        anchoredIds.push_back(pendingCubeScanFace);
        observation=observeFaceGrid(anchoredCorners,anchoredIds);
        if(observation.face==pendingCubeScanFace){
            observation.ids[4]=-1;
            observation.indices.erase(
                std::remove(observation.indices.begin(),observation.indices.end(),
                            anchoredIds.size()-1),
                observation.indices.end());
        }
    }

    if(!observation.isValid()){
        if(++missedCubeScanFrames>AllowedMissedFrames)
            resetPendingCubeScan();
        const bool centerVisible=std::any_of(ids.begin(),ids.end(),[](const int id){
            return id>=0 && id<6;
        });
        int remainingFace=-1;
        int remainingFaces=0;
        for(std::size_t face=0;face<scannedCubeFaces.size();++face){
            if(scannedCubeFaces[face][0]>=0)
                continue;
            remainingFace=static_cast<int>(face);
            ++remainingFaces;
        }
        if(centerVisible){
            updateCubeScanStatus(
                QString("Auto scan: align face grid (%1 markers detected)").arg(ids.size()));
        }else if(remainingFaces==1){
            updateCubeScanStatus(
                QString("Auto scan: %1 center marker missing (%2 markers detected)")
                    .arg(cubeFaceName(remainingFace))
                    .arg(ids.size()));
        }else{
            updateCubeScanStatus(
                QString("Auto scan: center marker missing (%1 markers detected)")
                    .arg(ids.size()));
        }
        return;
    }
    missedCubeScanFrames=0;

    if(scannedCubeFaces[observation.face][0]>=0){
        resetPendingCubeScan();
        updateCubeScanStatus(
            QString("%1 face already scanned - show a different face")
                .arg(cubeFaceName(observation.face)));
        return;
    }

    if(pendingCubeScanFace!=observation.face){
        resetPendingCubeScan();
        pendingCubeScanFace=observation.face;
    }
    if(centerObserved)
        pendingCubeScanCenterCorners=std::move(observedCenterCorners);

    auto markerAlreadyScanned=[this](const int markerId){
        return std::any_of(
            scannedCubeFaces.begin(),scannedCubeFaces.end(),
            [markerId](const auto &face){
                return std::find(face.begin(),face.end(),markerId)!=face.end();
            });
    };
    for(std::size_t position=0;position<observation.ids.size();++position){
        const int observedId=observation.ids[position];
        if(observedId<0 || markerAlreadyScanned(observedId))
            continue;

        // A marker can move between classified cells as perspective changes.
        // Keep only its newest cell so temporal accumulation can never create
        // two copies of one physical sticker on the same face.
        for(std::size_t other=0;other<pendingCubeScanIds.size();++other){
            if(other==position || pendingCubeScanIds[other]!=observedId)
                continue;
            pendingCubeScanIds[other]=-1;
            pendingCubeScanCounts[other]=0;
        }

        if(pendingCubeScanIds[position]!=observedId){
            pendingCubeScanIds[position]=observedId;
            pendingCubeScanCounts[position]=1;
        }else if(pendingCubeScanCounts[position]<RequiredObservationsPerMarker){
            ++pendingCubeScanCounts[position];
        }
    }

    const int collected=static_cast<int>(std::count_if(
        pendingCubeScanIds.begin(),pendingCubeScanIds.end(),
        [](const int id){ return id>=0; }));
    const int confirmed=static_cast<int>(std::count_if(
        pendingCubeScanCounts.begin(),pendingCubeScanCounts.end(),
        [](const int count){ return count>=RequiredObservationsPerMarker; }));
    if(confirmed<9){
        updateCubeScanStatus(
            QString("Collecting %1 face: %2/9 found, %3/9 confirmed")
                .arg(cubeFaceName(observation.face))
                .arg(collected)
                .arg(confirmed));
        return;
    }

    const int face=pendingCubeScanFace;
    const std::array<int,9> markers=pendingCubeScanIds;
    resetPendingCubeScan();
    commitCubeFace(face,markers);
}

void MainWindow::captureCubeFace(
    const std::vector<std::vector<cv::Point2f>> &corners,
    const std::vector<int> &ids){
    if(ids.size()<9){
        updateCubeScanStatus(
            QString("Show one complete face: at least 9 markers required (%1 found)")
                .arg(ids.size()));
        return;
    }

    const FaceGridObservation observation=orderCompleteFaceGrid(corners,ids);
    if(!observation.isValid()){
        const bool centerVisible=std::any_of(ids.begin(),ids.end(),[](const int id){
            return id>=0 && id<6;
        });
        if(centerVisible)
            updateCubeScanStatus(
                "Show one face more directly; adjacent face centers are too close");
        else
            updateCubeScanStatus("No center marker found (expected an ID from 0 to 5)");
        return;
    }
    commitCubeFace(observation.face,observation.ids);
}

void MainWindow::commitCubeFace(
    const int face,
    const std::array<int,9> &markers){
    std::array<bool,54> usedIds{};
    for(std::size_t existingFace=0;existingFace<scannedCubeFaces.size();++existingFace){
        if(static_cast<int>(existingFace)==face)
            continue;
        for(const int id:scannedCubeFaces[existingFace]){
            if(id>=0 && id<static_cast<int>(usedIds.size()))
                usedIds[id]=true;
        }
    }
    for(const int id:markers){
        if(id<0 || id>=static_cast<int>(usedIds.size()) || usedIds[id]){
            resetPendingCubeScan();
            updateCubeScanStatus(
                QString("Rejected %1 face: marker ID %2 is duplicated; hold one face steady")
                    .arg(cubeFaceName(face))
                    .arg(id));
            return;
        }
        usedIds[id]=true;
    }

    scannedCubeFaces[face]=markers;
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
        const bool allFacesPresent=std::all_of(
            scannedCubeFaces.begin(),scannedCubeFaces.end(),
            [](const auto &scannedFace){ return scannedFace[0]>=0; });
        if(allFacesPresent){
            scannedCubeFaces[face].fill(-1);
            cubeWidget->setCubeState(scannedCubeFaces);
            resetPendingCubeScan();
            updateCubeScanStatus(
                QString("Rejected %1 face: marker set conflicts with earlier scans; show it again")
                    .arg(cubeFaceName(face)));
            return;
        }
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
        static const cv::aruco::DetectorParameters parameters=[]{
            cv::aruco::DetectorParameters tuned;
            tuned.adaptiveThreshWinSizeMax=31;
            tuned.adaptiveThreshWinSizeStep=7;
            tuned.minMarkerPerimeterRate=0.015;
            tuned.minCornerDistanceRate=0.03;
            tuned.minMarkerDistanceRate=0.05;
            tuned.cornerRefinementMethod=cv::aruco::CORNER_REFINE_SUBPIX;
            return tuned;
        }();
        static cv::aruco::ArucoDetector detector5x5(
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100),parameters);
        static cv::aruco::ArucoDetector detector4x4(
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100),parameters);

        const bool use5x5=dictionarySelector->currentIndex()==0;
        const auto &detector=use5x5 ? detector5x5 : detector4x4;
        const float markerSize=use5x5 ? Marker5x5SizeMeters : Marker4x4SizeMeters;

        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        if(use5x5)
            detector.detectMarkers(frame,corners,ids);
        else
            detectCubeMarkers(frame,detector,corners,ids);
        if(!use5x5){
            lastMarkerIds=ids;
            processAutomaticMoveVerification(ids);
        }
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

                if(automaticCubeScan){
                    FaceGridObservation observation=orderCompleteFaceGrid(corners,ids);
                    if(!observation.isValid())
                        observation=observeFaceGrid(corners,ids);
                    if(observation.isValid()){
                        std::vector<cv::Point2f> facePoints;
                        for(const std::size_t index:observation.indices)
                            facePoints.insert(facePoints.end(),
                                              corners[index].begin(),
                                              corners[index].end());

                        std::vector<cv::Point2f> faceHull;
                        cv::convexHull(facePoints,faceHull);
                        std::vector<cv::Point> outline;
                        outline.reserve(faceHull.size());
                        for(const cv::Point2f &point:faceHull)
                            outline.emplace_back(cvRound(point.x),cvRound(point.y));
                        cv::polylines(frame,outline,true,cv::Scalar(0,255,0),6,cv::LINE_AA);

                        const std::string targetLabel=std::string("TARGET ")
                            +cubeFaceName(observation.face)+" FACE";
                        const cv::Point labelPosition=outline.empty()
                            ? cv::Point(20,50)
                            : outline.front()+cv::Point(0,-18);
                        cv::putText(frame,targetLabel,labelPosition,
                                    cv::FONT_HERSHEY_SIMPLEX,0.85,
                                    cv::Scalar(0,0,0),5,cv::LINE_AA);
                        cv::putText(frame,targetLabel,labelPosition,
                                    cv::FONT_HERSHEY_SIMPLEX,0.85,
                                    cv::Scalar(0,255,0),2,cv::LINE_AA);
                    }
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
