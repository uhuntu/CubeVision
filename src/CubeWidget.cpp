#include "CubeWidget.h"
#include <QPainter>
#include <QQuaternion>
#include <QVector3D>
#include <algorithm>
#include <cmath>
#include <vector>

namespace {
QColor stickerColor(const int markerId){
    if(markerId<0)
        return QColor(75,78,85);
    switch(markerId%6){
    case 0: return QColor(250,220,25);  // U: yellow
    case 1: return QColor(245,245,245); // D: white
    case 2: return QColor(35,90,220);   // L: blue
    case 3: return QColor(25,175,75);   // R: green
    case 4: return QColor(220,45,45);   // F: red
    default:return QColor(245,125,25);  // B: orange
    }
}

int moveFace(const QString &move){
    if(move.isEmpty()) return -1;
    if(move.front()=='U') return 0;
    if(move.front()=='D') return 1;
    if(move.front()=='L') return 2;
    if(move.front()=='R') return 3;
    if(move.front()=='F') return 4;
    return 5;
}

QString faceName(const int face){
    static const std::array<QString,6> names={
        "UP","DOWN","LEFT","RIGHT","FRONT","BACK"
    };
    return face>=0 && face<static_cast<int>(names.size()) ? names[face] : QString();
}

QString turnDirection(const QString &move){
    if(move.endsWith('2'))
        return "TURN 180\u00b0";
    if(move.endsWith('\''))
        return "\u21ba  COUNTERCLOCKWISE";
    return "\u21bb  CLOCKWISE";
}

QVector3D faceNormal(const int face){
    switch(face){
    case 0: return {0,1,0};
    case 1: return {0,-1,0};
    case 2: return {-1,0,0};
    case 3: return {1,0,0};
    case 4: return {0,0,1};
    default:return {0,0,-1};
    }
}

QVector3D facePoint(const int face,const int row,const int column){
    const float r=static_cast<float>(row);
    const float c=static_cast<float>(column);
    switch(face){
    case 0: return {-1.5F+c, 1.5F,-1.5F+r};
    case 1: return {-1.5F+c,-1.5F, 1.5F-r};
    case 2: return {-1.5F,1.5F-r,-1.5F+c};
    case 3: return { 1.5F,1.5F-r, 1.5F-c};
    case 4: return {-1.5F+c,1.5F-r, 1.5F};
    default:return {1.5F-c,1.5F-r,-1.5F};
    }
}

bool stickerIsInLayer(const int face,const int row,const int column,
                      const QVector3D &axis){
    const QVector3D center=facePoint(face,row,column)
        +facePoint(face,row+1,column+1);
    return QVector3D::dotProduct(center*0.5F,axis)>0.5F;
}

float animationAngle(const QString &move,const qint64 elapsedMilliseconds){
    constexpr qint64 TurnDuration=900;
    constexpr qint64 HoldDuration=650;
    constexpr qint64 ResetPause=250;
    constexpr qint64 CycleDuration=TurnDuration+HoldDuration+ResetPause;
    const qint64 cycleTime=elapsedMilliseconds%CycleDuration;
    if(cycleTime>=TurnDuration+HoldDuration)
        return 0.0F;

    float progress=cycleTime>=TurnDuration
        ? 1.0F
        : static_cast<float>(cycleTime)/static_cast<float>(TurnDuration);
    // Smooth acceleration and deceleration make the demonstrated turn easier
    // to follow than a constant-speed rotation.
    progress=progress*progress*(3.0F-2.0F*progress);

    float degrees=move.endsWith('2') ? -180.0F : -90.0F;
    if(move.endsWith('\''))
        degrees=90.0F;
    return degrees*progress;
}

struct PaintedSticker{
    QPolygonF polygon;
    QColor color;
    float depth;
    bool active;
};
}

CubeWidget::CubeWidget(QWidget *parent):QWidget(parent){
    setMinimumSize(260,260);
    setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Expanding);
    animationTimer.setInterval(16);
    connect(&animationTimer,&QTimer::timeout,this,
            qOverload<>(&CubeWidget::update));
    clearCubeState();
}

void CubeWidget::setCubeState(const std::array<std::array<int,9>,6> &newFaces){
    faces=newFaces;
    update();
}

void CubeWidget::clearCubeState(){
    for(auto &face:faces)
        face.fill(-1);
    activeMove.clear();
    animationTimer.stop();
    update();
}

void CubeWidget::setActiveMove(const QString &move){
    if(activeMove==move && !move.isEmpty())
        return;
    activeMove=move;
    if(activeMove.isEmpty()){
        animationTimer.stop();
    }else{
        animationClock.restart();
        animationTimer.start();
    }
    update();
}

void CubeWidget::paintEvent(QPaintEvent *){
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(),QColor(30,32,38));

    const int activeFace=moveFace(activeMove);
    const QVector3D activeAxis=activeFace>=0 ? faceNormal(activeFace) : QVector3D();
    const float turnAngle=activeFace>=0
        ? animationAngle(activeMove,animationClock.elapsed())
        : 0.0F;
    const QQuaternion layerRotation=activeFace>=0
        ? QQuaternion::fromAxisAndAngle(activeAxis,turnAngle)
        : QQuaternion();
    const QQuaternion tiltX=QQuaternion::fromAxisAndAngle(1,0,0,22.0F);
    const QQuaternion tiltY=QQuaternion::fromAxisAndAngle(0,1,0,-28.0F);
    // Keep a stable world frame: +Y/U/yellow is always the top of the cube.
    // Only the requested layer moves; selecting a move must not rotate the
    // entire cube and change the user's spatial reference.
    const QQuaternion model=tiltX*tiltY;

    const QPointF center(width()*0.5,height()*0.47);
    const float distance=8.0F;
    const float scale=std::min(width(),height())*1.55F;
    auto project=[&](const QVector3D &point){
        const QVector3D transformed=model.rotatedVector(point);
        const float perspective=scale/(distance-transformed.z());
        return std::pair<QPointF,float>(
            QPointF(center.x()+transformed.x()*perspective,
                    center.y()-transformed.y()*perspective),
            transformed.z());
    };

    std::vector<PaintedSticker> stickers;
    for(int face=0;face<6;++face){
        for(int row=0;row<3;++row){
            for(int column=0;column<3;++column){
                const bool moving=activeFace>=0
                    && stickerIsInLayer(face,row,column,activeAxis);
                QVector3D normal=faceNormal(face);
                if(moving)
                    normal=layerRotation.rotatedVector(normal);
                normal=model.rotatedVector(normal);
                if(normal.z()<=0.02F)
                    continue;

                QPolygonF polygon;
                float depth=0.0F;
                for(const auto &[cornerRow,cornerColumn]:
                    {std::pair{row,column},std::pair{row,column+1},
                     std::pair{row+1,column+1},std::pair{row+1,column}}){
                    QVector3D corner=facePoint(face,cornerRow,cornerColumn);
                    if(moving)
                        corner=layerRotation.rotatedVector(corner);
                    const auto [point,z]=project(corner);
                    polygon<<point;
                    depth+=z;
                }
                QColor color=stickerColor(faces[face][row*3+column]);
                if(moving)
                    color=color.lighter(112);
                stickers.push_back({polygon,color,depth*0.25F,moving});
            }
        }
    }
    std::sort(stickers.begin(),stickers.end(),[](const auto &left,const auto &right){
        return left.depth<right.depth;
    });
    for(const auto &sticker:stickers){
        painter.setPen(QPen(sticker.active ? QColor(255,235,80) : QColor(15,16,20),
                            sticker.active ? 3.0 : 2.0,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin));
        painter.setBrush(sticker.color);
        painter.drawPolygon(sticker.polygon);
    }

    // Put the notation directly on the selected face.  The translucent badge
    // keeps it readable over any sticker colors while leaving the face visible.
    const bool activeFaceVisible=activeFace>=0
        && model.rotatedVector(faceNormal(activeFace)).z()>0.02F;
    if(activeFaceVisible){
        QPolygonF activePolygon;
        for(const auto &[row,column]:
            {std::pair{0,0},std::pair{0,3},std::pair{3,3},std::pair{3,0}}){
            const QVector3D point=layerRotation.rotatedVector(
                facePoint(activeFace,row,column));
            activePolygon<<project(point).first;
        }
        const QRectF faceBounds=activePolygon.boundingRect();
        const qreal badgeSize=std::clamp(
            std::min(faceBounds.width(),faceBounds.height())*0.34,54.0,84.0);
        const QRectF badge(faceBounds.center().x()-badgeSize*0.5,
                           faceBounds.center().y()-badgeSize*0.5,
                           badgeSize,badgeSize);
        painter.setPen(QPen(QColor(255,235,80),3.0));
        painter.setBrush(QColor(20,22,28,210));
        painter.drawEllipse(badge);

        QFont badgeFont=font();
        badgeFont.setBold(true);
        badgeFont.setPointSizeF(badgeSize*0.37);
        painter.setFont(badgeFont);
        painter.setPen(Qt::white);
        painter.drawText(badge,Qt::AlignCenter,activeMove.left(1));
    }

    painter.setPen(QColor(225,228,235));
    QFont titleFont=font();
    titleFont.setBold(true);
    titleFont.setPointSize(titleFont.pointSize()+2);
    painter.setFont(titleFont);
    painter.drawText(QRectF(0,8,width(),28),Qt::AlignCenter,
                     activeMove.isEmpty()
                         ? "Cube state"
                         : QString("%1  =  %2 FACE").arg(activeMove,faceName(activeFace)));
    if(activeFace>=0){
        QFont directionFont=font();
        directionFont.setBold(true);
        directionFont.setPointSize(directionFont.pointSize()+1);
        painter.setFont(directionFont);
        painter.setPen(QColor(255,235,80));
        painter.drawText(QRectF(0,38,width(),28),Qt::AlignCenter,
                         turnDirection(activeMove));
    }
}
