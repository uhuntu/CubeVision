#pragma once

#include <QElapsedTimer>
#include <QTimer>
#include <QWidget>
#include <array>

class CubeWidget: public QWidget{
    Q_OBJECT
public:
    explicit CubeWidget(QWidget *parent=nullptr);
    void setCubeState(const std::array<std::array<int,9>,6> &faces);
    void clearCubeState();
    void setActiveMove(const QString &move);
protected:
    void paintEvent(QPaintEvent *event) override;
private:
    std::array<std::array<int,9>,6> faces;
    QString activeMove;
    QElapsedTimer animationClock;
    QTimer animationTimer;
};
