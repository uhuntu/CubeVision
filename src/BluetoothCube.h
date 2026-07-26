#pragma once

#include "CubeState.h"
#include <QObject>

#ifdef CUBEVISION_HAS_BLUETOOTH
#include <QBluetoothDeviceInfo>
class QBluetoothDeviceDiscoveryAgent;
class QLowEnergyController;
class QLowEnergyService;
#endif

class BluetoothCube:public QObject{
    Q_OBJECT
public:
    explicit BluetoothCube(QObject *parent=nullptr);
    bool isConnected() const;
public slots:
    void connectToCube();
    void connectToCube(const QString &macAddress);
    void disconnectFromCube();
signals:
    void statusChanged(const QString &status);
    void connectedChanged(bool connected);
    void cubeStateChanged(const CubeFaces &faces,const QString &lastMove);
private:
#ifdef CUBEVISION_HAS_BLUETOOTH
    void deviceDiscovered(const QBluetoothDeviceInfo &info);
    void beginConnection(const QBluetoothDeviceInfo &info);
    void serviceScanFinished();
    void serviceStateChanged(int state);
    void acceptPacket(const QByteArray &packet);

    QBluetoothDeviceDiscoveryAgent *discoveryAgent=nullptr;
    QLowEnergyController *controller=nullptr;
    QLowEnergyService *dataService=nullptr;
    bool matchingDeviceFound=false;
#endif
};

