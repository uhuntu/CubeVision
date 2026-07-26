#include "BluetoothCube.h"
#include "MiCubeProtocol.h"

#ifdef CUBEVISION_HAS_BLUETOOTH
#include <QBluetoothAddress>
#include <QBluetoothDeviceDiscoveryAgent>
#include <QBluetoothLocalDevice>
#include <QBluetoothUuid>
#include <QDebug>
#include <QLowEnergyCharacteristic>
#include <QLowEnergyController>
#include <QLowEnergyDescriptor>
#include <QLowEnergyService>

namespace {
const QBluetoothUuid DataServiceUuid(
    QStringLiteral("0000aadb-0000-1000-8000-00805f9b34fb"));
const QBluetoothUuid DataCharacteristicUuid(
    QStringLiteral("0000aadc-0000-1000-8000-00805f9b34fb"));
const quint16 XiaomiManufacturerId=0x038f;
const QBluetoothUuid XiaomiServiceUuid(
    QStringLiteral("0000fe95-0000-1000-8000-00805f9b34fb"));
}
#endif

BluetoothCube::BluetoothCube(QObject *parent):QObject(parent){
#ifdef CUBEVISION_HAS_BLUETOOTH
    discoveryAgent=new QBluetoothDeviceDiscoveryAgent(this);
    discoveryAgent->setLowEnergyDiscoveryTimeout(12000);
    connect(discoveryAgent,&QBluetoothDeviceDiscoveryAgent::deviceDiscovered,
            this,&BluetoothCube::deviceDiscovered);
    connect(discoveryAgent,&QBluetoothDeviceDiscoveryAgent::finished,this,[this]{
        qDebug()<<"[BluetoothCube] scan finished; matchingDeviceFound="<<matchingDeviceFound;
        if(!matchingDeviceFound)
            emit statusChanged("Bluetooth cube not found; twist a face to wake it and retry");
    });
    connect(discoveryAgent,&QBluetoothDeviceDiscoveryAgent::errorOccurred,
            this,[this](QBluetoothDeviceDiscoveryAgent::Error error){
        qDebug()<<"[BluetoothCube] scan error:"<<error<<discoveryAgent->errorString();
        emit statusChanged("Bluetooth scan failed: "+discoveryAgent->errorString());
    });
#endif
}

bool BluetoothCube::isConnected() const{
#ifdef CUBEVISION_HAS_BLUETOOTH
    return controller
        &&controller->state()==QLowEnergyController::ConnectedState
        &&dataService;
#else
    return false;
#endif
}

void BluetoothCube::connectToCube(){
#ifdef CUBEVISION_HAS_BLUETOOTH
    if(isConnected()){
        disconnectFromCube();
        return;
    }
    QBluetoothLocalDevice localDevice;
    qDebug()<<"[BluetoothCube] localDevice valid="<<localDevice.isValid()
            <<"hostMode="<<localDevice.hostMode();
    if(!localDevice.isValid()){
        emit statusChanged("Bluetooth adapter not detected; enable Bluetooth or use Connect by MAC");
        return;
    }
    if(localDevice.hostMode()==QBluetoothLocalDevice::HostPoweredOff){
        emit statusChanged("Bluetooth is off; turn it on and retry");
        return;
    }
    matchingDeviceFound=false;
    emit statusChanged("Scanning for Mi Smart Magic Cube...");
    discoveryAgent->start();
#else
    emit statusChanged(
        "Bluetooth support is unavailable in this build (install Qt 6 Connectivity and rebuild)");
#endif
}

void BluetoothCube::connectToCube(const QString &macAddress){
#ifdef CUBEVISION_HAS_BLUETOOTH
    if(isConnected()){
        disconnectFromCube();
        return;
    }
    QBluetoothAddress address(macAddress.trimmed());
    if(address.isNull()){
        emit statusChanged("Invalid MAC address: "+macAddress);
        return;
    }
    QBluetoothDeviceInfo info(address,"Mi Smart Magic Cube",0);
    info.setServiceUuids({XiaomiServiceUuid});
    qDebug()<<"[BluetoothCube] direct connect to"<<address.toString();
    matchingDeviceFound=true;
    beginConnection(info);
#else
    emit statusChanged(
        "Bluetooth support is unavailable in this build (install Qt 6 Connectivity and rebuild)");
#endif
}

void BluetoothCube::disconnectFromCube(){
#ifdef CUBEVISION_HAS_BLUETOOTH
    if(discoveryAgent->isActive())
        discoveryAgent->stop();
    if(controller)
        controller->disconnectFromDevice();
    if(dataService){
        dataService->deleteLater();
        dataService=nullptr;
    }
    emit connectedChanged(false);
    emit statusChanged("Bluetooth cube disconnected");
#endif
}

#ifdef CUBEVISION_HAS_BLUETOOTH
void BluetoothCube::deviceDiscovered(const QBluetoothDeviceInfo &info){
    const QString name=info.name();
    const auto manufacturerData=info.manufacturerData(XiaomiManufacturerId);
    const auto serviceUuids=info.serviceUuids();
    const bool matchesName=name.startsWith("Mi Smart Magic Cube",Qt::CaseInsensitive)
        ||name.startsWith("Gi",Qt::CaseInsensitive);
    const bool matchesXiaomi=!manufacturerData.isEmpty();
    const bool matchesService=serviceUuids.contains(XiaomiServiceUuid);
    qDebug()<<"[BluetoothCube] discovered"<<info.address().toString()<<"name="<<name
            <<"manufacturer[Xiaomi]="<<manufacturerData.toHex()
            <<"services="<<serviceUuids<<"matches="<<(matchesName||matchesXiaomi||matchesService);
    if(!matchesName&&!matchesXiaomi&&!matchesService)
        return;
    matchingDeviceFound=true;
    discoveryAgent->stop();
    beginConnection(info);
}

void BluetoothCube::beginConnection(const QBluetoothDeviceInfo &info){
    if(controller){
        controller->deleteLater();
        controller=nullptr;
    }
    const QString displayName=info.name().isEmpty()?QStringLiteral("Mi Smart Magic Cube"):info.name();
    emit statusChanged("Connecting to "+displayName+"...");
    controller=QLowEnergyController::createCentral(info,this);
    connect(controller,&QLowEnergyController::connected,this,[this]{
        emit statusChanged("Discovering Bluetooth cube service...");
        controller->discoverServices();
    });
    connect(controller,&QLowEnergyController::discoveryFinished,
            this,&BluetoothCube::serviceScanFinished);
    connect(controller,&QLowEnergyController::disconnected,this,[this]{
        if(dataService){
            dataService->deleteLater();
            dataService=nullptr;
        }
        emit connectedChanged(false);
        emit statusChanged("Bluetooth cube disconnected");
    });
    connect(controller,&QLowEnergyController::errorOccurred,
            this,[this](QLowEnergyController::Error){
        emit statusChanged("Bluetooth connection failed: "+controller->errorString());
    });
    controller->connectToDevice();
}

void BluetoothCube::serviceScanFinished(){
    dataService=controller->createServiceObject(DataServiceUuid,this);
    if(!dataService){
        emit statusChanged("Connected device does not expose the Mi cube service");
        controller->disconnectFromDevice();
        return;
    }
    connect(dataService,&QLowEnergyService::stateChanged,this,
            [this](QLowEnergyService::ServiceState state){
        serviceStateChanged(static_cast<int>(state));
    });
    connect(dataService,&QLowEnergyService::characteristicChanged,this,
            [this](const QLowEnergyCharacteristic &characteristic,
                   const QByteArray &value){
        if(characteristic.uuid()==DataCharacteristicUuid)
            acceptPacket(value);
    });
    connect(dataService,&QLowEnergyService::characteristicRead,this,
            [this](const QLowEnergyCharacteristic &characteristic,
                   const QByteArray &value){
        if(characteristic.uuid()==DataCharacteristicUuid)
            acceptPacket(value);
    });
    connect(dataService,&QLowEnergyService::errorOccurred,this,
            [this](QLowEnergyService::ServiceError){
        emit statusChanged("Bluetooth cube service error");
    });
    dataService->discoverDetails();
}

void BluetoothCube::serviceStateChanged(const int state){
    if(state!=static_cast<int>(QLowEnergyService::RemoteServiceDiscovered))
        return;
    const auto characteristic=dataService->characteristic(DataCharacteristicUuid);
    if(!characteristic.isValid()){
        emit statusChanged("Mi cube state characteristic was not found");
        return;
    }
    const auto notification=characteristic.clientCharacteristicConfiguration();
    if(notification.isValid())
        dataService->writeDescriptor(
            notification,QLowEnergyCharacteristic::CCCDEnableNotification);
    dataService->readCharacteristic(characteristic);
    emit connectedChanged(true);
    emit statusChanged("Mi Bluetooth cube connected; twist any face to sync");
}

void BluetoothCube::acceptPacket(const QByteArray &packet){
    const auto state=decodeMiCubePacket(packet);
    if(!state){
        emit statusChanged("Received an invalid Mi cube state packet");
        return;
    }
    emit cubeStateChanged(state->faces,state->lastMove);
}
#endif
