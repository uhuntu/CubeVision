#include "BluetoothCube.h"
#include "CubeState.h"
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
    info.setCoreConfigurations(QBluetoothDeviceInfo::LowEnergyCoreConfiguration);
    qDebug()<<"[BluetoothCube] direct connect to"<<address.toString()
            <<"coreConfigs="<<info.coreConfigurations();
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
    previousFaces.reset();
    physicalFaces.reset();
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
            this,[this](QLowEnergyController::Error error){
        qDebug()<<"[BluetoothCube] connection error:"<<error<<controller->errorString();
        emit statusChanged("Bluetooth connection failed ["+QString::number(static_cast<int>(error))+
                           "]: "+controller->errorString());
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
            [this](QLowEnergyService::ServiceError error){
        qDebug()<<"[BluetoothCube] service error:"<<error;
        emit statusChanged("Bluetooth cube service error ["+
                           QString::number(static_cast<int>(error))+"]");
    });
    dataService->discoverDetails();
}

void BluetoothCube::serviceStateChanged(const int state){
    qDebug()<<"[BluetoothCube] service state changed:"<<state;
    if(state!=static_cast<int>(QLowEnergyService::RemoteServiceDiscovered))
        return;
    const auto characteristic=dataService->characteristic(DataCharacteristicUuid);
    qDebug()<<"[BluetoothCube] characteristic valid="<<characteristic.isValid()
            <<"props="<<characteristic.properties();
    if(!characteristic.isValid()){
        emit statusChanged("Mi cube state characteristic was not found");
        return;
    }
    const auto notification=characteristic.clientCharacteristicConfiguration();
    qDebug()<<"[BluetoothCube] CCCD valid="<<notification.isValid();
    if(notification.isValid()){
        qDebug()<<"[BluetoothCube] enabling notifications";
        dataService->writeDescriptor(
            notification,QLowEnergyCharacteristic::CCCDEnableNotification);
    }else{
        qDebug()<<"[BluetoothCube] CCCD not valid, trying direct read";
        dataService->readCharacteristic(characteristic);
    }
    emit connectedChanged(true);
    emit statusChanged("Mi Bluetooth cube connected; twist any face to sync");
}

static QString deriveMoveFromStateChange(const CubeFaces &previous,const CubeFaces &current){
    static constexpr std::array<char,6> faceChars={{'U','D','L','R','F','B'}};
    for(char faceChar:faceChars){
        for(int turns=1;turns<=3;++turns){
            if(applyCubeMove(previous,faceChar,turns)==current){
                QString move=QChar(faceChar);
                if(turns==2)
                    move+='2';
                else if(turns==3)
                    move+='\'';
                return move;
            }
        }
    }
    return QString();
}

static QString changedFaceIndices(const CubeFaces &previous,const CubeFaces &current){
    QStringList changed;
    static constexpr std::array<char,6> faceChars={{'U','D','L','R','F','B'}};
    for(int face=0;face<6;++face){
        if(previous[face]!=current[face])
            changed.append(QChar(faceChars[face]));
    }
    return changed.join(',');
}

// The Mi Smart Magic Cube's firmware uses a face coordinate system that is
// rotated 120 degrees around a body diagonal relative to the physical cube.
// Observed mapping (physical twist -> reported/displayed move before remap):
//   U<->D, L<->B, R<->F
static QString remapMiCubeMove(const QString &move){
    if(move.isEmpty()) return move;
    static constexpr std::array<int,6> remap={{'D','U','B','F','R','L'}};
    static constexpr std::array<char,6> faces={{'U','D','L','R','F','B'}};
    const char faceChar = move[0].toUpper().toLatin1();
    int index=-1;
    for(int i=0;i<6;++i){
        if(faces[i]==faceChar){
            index=i;
            break;
        }
    }
    if(index<0) return move;
    QString result=move;
    result[0]=QChar(remap[index]);
    return result;
}

static CubeFaces solvedCubeFaces(){
    return {{{{0,0,0,0,0,0,0,0,0}},
             {{1,1,1,1,1,1,1,1,1}},
             {{2,2,2,2,2,2,2,2,2}},
             {{3,3,3,3,3,3,3,3,3}},
             {{4,4,4,4,4,4,4,4,4}},
             {{5,5,5,5,5,5,5,5,5}}}};
}

static CubeFaces applyMoveString(const CubeFaces &faces,const QString &move){
    if(move.isEmpty()) return faces;
    const QChar face=move.front().toUpper();
    int turns=1;
    if(move.size()>1){
        if(move[1]=='2') turns=2;
        else if(move[1]=='\'') turns=3;
    }
    return applyCubeMove(faces,face,turns);
}

void BluetoothCube::acceptPacket(const QByteArray &packet){
    qDebug()<<"[BluetoothCube] packet received, len="<<packet.size()
            <<"hex="<<packet.toHex();
    const auto state=decodeMiCubePacket(packet);
    if(!state){
        qDebug()<<"[BluetoothCube] packet decode failed";
        emit statusChanged("Received an invalid Mi cube state packet");
        return;
    }
    QString move=state->lastMove;
    QString changedFaces;
    if(previousFaces){
        changedFaces=changedFaceIndices(*previousFaces,state->faces);
        const QString derived=deriveMoveFromStateChange(*previousFaces,state->faces);
        qDebug()<<"[BluetoothCube] changed faces:"<<changedFaces
                <<"packet move="<<move
                <<"derived move="<<(derived.isEmpty()?"<none>":derived);
        if(!derived.isEmpty())
            move=derived;
    }else{
        qDebug()<<"[BluetoothCube] first packet, packet move="<<move;
    }
    previousFaces=state->faces;
    const QString remappedMove=remapMiCubeMove(move);
    qDebug()<<"[BluetoothCube] final lastMove="<<move<<"remapped="<<remappedMove;
    QString emittedMove=remappedMove;
    if(!physicalFaces){
        // The first packet after connect carries the current cube state, but its
        // lastMove field may be stale (the move before sleep/wake). Start from a
        // solved physical state and do not apply that first move.
        physicalFaces=solvedCubeFaces();
        emittedMove.clear();
    }else{
        *physicalFaces=applyMoveString(*physicalFaces,remappedMove);
    }
    emit cubeStateChanged(*physicalFaces,emittedMove);
}
#endif
