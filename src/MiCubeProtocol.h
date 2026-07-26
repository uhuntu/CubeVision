#pragma once

#include "CubeState.h"
#include <QByteArray>
#include <QString>
#include <optional>

struct MiCubeState{
    CubeFaces faces;
    QString lastMove;
};

// Decodes the 20-byte state used by Giiker and Xiaomi's
// "Mi Smart Magic Cube" BLE characteristic.
std::optional<MiCubeState> decodeMiCubePacket(const QByteArray &packet);

