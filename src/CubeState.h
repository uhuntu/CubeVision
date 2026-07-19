#pragma once

#include <QChar>
#include <array>

using CubeFaces=std::array<std::array<int,9>,6>;

// Applies a standard face turn to all 54 uniquely identified facelets.
// turns is 1 for clockwise, 2 for 180 degrees, and 3 for counterclockwise.
CubeFaces applyCubeMove(const CubeFaces &faces,QChar face,int turns);
