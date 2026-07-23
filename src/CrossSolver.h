#pragma once

#include "CubeState.h"
#include <QStringList>
#include <optional>

// Returns a shortest half-turn-metric sequence that solves the white D cross.
// The four white edges are only considered solved when their side stickers
// also align with the L/R/F/B centers.
std::optional<QStringList> solveWhiteCross(const CubeFaces &faces);

bool whiteCrossIsSolved(const CubeFaces &faces);
