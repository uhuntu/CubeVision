#include "CubeState.h"
#include <array>
#include <cassert>

namespace {
CubeFaces labelledCube(){
    CubeFaces faces;
    int label=0;
    for(auto &face:faces)
        for(int &facelet:face)
            facelet=label++;
    return faces;
}
}

int main(){
    static constexpr std::array<QChar,6> moves={'U','D','L','R','F','B'};
    const CubeFaces solved=labelledCube();
    for(const QChar move:moves){
        CubeFaces state=solved;
        for(int turn=0;turn<4;++turn)
            state=applyCubeMove(state,move,1);
        assert(state==solved);
        assert(applyCubeMove(applyCubeMove(solved,move,1),move,3)==solved);
        assert(applyCubeMove(applyCubeMove(solved,move,2),move,2)==solved);
    }
}
