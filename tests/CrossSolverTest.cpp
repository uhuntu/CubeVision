#include "CrossSolver.h"

#include <array>
#include <cassert>

namespace {
CubeFaces solvedMarkerCube(){
    return {{
        {{42,48, 6,36, 0,12,30,24,18}},
        {{43,49, 7,37, 1,13,31,25,19}},
        {{44,50, 8,38, 2,14,32,26,20}},
        {{45,51, 9,39, 3,15,33,27,21}},
        {{46,52,10,40, 4,16,34,28,22}},
        {{47,53,11,41, 5,17,35,29,23}}
    }};
}

CubeFaces applyMoves(CubeFaces state,const QStringList &moves){
    for(const QString &move:moves){
        int turns=1;
        if(move.endsWith('2'))
            turns=2;
        else if(move.endsWith('\''))
            turns=3;
        state=applyCubeMove(state,move.front(),turns);
    }
    return state;
}
}

int main(){
    const CubeFaces solved=solvedMarkerCube();
    assert(whiteCrossIsSolved(solved));
    const auto emptySolution=solveWhiteCross(solved);
    assert(emptySolution && emptySolution->isEmpty());

    const std::array<QStringList,4> scrambles={{
        {"R","U","F2","D","L'","B","U2"},
        {"F","R2","U'","B2","L","D2","F'","U"},
        {"D","B'","R","U2","L2","F","D'","R2"},
        {"L'","F2","D","R'","B","U","F'","D2","L"}
    }};
    for(const QStringList &scramble:scrambles){
        const CubeFaces scrambled=applyMoves(solved,scramble);
        assert(!whiteCrossIsSolved(scrambled));
        const auto solution=solveWhiteCross(scrambled);
        assert(solution);
        assert(whiteCrossIsSolved(applyMoves(scrambled,*solution)));
    }

    CubeFaces invalid=applyMoves(solved,scrambles.front());
    for(auto &face:invalid)
        for(int &id:face)
            if(id==49)
                id=0;
    assert(!solveWhiteCross(invalid));
}
