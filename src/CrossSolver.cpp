#include "CrossSolver.h"

#include <QChar>
#include <array>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <vector>

namespace {
constexpr std::array<int,8> CrossMarkerIds={
    49,28, // D/F
    13,27, // D/R
    37,26, // D/L
    25,29  // D/B
};

constexpr std::array<std::uint8_t,8> SolvedCrossPositions={
    1*9+1,4*9+7,
    1*9+5,3*9+7,
    1*9+3,2*9+7,
    1*9+7,5*9+7
};

constexpr std::array<QChar,6> MoveFaces={'U','D','L','R','F','B'};

struct CrossKey{
    std::array<std::uint8_t,8> positions{};

    bool operator==(const CrossKey &other) const{
        return positions==other.positions;
    }
};

struct CrossKeyHash{
    std::size_t operator()(const CrossKey &key) const{
        std::size_t hash=1469598103934665603ULL;
        for(const std::uint8_t position:key.positions){
            hash^=position;
            hash*=1099511628211ULL;
        }
        return hash;
    }
};

struct Move{
    QChar face;
    int turns;
    std::array<std::uint8_t,54> destination;
};

struct SearchNode{
    CrossKey key;
    int parent=-1;
    std::uint8_t move=0;
    std::int8_t lastFace=-1;
};

const std::array<Move,18>& moves(){
    static const std::array<Move,18> all=[]{
        std::array<Move,18> result;
        CubeFaces labelled;
        for(int face=0;face<6;++face)
            for(int position=0;position<9;++position)
                labelled[face][position]=face*9+position;

        std::size_t moveIndex=0;
        for(const QChar face:MoveFaces){
            for(int turns=1;turns<=3;++turns){
                Move move;
                move.face=face;
                move.turns=turns;
                const CubeFaces moved=applyCubeMove(labelled,face,turns);
                for(int destinationFace=0;destinationFace<6;++destinationFace){
                    for(int destinationPosition=0;destinationPosition<9;
                        ++destinationPosition){
                        const int source=moved[destinationFace][destinationPosition];
                        move.destination[source]=static_cast<std::uint8_t>(
                            destinationFace*9+destinationPosition);
                    }
                }
                result[moveIndex++]=move;
            }
        }
        return result;
    }();
    return all;
}

std::optional<CrossKey> crossKey(const CubeFaces &faces){
    CrossKey key;
    std::array<bool,8> found{};
    for(int face=0;face<6;++face){
        for(int position=0;position<9;++position){
            const int id=faces[face][position];
            for(std::size_t marker=0;marker<CrossMarkerIds.size();++marker){
                if(id!=CrossMarkerIds[marker])
                    continue;
                if(found[marker])
                    return std::nullopt;
                found[marker]=true;
                key.positions[marker]=static_cast<std::uint8_t>(face*9+position);
            }
        }
    }
    for(const bool markerFound:found)
        if(!markerFound)
            return std::nullopt;
    return key;
}

QString moveNotation(const Move &move){
    QString notation(move.face);
    if(move.turns==2)
        notation+='2';
    else if(move.turns==3)
        notation+='\'';
    return notation;
}
}

bool whiteCrossIsSolved(const CubeFaces &faces){
    const auto key=crossKey(faces);
    return key && key->positions==SolvedCrossPositions;
}

std::optional<QStringList> solveWhiteCross(const CubeFaces &faces){
    const auto initial=crossKey(faces);
    if(!initial)
        return std::nullopt;
    if(initial->positions==SolvedCrossPositions)
        return QStringList();

    std::vector<SearchNode> nodes;
    nodes.reserve(200000);
    nodes.push_back({*initial,-1,0,-1});

    std::queue<int> frontier;
    frontier.push(0);
    std::unordered_map<CrossKey,int,CrossKeyHash> visited;
    visited.reserve(200000);
    visited.emplace(*initial,0);

    int goal=-1;
    const auto &availableMoves=moves();
    while(!frontier.empty() && goal<0){
        const int parentIndex=frontier.front();
        frontier.pop();
        const SearchNode parent=nodes[parentIndex];
        for(std::size_t moveIndex=0;moveIndex<availableMoves.size();++moveIndex){
            const int moveFace=static_cast<int>(moveIndex/3);
            if(moveFace==parent.lastFace)
                continue;

            CrossKey next;
            for(std::size_t marker=0;marker<next.positions.size();++marker)
                next.positions[marker]=
                    availableMoves[moveIndex].destination[parent.key.positions[marker]];
            if(visited.find(next)!=visited.end())
                continue;

            const int nodeIndex=static_cast<int>(nodes.size());
            nodes.push_back({next,parentIndex,static_cast<std::uint8_t>(moveIndex),
                             static_cast<std::int8_t>(moveFace)});
            visited.emplace(next,nodeIndex);
            if(next.positions==SolvedCrossPositions){
                goal=nodeIndex;
                break;
            }
            frontier.push(nodeIndex);
        }
    }
    if(goal<0)
        return std::nullopt;

    QStringList solution;
    while(nodes[goal].parent>=0){
        solution.prepend(moveNotation(availableMoves[nodes[goal].move]));
        goal=nodes[goal].parent;
    }
    return solution;
}
