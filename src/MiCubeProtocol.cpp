#include "MiCubeProtocol.h"

#include <algorithm>
#include <array>
#include <cstdint>

namespace {
// Giiker/Mi packet layout and constants are documented by the MIT-licensed
// smartcube-web-bluetooth Giiker driver:
// https://github.com/poliva/smartcube-web-bluetooth
constexpr std::array<std::array<int,3>,8> GiikerCornerFacelets={{
    {{26,15,29}},{{20,8,9}},{{18,38,6}},{{24,27,44}},
    {{51,35,17}},{{45,11,2}},{{47,0,36}},{{53,42,33}}
}};
constexpr std::array<std::array<int,2>,12> GiikerEdgeFacelets={{
    {{25,28}},{{23,12}},{{19,7}},{{21,41}},{{32,16}},{{5,10}},
    {{3,37}},{{30,43}},{{52,34}},{{48,14}},{{46,1}},{{50,39}}
}};
constexpr std::array<int,8> CornerOrientationSign=
    {{-1,1,-1,1,1,-1,1,-1}};

constexpr CubeFaces SolvedMarkers={{
    {{42,48, 6,36, 0,12,30,24,18}},
    {{43,49, 7,37, 1,13,31,25,19}},
    {{44,50, 8,38, 2,14,32,26,20}},
    {{45,51, 9,39, 3,15,33,27,21}},
    {{46,52,10,40, 4,16,34,28,22}},
    {{47,53,11,41, 5,17,35,29,23}}
}};

struct Facelet{ int face; int position; };
constexpr Facelet CornerFacelets[8][3]={
    {{0,8},{3,0},{4,2}},{{0,6},{4,0},{2,2}},
    {{0,0},{2,0},{5,2}},{{0,2},{5,0},{3,2}},
    {{1,2},{4,8},{3,6}},{{1,0},{2,8},{4,6}},
    {{1,6},{5,8},{2,6}},{{1,8},{3,8},{5,6}}
};
constexpr Facelet EdgeFacelets[12][2]={
    {{0,5},{3,1}},{{0,7},{4,1}},{{0,3},{2,1}},{{0,1},{5,1}},
    {{1,5},{3,7}},{{1,1},{4,7}},{{1,3},{2,7}},{{1,7},{5,7}},
    {{4,5},{3,3}},{{4,3},{2,5}},{{5,5},{2,3}},{{5,3},{3,5}}
};

std::array<int,40> packetNibbles(const QByteArray &packet){
    std::array<std::uint8_t,20> bytes{};
    for(std::size_t index=0;index<bytes.size();++index)
        bytes[index]=static_cast<std::uint8_t>(packet.at(index));

    if(bytes[18]==0xa7){
        static constexpr std::array<int,36> key={{
            176,81,104,224,86,137,237,119,38,26,193,161,
            210,126,150,81,93,13,236,249,89,235,88,24,
            113,81,214,131,130,199,2,169,39,165,171,41
        }};
        const int first=bytes[19]>>4;
        const int second=bytes[19]&0x0f;
        for(int index=0;index<18;++index)
            bytes[index]=static_cast<std::uint8_t>(
                bytes[index]+key[index+first]+key[index+second]);
    }

    std::array<int,40> nibbles{};
    for(std::size_t index=0;index<bytes.size();++index){
        nibbles[index*2]=bytes[index]>>4;
        nibbles[index*2+1]=bytes[index]&0x0f;
    }
    return nibbles;
}

template<std::size_t Size>
bool isPermutation(const std::array<int,Size> &values){
    std::array<bool,Size> seen{};
    for(const int value:values){
        if(value<0 || value>=static_cast<int>(Size) || seen[value])
            return false;
        seen[value]=true;
    }
    return true;
}

CubeFaces markerFacesFromColors(const CubeFaces &colors){
    CubeFaces result;
    for(auto &face:result)
        face.fill(-1);
    for(int face=0;face<6;++face)
        result[face][4]=SolvedMarkers[face][4];

    std::array<bool,8> usedCorners{};
    for(const auto &position:CornerFacelets){
        std::array<int,3> observed={{
            colors[position[0].face][position[0].position],
            colors[position[1].face][position[1].position],
            colors[position[2].face][position[2].position]
        }};
        auto sortedObserved=observed;
        std::sort(sortedObserved.begin(),sortedObserved.end());

        int cubie=-1;
        for(int candidate=0;candidate<8;++candidate){
            std::array<int,3> candidateColors;
            for(int sticker=0;sticker<3;++sticker){
                const Facelet solved=CornerFacelets[candidate][sticker];
                candidateColors[sticker]=SolvedMarkers[solved.face][solved.position]%6;
            }
            std::sort(candidateColors.begin(),candidateColors.end());
            if(candidateColors==sortedObserved){
                cubie=candidate;
                break;
            }
        }
        if(cubie<0 || usedCorners[cubie])
            return {};
        usedCorners[cubie]=true;
        for(int sticker=0;sticker<3;++sticker){
            const Facelet destination=position[sticker];
            for(int sourceSticker=0;sourceSticker<3;++sourceSticker){
                const Facelet source=CornerFacelets[cubie][sourceSticker];
                const int marker=SolvedMarkers[source.face][source.position];
                if(marker%6==observed[sticker]){
                    result[destination.face][destination.position]=marker;
                    break;
                }
            }
        }
    }

    std::array<bool,12> usedEdges{};
    for(const auto &position:EdgeFacelets){
        std::array<int,2> observed={{
            colors[position[0].face][position[0].position],
            colors[position[1].face][position[1].position]
        }};
        auto sortedObserved=observed;
        std::sort(sortedObserved.begin(),sortedObserved.end());

        int cubie=-1;
        for(int candidate=0;candidate<12;++candidate){
            std::array<int,2> candidateColors;
            for(int sticker=0;sticker<2;++sticker){
                const Facelet solved=EdgeFacelets[candidate][sticker];
                candidateColors[sticker]=SolvedMarkers[solved.face][solved.position]%6;
            }
            std::sort(candidateColors.begin(),candidateColors.end());
            if(candidateColors==sortedObserved){
                cubie=candidate;
                break;
            }
        }
        if(cubie<0 || usedEdges[cubie])
            return {};
        usedEdges[cubie]=true;
        for(int sticker=0;sticker<2;++sticker){
            const Facelet destination=position[sticker];
            for(int sourceSticker=0;sourceSticker<2;++sourceSticker){
                const Facelet source=EdgeFacelets[cubie][sourceSticker];
                const int marker=SolvedMarkers[source.face][source.position];
                if(marker%6==observed[sticker]){
                    result[destination.face][destination.position]=marker;
                    break;
                }
            }
        }
    }
    return result;
}
}

std::optional<MiCubeState> decodeMiCubePacket(const QByteArray &packet){
    if(packet.size()!=20)
        return std::nullopt;
    const auto value=packetNibbles(packet);

    std::array<int,8> corners;
    std::array<int,8> cornerOrientations;
    int cornerOrientationSum=0;
    for(int index=0;index<8;++index){
        corners[index]=value[index]-1;
        cornerOrientations[index]=
            (3+value[index+8]*CornerOrientationSign[index])%3;
        cornerOrientationSum+=cornerOrientations[index];
    }
    std::array<int,12> edges;
    std::array<int,12> edgeOrientations{};
    int edgeOrientationSum=0;
    for(int index=0;index<12;++index){
        edges[index]=value[index+16]-1;
        const int mask=8>>(index%4);
        edgeOrientations[index]=(value[28+index/4]&mask) ? 1 : 0;
        edgeOrientationSum+=edgeOrientations[index];
    }
    if(!isPermutation(corners) || !isPermutation(edges)
        || cornerOrientationSum%3!=0 || edgeOrientationSum%2!=0)
        return std::nullopt;

    // Giiker's decoder produces a facelet string in URFDLB order.
    std::array<int,54> standardColors;
    for(int index=0;index<54;++index)
        standardColors[index]=index/9;
    for(int position=0;position<8;++position){
        const int cubie=corners[position];
        const int orientation=cornerOrientations[position];
        for(int sticker=0;sticker<3;++sticker)
            standardColors[GiikerCornerFacelets[position][(sticker+orientation)%3]]
                =GiikerCornerFacelets[cubie][sticker]/9;
    }
    for(int position=0;position<12;++position){
        const int cubie=edges[position];
        const int orientation=edgeOrientations[position];
        for(int sticker=0;sticker<2;++sticker)
            standardColors[GiikerEdgeFacelets[position][(sticker+orientation)%2]]
                =GiikerEdgeFacelets[cubie][sticker]/9;
    }

    // Convert the Mi Smart Magic Cube's facelet order to CubeVision's UDLRFB
    // face order. The Mi cube reports faces with U/D, R/F and L/B swapped
    // relative to the Giiker-style URFDLB order assumed by the decoder tables.
    static constexpr std::array<int,6> standardToCubeVision={{1,4,3,0,5,2}};
    CubeFaces colors;
    for(int standardFace=0;standardFace<6;++standardFace){
        const int cubeVisionFace=standardToCubeVision[standardFace];
        for(int position=0;position<9;++position)
            colors[cubeVisionFace][position]=
                standardToCubeVision[standardColors[standardFace*9+position]];
    }
    CubeFaces faces=markerFacesFromColors(colors);
    for(const auto &face:faces)
        if(std::find(face.begin(),face.end(),-1)!=face.end())
            return std::nullopt;

    QString move;
    const int moveFace=value[32]-1;
    const int moveAmount=value[33];
    if(moveFace>=0 && moveFace<6 && moveAmount>0){
        static constexpr std::array<char,6> facesByCode={{'B','D','L','U','R','F'}};
        move=QChar(facesByCode[moveFace]);
        if(moveAmount==3 || moveAmount==9)
            move+='2';
        else if(moveAmount==4)
            move+='\'';
    }
    return MiCubeState{faces,move};
}
