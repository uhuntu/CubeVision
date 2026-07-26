#include "MiCubeProtocol.h"

#include <array>
#include <cassert>

namespace {
constexpr CubeFaces SolvedMarkers={{
    {{42,48, 6,36, 0,12,30,24,18}},
    {{43,49, 7,37, 1,13,31,25,19}},
    {{44,50, 8,38, 2,14,32,26,20}},
    {{45,51, 9,39, 3,15,33,27,21}},
    {{46,52,10,40, 4,16,34,28,22}},
    {{47,53,11,41, 5,17,35,29,23}}
}};

constexpr std::array<std::array<int,3>,8> CornerFacelets={{
    {{26,15,29}},{{20,8,9}},{{18,38,6}},{{24,27,44}},
    {{51,35,17}},{{45,11,2}},{{47,0,36}},{{53,42,33}}
}};
constexpr std::array<std::array<int,2>,12> EdgeFacelets={{
    {{25,28}},{{23,12}},{{19,7}},{{21,41}},{{32,16}},{{5,10}},
    {{3,37}},{{30,43}},{{52,34}},{{48,14}},{{46,1}},{{50,39}}
}};
constexpr std::array<int,8> OrientationSign={{-1,1,-1,1,1,-1,1,-1}};

void setNibble(QByteArray &packet,const int index,const int value){
    const int byteIndex=index/2;
    const unsigned char old=static_cast<unsigned char>(packet[byteIndex]);
    packet[byteIndex]=static_cast<char>(
        index%2==0 ? (old&0x0f)|(value<<4) : (old&0xf0)|value);
}

QByteArray encodeState(const CubeFaces &faces){
    static constexpr std::array<int,6> cubeVisionToStandard={{0,3,4,1,2,5}};
    std::array<int,54> colors;
    for(int face=0;face<6;++face)
        for(int position=0;position<9;++position)
            colors[cubeVisionToStandard[face]*9+position]=
                cubeVisionToStandard[faces[face][position]%6];

    QByteArray packet(20,'\0');
    for(int position=0;position<8;++position){
        int cubie=0;
        int orientation=0;
        for(;cubie<8;++cubie){
            for(orientation=0;orientation<3;++orientation){
                bool matches=true;
                for(int sticker=0;sticker<3;++sticker){
                    const int destination=
                        CornerFacelets[position][(sticker+orientation)%3];
                    if(colors[destination]!=CornerFacelets[cubie][sticker]/9){
                        matches=false;
                        break;
                    }
                }
                if(matches)
                    break;
            }
            if(orientation<3)
                break;
        }
        assert(cubie<8);
        setNibble(packet,position,cubie+1);
        setNibble(packet,position+8,
                  OrientationSign[position]>0
                      ? orientation : (3-orientation)%3);
    }

    std::array<int,12> edgeOrientations{};
    for(int position=0;position<12;++position){
        int cubie=0;
        int orientation=0;
        for(;cubie<12;++cubie){
            if(colors[EdgeFacelets[position][0]]==EdgeFacelets[cubie][0]/9
               &&colors[EdgeFacelets[position][1]]==EdgeFacelets[cubie][1]/9)
                break;
            if(colors[EdgeFacelets[position][0]]==EdgeFacelets[cubie][1]/9
               &&colors[EdgeFacelets[position][1]]==EdgeFacelets[cubie][0]/9){
                orientation=1;
                break;
            }
        }
        assert(cubie<12);
        setNibble(packet,position+16,cubie+1);
        edgeOrientations[position]=orientation;
    }
    for(int group=0;group<3;++group){
        int bits=0;
        for(int offset=0;offset<4;++offset)
            if(edgeOrientations[group*4+offset])
                bits|=8>>offset;
        setNibble(packet,28+group,bits);
    }
    setNibble(packet,32,1);
    setNibble(packet,33,1);
    return packet;
}
}

int main(){
    assert(!decodeMiCubePacket(QByteArray(19,'\0')));
    assert(!decodeMiCubePacket(QByteArray(20,'\0')));

    const QByteArray solved=QByteArray::fromHex(
        "1234567800000000123456789abc000000000000");
    const auto decoded=decodeMiCubePacket(solved);
    assert(decoded);
    assert(decoded->faces==SolvedMarkers);

    QByteArray movePacket=solved;
    setNibble(movePacket,32,4); // U
    setNibble(movePacket,33,4); // counterclockwise
    const auto moveDecoded=decodeMiCubePacket(movePacket);
    assert(moveDecoded && moveDecoded->lastMove=="U'");

    static constexpr std::array<QChar,6> moves={{'U','D','L','R','F','B'}};
    for(const QChar move:moves){
        for(int turns=1;turns<=3;++turns){
            const CubeFaces expected=applyCubeMove(SolvedMarkers,move,turns);
            const auto moved=decodeMiCubePacket(encodeState(expected));
            assert(moved);
            assert(moved->faces==expected);
        }
    }
}
