#include "CubeState.h"

namespace {
struct CubeVector{ int x; int y; int z; };
struct FaceletGeometry{ CubeVector position; CubeVector normal; };

FaceletGeometry faceletGeometry(const int face,const int row,const int column){
    switch(face){
    case 0: return {{column-1, 1,row-1},{ 0, 1, 0}}; // U
    case 1: return {{column-1,-1,1-row},{ 0,-1, 0}}; // D
    case 2: return {{-1,1-row,column-1},{-1, 0, 0}}; // L
    case 3: return {{ 1,1-row,1-column},{ 1, 0, 0}}; // R
    case 4: return {{column-1,1-row, 1},{ 0, 0, 1}}; // F
    default:return {{1-column,1-row,-1},{ 0, 0,-1}}; // B
    }
}

std::pair<int,int> faceletIndex(const FaceletGeometry &geometry){
    const auto &position=geometry.position;
    const auto &normal=geometry.normal;
    if(normal.y==1)  return {0,(position.z+1)*3+position.x+1};
    if(normal.y==-1) return {1,(1-position.z)*3+position.x+1};
    if(normal.x==-1) return {2,(1-position.y)*3+position.z+1};
    if(normal.x==1)  return {3,(1-position.y)*3+1-position.z};
    if(normal.z==1)  return {4,(1-position.y)*3+position.x+1};
    return {5,(1-position.y)*3+1-position.x};
}

CubeVector rotateClockwise(const CubeVector value,const CubeVector normal){
    const int parallel=value.x*normal.x+value.y*normal.y+value.z*normal.z;
    const CubeVector cross={normal.y*value.z-normal.z*value.y,
                            normal.z*value.x-normal.x*value.z,
                            normal.x*value.y-normal.y*value.x};
    return {normal.x*parallel-cross.x,
            normal.y*parallel-cross.y,
            normal.z*parallel-cross.z};
}

int faceIndex(const QChar face){
    if(face=='U') return 0;
    if(face=='D') return 1;
    if(face=='L') return 2;
    if(face=='R') return 3;
    if(face=='F') return 4;
    return 5;
}
}

CubeFaces applyCubeMove(const CubeFaces &faces,const QChar face,const int turns){
    CubeFaces result=faces;
    const int movingFace=faceIndex(face);
    const CubeVector axis=faceletGeometry(movingFace,1,1).normal;
    for(int sourceFace=0;sourceFace<6;++sourceFace){
        for(int sourcePosition=0;sourcePosition<9;++sourcePosition){
            FaceletGeometry geometry=faceletGeometry(
                sourceFace,sourcePosition/3,sourcePosition%3);
            const int layer=geometry.position.x*axis.x
                +geometry.position.y*axis.y+geometry.position.z*axis.z;
            if(layer!=1)
                continue;
            for(int turn=0;turn<turns;++turn){
                geometry.position=rotateClockwise(geometry.position,axis);
                geometry.normal=rotateClockwise(geometry.normal,axis);
            }
            const auto [destinationFace,destinationPosition]=faceletIndex(geometry);
            result[destinationFace][destinationPosition]=faces[sourceFace][sourcePosition];
        }
    }
    return result;
}
