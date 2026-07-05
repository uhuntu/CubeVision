#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {
std::string base64Encode(const std::vector<unsigned char> &data){
    static constexpr char alphabet[]=
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string output;
    output.reserve((data.size()+2)/3*4);

    for(std::size_t index=0;index<data.size();index+=3){
        const unsigned int first=data[index];
        const unsigned int second=index+1<data.size() ? data[index+1] : 0;
        const unsigned int third=index+2<data.size() ? data[index+2] : 0;
        const unsigned int value=(first<<16)|(second<<8)|third;
        output.push_back(alphabet[(value>>18)&0x3f]);
        output.push_back(alphabet[(value>>12)&0x3f]);
        output.push_back(index+1<data.size() ? alphabet[(value>>6)&0x3f] : '=');
        output.push_back(index+2<data.size() ? alphabet[value&0x3f] : '=');
    }
    return output;
}
}

int main(int argc,char **argv){
    if(argc!=2){
        std::cerr<<"Usage: GenerateCharuco <output.svg>\n";
        return 1;
    }

    constexpr int squaresX=8;
    constexpr int squaresY=11;
    constexpr float squareLength=20.0F;
    constexpr float markerLength=15.0F;
    constexpr int pixelsPerMillimeter=20;

    const cv::aruco::CharucoBoard board(
        cv::Size(squaresX,squaresY),
        squareLength,
        markerLength,
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100));

    cv::Mat image;
    board.generateImage(cv::Size(squaresX*20*pixelsPerMillimeter,
                                 squaresY*20*pixelsPerMillimeter),
                        image,0,1);

    const cv::aruco::CharucoDetector detector(board);
    std::vector<cv::Point2f> detectedCorners;
    std::vector<int> detectedIds;
    detector.detectBoard(image,detectedCorners,detectedIds);
    const std::size_t expectedCorners=(squaresX-1)*(squaresY-1);
    if(detectedIds.size()!=expectedCorners){
        std::cerr<<"Generated board self-check failed: detected "
                 <<detectedIds.size()<<" of "<<expectedCorners<<" corners\n";
        return 1;
    }

    std::vector<unsigned char> png;
    cv::imencode(".png",image,png,{cv::IMWRITE_PNG_COMPRESSION,9});

    std::ofstream output(argv[1],std::ios::binary);
    if(!output){
        std::cerr<<"Cannot write "<<argv[1]<<'\n';
        return 1;
    }

    output<<"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
          <<"<svg xmlns=\"http://www.w3.org/2000/svg\" "
          <<"xmlns:xlink=\"http://www.w3.org/1999/xlink\" "
          <<"width=\"180mm\" height=\"240mm\" viewBox=\"0 0 180 240\">\n"
          <<"  <rect width=\"180\" height=\"240\" fill=\"white\"/>\n"
          <<"  <image x=\"10\" y=\"10\" width=\"160\" height=\"220\" "
          <<"image-rendering=\"pixelated\" xlink:href=\"data:image/png;base64,"
          <<base64Encode(png)<<"\"/>\n"
          <<"</svg>\n";

    std::cout<<"Generated 8x11 ChArUco board: 20 mm squares, 15 mm markers, "
             <<"DICT_5X5_100; verified "<<detectedIds.size()<<" corners\n";
    return 0;
}
