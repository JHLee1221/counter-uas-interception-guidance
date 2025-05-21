#ifndef YOLOV5_TRT_INFER_COLORMAT_H
#define YOLOV5_TRT_INFER_COLORMAT_H

// using vector type data
#include <iostream>
#include <string>
#include <stdio.h>
#include <ctime>

// for using OpenCV operation using Opencv4Tegra
// optimized OpenCV function using GPU (therefore, the original GPU function is removed)
#include "opencv2/opencv.hpp"

#define CNUM 20  // number of colors

// setup the initial name
// please adding namespace using both cv and dlib simultaneously
using namespace std;
using namespace cv;

enum ColorInfo
{
  BLUE,
  LIME,
  RED,
  YELLOW,
  CYAN,
  MAGNETA,
  SILVER,
  NAVY,
  GREEN,
  MAROON,
  OLIVE,
  TEAL,
  PURPLE,
  GRAY,
  BLACK,
  WHITE
};

class ColorStatus
{
public:
  ColorStatus()
  {
    // maximum 14ea, BGR color space
    // blue, lime(light green), red, yellow, cyan, magneta
    // silver, navy, green, maroon, olive, teal, purple, gray
    mBGRcolor_comb.create(16, 3, CV_32FC1);

    mBGRcolor_comb.at<float>(0, 0) = 255;
    mBGRcolor_comb.at<float>(0, 1) = 0;
    mBGRcolor_comb.at<float>(0, 2) = 0;  // blue
    mBGRcolor_comb.at<float>(1, 0) = 0;
    mBGRcolor_comb.at<float>(1, 1) = 255;
    mBGRcolor_comb.at<float>(1, 2) = 0;  // lime   (light green)
    mBGRcolor_comb.at<float>(2, 0) = 0;
    mBGRcolor_comb.at<float>(2, 1) = 0;
    mBGRcolor_comb.at<float>(2, 2) = 255;  // red
    mBGRcolor_comb.at<float>(3, 0) = 0;
    mBGRcolor_comb.at<float>(3, 1) = 255;
    mBGRcolor_comb.at<float>(3, 2) = 255;  // yellow
    mBGRcolor_comb.at<float>(4, 0) = 255;
    mBGRcolor_comb.at<float>(4, 1) = 255;
    mBGRcolor_comb.at<float>(4, 2) = 0;  // cyan
    mBGRcolor_comb.at<float>(5, 0) = 255;
    mBGRcolor_comb.at<float>(5, 1) = 0;
    mBGRcolor_comb.at<float>(5, 2) = 255;  // magenta
    mBGRcolor_comb.at<float>(6, 0) = 192;
    mBGRcolor_comb.at<float>(6, 1) = 192;
    mBGRcolor_comb.at<float>(6, 2) = 192;  // silver (light gray)
    mBGRcolor_comb.at<float>(7, 0) = 128;
    mBGRcolor_comb.at<float>(7, 1) = 0;
    mBGRcolor_comb.at<float>(7, 2) = 0;  // navy   (dark blue)
    mBGRcolor_comb.at<float>(8, 0) = 0;
    mBGRcolor_comb.at<float>(8, 1) = 128;
    mBGRcolor_comb.at<float>(8, 2) = 0;  // green  (dark green)
    mBGRcolor_comb.at<float>(9, 0) = 0;
    mBGRcolor_comb.at<float>(9, 1) = 0;
    mBGRcolor_comb.at<float>(9, 2) = 128;  // maroon (dark red)
    mBGRcolor_comb.at<float>(10, 0) = 0;
    mBGRcolor_comb.at<float>(10, 1) = 128;
    mBGRcolor_comb.at<float>(10, 2) = 128;  // Olive  (dark yellow)
    mBGRcolor_comb.at<float>(11, 0) = 128;
    mBGRcolor_comb.at<float>(11, 1) = 128;
    mBGRcolor_comb.at<float>(11, 2) = 0;  // teal   (dark cyan)
    mBGRcolor_comb.at<float>(12, 0) = 128;
    mBGRcolor_comb.at<float>(12, 1) = 0;
    mBGRcolor_comb.at<float>(12, 2) = 128;  // purple (dark margenta)
    mBGRcolor_comb.at<float>(13, 0) = 128;
    mBGRcolor_comb.at<float>(13, 1) = 128;
    mBGRcolor_comb.at<float>(13, 2) = 128;  // gray (dark gray)
    mBGRcolor_comb.at<float>(14, 0) = 0;
    mBGRcolor_comb.at<float>(14, 1) = 0;
    mBGRcolor_comb.at<float>(14, 2) = 0;  // black
    mBGRcolor_comb.at<float>(15, 0) = 255;
    mBGRcolor_comb.at<float>(15, 1) = 255;
    mBGRcolor_comb.at<float>(15, 2) = 255;  // white

    scalBlue = Scalar((int)(mBGRcolor_comb.at<float>(0, 0)), (int)(mBGRcolor_comb.at<float>(0, 1)),
                      (int)(mBGRcolor_comb.at<float>(0, 2)));
    scalLime = Scalar((int)(mBGRcolor_comb.at<float>(1, 0)), (int)(mBGRcolor_comb.at<float>(1, 1)),
                      (int)(mBGRcolor_comb.at<float>(1, 2)));
    scalRed = Scalar((int)(mBGRcolor_comb.at<float>(2, 0)), (int)(mBGRcolor_comb.at<float>(2, 1)),
                     (int)(mBGRcolor_comb.at<float>(2, 2)));
    scalYellow = Scalar((int)(mBGRcolor_comb.at<float>(3, 0)), (int)(mBGRcolor_comb.at<float>(3, 1)),
                        (int)(mBGRcolor_comb.at<float>(3, 2)));
    scalCyan = Scalar((int)(mBGRcolor_comb.at<float>(4, 0)), (int)(mBGRcolor_comb.at<float>(4, 1)),
                      (int)(mBGRcolor_comb.at<float>(4, 2)));
    scalMagneta = Scalar((int)(mBGRcolor_comb.at<float>(5, 0)), (int)(mBGRcolor_comb.at<float>(5, 1)),
                         (int)(mBGRcolor_comb.at<float>(5, 2)));
    scalSilver = Scalar((int)(mBGRcolor_comb.at<float>(6, 0)), (int)(mBGRcolor_comb.at<float>(6, 1)),
                        (int)(mBGRcolor_comb.at<float>(6, 2)));
    scalNavy = Scalar((int)(mBGRcolor_comb.at<float>(7, 0)), (int)(mBGRcolor_comb.at<float>(7, 1)),
                      (int)(mBGRcolor_comb.at<float>(7, 2)));
    scalGreen = Scalar((int)(mBGRcolor_comb.at<float>(8, 0)), (int)(mBGRcolor_comb.at<float>(8, 1)),
                       (int)(mBGRcolor_comb.at<float>(8, 2)));
    scalMaroon = Scalar((int)(mBGRcolor_comb.at<float>(9, 0)), (int)(mBGRcolor_comb.at<float>(9, 1)),
                        (int)(mBGRcolor_comb.at<float>(9, 2)));
    scalOlive = Scalar((int)(mBGRcolor_comb.at<float>(10, 0)), (int)(mBGRcolor_comb.at<float>(10, 1)),
                       (int)(mBGRcolor_comb.at<float>(10, 2)));
    scalTeal = Scalar((int)(mBGRcolor_comb.at<float>(11, 0)), (int)(mBGRcolor_comb.at<float>(11, 1)),
                      (int)(mBGRcolor_comb.at<float>(11, 2)));
    scalPurple = Scalar((int)(mBGRcolor_comb.at<float>(12, 0)), (int)(mBGRcolor_comb.at<float>(12, 1)),
                        (int)(mBGRcolor_comb.at<float>(12, 2)));
    scalGray = Scalar((int)(mBGRcolor_comb.at<float>(13, 0)), (int)(mBGRcolor_comb.at<float>(13, 1)),
                      (int)(mBGRcolor_comb.at<float>(13, 2)));
    scalBlack = Scalar((int)(mBGRcolor_comb.at<float>(14, 0)), (int)(mBGRcolor_comb.at<float>(14, 1)),
                       (int)(mBGRcolor_comb.at<float>(14, 2)));
    scalWhite = Scalar((int)(mBGRcolor_comb.at<float>(15, 0)), (int)(mBGRcolor_comb.at<float>(15, 1)),
                       (int)(mBGRcolor_comb.at<float>(15, 2)));

    MapColorStatus();
  }
  ~ColorStatus()
  {
  }
  void MapColorStatus()
  {
    mapColorInfo["blue"] = ColorInfo::BLUE;
    mapColorInfo["lime"] = ColorInfo::LIME;
    mapColorInfo["red"] = ColorInfo::RED;
    mapColorInfo["yellow"] = ColorInfo::YELLOW;
    mapColorInfo["cyan"] = ColorInfo::CYAN;
    mapColorInfo["magneta"] = ColorInfo::MAGNETA;
    mapColorInfo["silver"] = ColorInfo::SILVER;
    mapColorInfo["navy"] = ColorInfo::NAVY;
    mapColorInfo["green"] = ColorInfo::GREEN;
    mapColorInfo["maroon"] = ColorInfo::MAROON;
    mapColorInfo["olive"] = ColorInfo::OLIVE;
    mapColorInfo["teal"] = ColorInfo::TEAL;
    mapColorInfo["purple"] = ColorInfo::PURPLE;
    mapColorInfo["gray"] = ColorInfo::GRAY;
    mapColorInfo["black"] = ColorInfo::BLACK;
    mapColorInfo["white"] = ColorInfo::WHITE;

    // initializing random color
    cv::RNG rng(0xFFFFFFFF);
    for (int i = 0; i < CNUM; i++)
      rng.fill(randColor[i], cv::RNG::UNIFORM, 0, 256);
  }
  Scalar GetColorStatus(int nColorInfo)
  {
    Scalar res;
    switch (nColorInfo)
    {
      case BLUE: {
        res = scalBlue;
        break;
      }
      case LIME: {
        res = scalLime;
        break;
      }
      case RED: {
        res = scalRed;
        break;
      }
      case YELLOW: {
        res = scalYellow;
        break;
      }
      case CYAN: {
        res = scalCyan;
        break;
      }
      case MAGNETA: {
        res = scalMagneta;
        break;
      }
      case SILVER: {
        res = scalSilver;
        break;
      }
      case NAVY: {
        res = scalNavy;
        break;
      }
      case GREEN: {
        res = scalGreen;
        break;
      }
      case MAROON: {
        res = scalMaroon;
        break;
      }
      case OLIVE: {
        res = scalOlive;
        break;
      }
      case TEAL: {
        res = scalTeal;
        break;
      }
      case PURPLE: {
        res = scalPurple;
        break;
      }
      case GRAY: {
        res = scalGray;
        break;
      }
      case BLACK: {
        res = scalBlack;
        break;
      }
      case WHITE: {
        res = scalWhite;
        break;
      }
      default: {
        res = scalWhite;
        break;
      }
    }
    return res;
  }

  Scalar GenColorInfo(string strInput)
  {
    Scalar scalRes;
    if (strInput == "blue")
      scalRes = scalBlue;
    else if (strInput == "lime")
      scalRes = scalLime;
    else if (strInput == "red")
      scalRes = scalRed;
    else if (strInput == "yellow")
      scalRes = scalYellow;
    else if (strInput == "cyan")
      scalRes = scalCyan;
    else if (strInput == "magneta")
      scalRes = scalMagneta;
    else if (strInput == "silver")
      scalRes = scalSilver;
    else if (strInput == "navy")
      scalRes = scalNavy;
    else if (strInput == "green")
      scalRes = scalGreen;
    else if (strInput == "maroon")
      scalRes = scalMaroon;
    else if (strInput == "olive")
      scalRes = scalOlive;
    else if (strInput == "teal")
      scalRes = scalTeal;
    else if (strInput == "purple")
      scalRes = scalPurple;
    else if (strInput == "gray")
      scalRes = scalGray;
    else if (strInput == "black")
      scalRes = scalBlack;
    else if (strInput == "white")
      scalRes = scalWhite;
    else
      scalRes = scalBlack;
    return scalRes;
  }

  // color combinations, BGR
  Mat mBGRcolor_comb;

  // define color
  Scalar scalBlue;
  Scalar scalLime;
  Scalar scalRed;
  Scalar scalYellow;
  Scalar scalCyan;
  Scalar scalMagneta;
  Scalar scalSilver;
  Scalar scalNavy;
  Scalar scalGreen;
  Scalar scalMaroon;
  Scalar scalOlive;
  Scalar scalTeal;
  Scalar scalPurple;
  Scalar scalGray;
  Scalar scalBlack;
  Scalar scalWhite;

  map<std::string, ColorInfo> mapColorInfo;
  cv::Scalar_<int> randColor[CNUM];
};

#endif