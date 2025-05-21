#include "misc_calc_lib.h"

using namespace std;
using namespace rclcpp;
using namespace cv;

MiscCalc::MiscCalc(const ConfigParam& cfg, std::shared_ptr<rclcpp::Node> node)
 : cfgParam_(cfg), node_(node)
{
  // initializing variables
  bSizeCalcFlag = false;
  bVidStatus = false;
}

MiscCalc::~MiscCalc()
{
}

// drawing the vector rect w.r.t label info
void MiscCalc::DrawVectorLabeledRect(Mat imgSrc, vector<BboxInfo> vecInfo)
{
  if (vecInfo.size() > 0)
  {
    for (auto i = 0; i < vecInfo.size(); i++)
      DrawSingleLabeledRect(imgSrc, vecInfo[i]);
  }
}

// drawing the single rect w.r.t label info
void MiscCalc::DrawSingleLabeledRect(Mat imgSrc, BboxInfo bboxInfo)
{
  Point ptStrRes;
  ptStrRes.x = bboxInfo.rectBbox.tl().x;
  ptStrRes.y = bboxInfo.rectBbox.tl().y - 5;
  string strLabelConfidenceInfo = bboxInfo.labelInfo.strLabel + ", conf: " + GetConfString(bboxInfo.fConfidence, 4);
  putText(imgSrc, strLabelConfidenceInfo, ptStrRes, 2, 0.35, bboxInfo.labelInfo.scalColor, 1, 8, 0);
  rectangle(imgSrc, bboxInfo.rectBbox, bboxInfo.labelInfo.scalColor, 2, 8, 0);
}

// saving the raw image to the image file
void MiscCalc::SaveImgRaw(Mat imgSrc, string path)
{
  string strSaveImgRes;
  strSaveImgRes = path + cfgParam_.GenLocalTimeStringNormal() + ".png";
  imwrite(strSaveImgRes, imgSrc);
  return;
}

// saving the result image to the image file
void MiscCalc::SaveImgRes(Mat imgSrc, string pathImg, string pathLabel, vector<BboxInfo> vecInfo, bool bFeatUse)
{
  string strSaveImgRes;
  string strCurrTime = cfgParam_.GenLocalTimeStringNormal();
  strSaveImgRes = pathImg + strCurrTime + ".png";
  imwrite(strSaveImgRes, imgSrc);
  if (bFeatUse)
  {
    string strSaveTxtRes;
    strSaveTxtRes = pathLabel + strCurrTime + ".txt";
    std::ofstream outTotalResTxt;
    outTotalResTxt.open(strSaveTxtRes, std::ios_base::app);

    // yolo style: label, xcen_norm, ycen_norm, bbox_width_norm, bbox_height_norm
    if (vecInfo.size() == 0)
    {
      // only saving the empty text result
      outTotalResTxt << "0 0.0 0.0 0.0 0.0" << endl;
      return;
    }
    else
    {
      for (auto i = 0; i < vecInfo.size(); i++)
      {
        outTotalResTxt << vecInfo[i].nLabel << " ";
        for (auto j = 0; j < 4; j++)
          outTotalResTxt << vecInfo[i].fYoloData[j] << " ";
        outTotalResTxt << endl;
      }
      outTotalResTxt.close();
      return;
    }
  }
  return;
}

// generating size calculation flag for terminating converter
bool MiscCalc::GenSizeCalcFlag(int nSize, int nTotal)
{
  bool bRes = false;
  if (fabs(nSize - nTotal) == 1.0f)
    bRes = true;
  else
    RCLCPP_INFO(node_->get_logger(), " ");
  return bRes;
}

// generating img size(width, height)
Size MiscCalc::GenImgSize(Mat imgSrc)
{
  Size szRes;
  szRes.width = imgSrc.cols;
  szRes.height = imgSrc.rows;
  return szRes;
}

// getting video status flag
bool MiscCalc::GetVidFlag()
{
  return bVidStatus;
}

// getting size calculation flag
bool MiscCalc::GetSizeCalcFlag()
{
  return bSizeCalcFlag;
}

// getting the value with the flotting point
string MiscCalc::GetConfString(float fInput, int nNumFlot)
{
  std::ostringstream outConf;
  outConf.precision(nNumFlot);
  outConf << std::fixed << fInput;
  return outConf.str();
}

// saturation, pixel-wise
int MiscCalc::Saturation(int val, int min, int max)
{
  int res = val;
  if (val < min)
    res = min;
  else if (val > max)
    res = max;
  else
    res = val;
  return res;
}
