#ifndef KARI_DRONECOP_RD_IMGPROC_MISC_FUNC_H
#define KARI_DRONECOP_RD_IMGPROC_MISC_FUNC_H

#include "global_header.h"
#include "config_param.h"

using namespace std;
using namespace rclcpp;

class MiscFunction
{
public:
  MiscFunction(const ConfigParam& cfg, std::shared_ptr<rclcpp::Node> node);
  ~MiscFunction();

private:
  ConfigParam cfgParam_;
  std::shared_ptr<rclcpp::Node> node_;
};

#endif  // KARI_DRONECOP_RD_IMGPROC_MISC_FUNC_H
