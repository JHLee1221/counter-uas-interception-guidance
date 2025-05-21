#include "misc_func.h"

using namespace std;
using namespace rclcpp;

MiscFunction::MiscFunction(const ConfigParam& cfg, std::shared_ptr<rclcpp::Node> node)
 : cfgParam_(cfg), node_(node)
{
}

MiscFunction::~MiscFunction()
{
}