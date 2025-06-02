#include "siyi_eo_status_lib.h"

using namespace std;
using namespace rclcpp;
using namespace cv;

shared_ptr<rclcpp::Node> node;

// using SIGINT handler
void SigIntHandler([[maybe_unused]] int param)
{
  if (node) 
  {
    RCLCPP_INFO(node->get_logger(), "User pressed Ctrl+C..forced exit..");
  }
  shutdown();
}

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char** argv)
{
  // Set up ROS2
  init(argc, argv);
  
  node = make_shared<rclcpp::Node>("siyi_eo_status_node");

  // Reading ros2 params
  ConfigParam cfg(node);
  if (!cfg.GetRosParams())
  {
    RCLCPP_ERROR(node->get_logger(), "Wrong params!! Please check the parameter sheet..");
    shutdown();
    return -1;
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Pass:GetRosParams()..");
  }

  SiyiEoStatusLib siyiEoStatus(cfg, node);
  signal(SIGINT, SigIntHandler);

  Rate loopRate(30);

  while (ok())
  {
    try
    {
      // main loop function
      siyiEoStatus.MainStatusLoop();
      spin_some(node);
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
      break;
    }
    loopRate.sleep();
  }

  //siyiEoStatus.~SiyiEoStatusLib();
  siyiEoStatus.ShutImageSrc();
  rclcpp::shutdown();
  node.reset();
  return 0;
}