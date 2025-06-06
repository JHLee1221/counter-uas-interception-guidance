#include "counter_uas_offboard_ctrl.h"

using namespace std;
using namespace rclcpp;

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

 int main(int argc, char **argv)
{
  // Set up ROS2
  init(argc, argv);

  node = make_shared<rclcpp::Node>("counter_uas_offboard_ctrl_node");

  //Reading ROS2 params
  ConfigParamOffboardCtrl cfg(node);
  if (!cfg.readParams())
  {
    RCLCPP_ERROR(node->get_logger(), "Wrong params!! Please check the parameter sheet..");
    shutdown();
    return -1;
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Pass:readParams()..");
  }

  // Setting main function class
  RCLCPP_INFO(node->get_logger(), "Counter Uas Offboard Control Node");
  CounterUasOffboardCtrlLib counterUasOffboardCtrl(cfg, node);
  signal(SIGINT, SigIntHandler);

  // Setting main function loop rate
  Rate loopRate(cfg.nFrameRate);

  // Main loop
  while (ok())
  {
    try
    {
      // Main loop function
      counterUasOffboardCtrl.MainOffboardLoop();

      // Loop rate setting and pub/sub spin
      spin_some(node);
    }
    catch(const std::exception& e)
    {
      //RCLCPP_ERROR(node->get_logger(), "Exception in main loop: %s", e.what());
      return -1;
    }
    loopRate.sleep();
  }

  shutdown();
  node.reset(); // Reset node to free resources
  return 0; 
}