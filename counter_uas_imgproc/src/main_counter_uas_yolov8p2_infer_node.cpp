#include "counter_uas_yolov8p2_infer_lib.h"

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

  node = make_shared<rclcpp::Node>("counter_uas_yolov8p2_infer_node");

  // Reading ROS2 params
  ConfigParamDetection cfg(node);
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

  // Setting main function class
  RCLCPP_INFO(node->get_logger(), "YoloV8P2 Model Inference for ROS-enabled Illegal Drone Detection(Counter Uas System)");
  CounterUasYoloV8P2InferLib counterUasYoloV8P2Infer(cfg, node);
  signal(SIGINT, SigIntHandler);

  // Setting main function loop rate
  Rate loopRate(cfg.nFrameRate);

  // Main loop
  while (ok())
  {
    try
    {
      // Main loop function
      counterUasYoloV8P2Infer.MainInferLoop(cfg.nPlayMode);
      
      // Loop rate setting and pub/sub spin
      spin_some(node);
      
    }
    catch(const std::exception& e)
    {
      //RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
      break;
    }
    loopRate.sleep();
  }
  
  shutdown();
  node.reset();
  return 0;
 }

