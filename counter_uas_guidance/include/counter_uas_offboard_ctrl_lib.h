#ifndef COUNTER_UAS_GUIDANCE_OFFBOARD_MODE_CTRL_LIB_H
#define COUNTER_UAS_GUIDANCE_OFFBOARD_MODE_CTRL_LIB_H

#include "global_header.h"
#include "config_param_offboard_ctrl.h"
#include "yaw_rate_ctrl.h"

class CounterUasOffboardCtrlLib
{
public:
  CounterUasOffboardCtrlLib(const ConfigParamOffboardCtrl& cfg, std::shared_ptr<rclcpp::Node> node);
  ~CounterUasOffboardCtrlLib();

  void MainOffboardLoop();

private:
  ConfigParamOffboardCtrl cfgParam_;
  std::shared_ptr<rclcpp::Node> node_;

  std::unique_ptr<CounterUASGuidanceYawRateCtrl> yawRateCtrl_;

  mavros_msgs::msg::State currentState_;

  // Counter uas state
  Eigen::Vector3d vecUasPos_;
  Eigen::Quaterniond quatUasOri_;

  // Target state
  Eigen::Vector3d vecTgtPos_;

  // LoS state
  Eigen::Vector3d vecLosVec_;

  bool bOffboardMode_;
  bool bArmState_;
  bool bTakeoffState_;
  bool bHasUasOdomResCallback_;
  bool bHasTgtOdomResCallback_;
  bool bHasMountAngResCallback_;

  // Current yaw angle and rate of the counter-uas
  double dUasYawAng_; // Current yaw angle of the counter-uas in radians
  double dYawRate_; // Current yaw rate in radians per second
  double dYawErr_; // Current yaw error in radians

  double dLosYawAng_; // Current LoS yaw angle in radians
  double dMountYawAng_; // Current mount yaw angle in radians
  double dTakeoffAlt_; // Takeoff altitude in meters
  double dMaxYawRate_; // Maximum yaw rate in radians per second
  double dKpAlt_; // Proportional gain for altitude control

  double dCuiVelX_;
  double dCuiVelY_;
  double dCuiVelZ_;

  void WaitForServices();
  void CbUasState(const mavros_msgs::msg::State::SharedPtr msgState);
  void CbUasOdom(const nav_msgs::msg::Odometry::SharedPtr msgOdom);
  void CbTgtOdom(const nav_msgs::msg::Odometry::SharedPtr msgOdom);
  void CbMountAng(const mavros_msgs::msg::MountControl::SharedPtr msgMountAng);
  void LaunchGuidance();

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subUasState_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subUasOdom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subTgtOdom_;
  rclcpp::Subscription<mavros_msgs::msg::MountControl>::SharedPtr subMountAng_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr pubPosTgt_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr cliArm_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr cliSetMode_;
  rclcpp::Time lastReq_;

};

#endif // COUNTER_UAS_GUIDANCE_OFFBOARD_MODE_CTRL_LIB_H