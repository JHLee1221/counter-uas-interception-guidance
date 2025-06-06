#include "counter_uas_offboard_ctrl_lib.h"  

using namespace std;
using namespace rclcpp;

CounterUasOffboardCtrlLib::CounterUasOffboardCtrlLib(const ConfigParamOffboardCtrl& cfg, shared_ptr<rclcpp::Node> node_)
  : cfgParam_(cfg)
  , node_(node)
  , bOffboardMode_(false)
  , bArmState_(false)
  , bTakeoffState_(false)
  , bHasUasOdomResCallback_(false)
  , bHasTgtOdomResCallback_(false)
  , bHasMountAngResCallback_(false)
  , dMountYawAng_(0.0)
  , dTakeoffAlt_(0.0)
  , dMaxYawRate_(0.0)
  , dKpAlt_(0.0)
  , dCuiVelX_(0.0)
  , dCuiVelY_(0.0)
  , dCuiVelZ_(0.0)
{
  // QoS profile
  auto qos = QoS(KeepLast(10)).best_effort();
  
  // Generating counter-uas state subscriber
  RCLCPP_INFO(node_->get_logger(), "Subscriber(conter-uas state): %s", cfgParam_.strUasStaTpNmSrc.c_str());
  subUasState_ = node_->create_subscription<mavros_msgs::msg::State>(
    cfgParam_.strUasStaTpNmSrc, QoS(10),
    bind(&CounterUasOffboardCtrlLib::CbUasState, this, placeholders::_1));
  
  // Generating conter-uas odom subscriber
  RCLCPP_INFO(node_->get_logger(), "Subscriber(conter-uas odom): %s", cfgParam_.strUasOdomTpNmSrc.c_str());
  subUasOdom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    cfgParam_.strUasOdomTpNmSrc, qos,
    bind(&CounterUasOffboardCtrlLib::CbUasOdom, this, placeholders::_1));

  // Generating target odom subscriber
  RCLCPP_INFO(node_->get_logger(), "Subscriber(target odom): %s", cfgParam_.strTgtOdomTpNmSrc.c_str());
  subTgtOdom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    cfgParam_.strTgtOdomTpNmSrc, QoS(10),
    bind(&CounterUasOffboardCtrlLib::CbTgtOdom, this, placeholders::_1));
  
  // Generating counter-uas's mount angle subscriber
  RCLCPP_INFO(node_->get_logger(), "Subscriber(counter-uas's mount angle): %s", cfgParam_.strMntAngTpNmSrc.c_str());
  subMountAng_ = node_->create_subscription<mavros_msgs::msg::MountControl>(
    cfgParam_.strMntAngTpNmSrc, QoS(10),
    bind(&CounterUasOffboardCtrlLib::CbMountAng, this, placeholders::_1));
  
  // Generating counter-uas vel cmd publisher
  RCLCPP_INFO(node_->get_logger(), "Publisher(counter-uas's vel cmd): %s", cfgParam_.strUasSetPtTpNmDst.c_str());
  pubPosTgt_ = node_->create_publisher<mavros_msgs::msg::PositionTarget>(
    cfgParam_.strUasSetPtTpNmDst, QoS(10));
  
  // Generating counter-uas arm service client
  RCLCPP_INFO(node_->get_logger(), "Client(counter-uas arm service): %s", cfgParam_.strUasArmSrvNmSrc.c_str());
  cliArm_ = node_->create_client<mavros_msgs::srv::CommandBool>(
    cfgParam_.strUasArmSrvNmSrc);
  
  // Generating counter-uas set mode service client
  RCLCPP_INFO(node_->get_logger(), "Client(counter-uas set mode service): %s", cfgParam_.strUasStMdSrvNmSrc.c_str());
  cliSetMode_ = node_->create_client<mavros_msgs::srv::SetMode>(
    cfgParam_.strUasStMdSrvNmSrc);
  
  WaitForServices();
  lastReq_ = node_->now();

  yawRateCtrl_ = make_unique<CounterUASGuidanceYawRateCtrl>(
    cfgParam_.nFrameRate, cfgParam_.dKp, cfgParam_.dKi, cfgParam_.dKd, 
    cfgParam_.dAdpGain, cfgParam_.dAlpha, cfgParam_.dYawAngLmt, cfgParam_.dYawRateLmt);

  dTakeoffAlt_ = cfgParam_.dTakeoffAlt;
  dMaxYawRate_ = cfgParam_.dMaxYawRate;
  dKpAlt_ = cfgParam_.dKpAlt;

  bOffboardMode_ = false;
  bArmState_ = false;
  bTakeoffState_ = false;
  bHasUasOdomResCallback_ = false;
  bHasTgtOdomResCallback_ = false;
  bHasMountAngResCallback_ = false;
}

CounterUasOffboardCtrlLib::~CounterUasOffboardCtrlLib()
{
}

void CounterUasOffboardCtrlLib::WaitForServices()
{
  while (!cliArm_->wait_for_service(chrono::seconds(1))) 
  {
    RCLCPP_INFO(node_->get_logger(), "Waiting for arming service...");
  }
  while (!cliSetMode_->wait_for_service(chrono::seconds(1)))
  {
    RCLCPP_INFO(node_->get_logger(), "Waiting for set_mode service..."); 
  }
}

void CounterUasOffboardCtrlLib::CbUasState(const mavros_msgs::msg::State::SharedPtr msgState)
{
  currentState_ = *msgState;
}

void CounterUasOffboardCtrlLib::CbUasOdom(const nav_msgs::msg::Odometry::SharedPtr msgOdom)
{
  if (msgOdom->pose.empty())
  {
    RCLCPP_WARN(node_->get_logger(), "CbUasOdom: No counter uas odometry data received.");
    bHasUasOdomResCallback_ = false; // No new data
    return;
  }

  vecUasPos_ << msgOdom->pose.pose.position.x, msgOdom->pose.pose.position.y, msgOdom->pose.pose.position.z;
  quatUasOri_ = Eigen::Quaterniond(
    msgOdom->pose.pose.orientation.w,
    msgOdom->pose.pose.orientation.x,
    msgOdom->pose.pose.orientation.y,
    msgOdom->pose.pose.orientation.z);

  if (!vecUasPos_.empty())
  {
    bHasUasOdomResCallback_ = true; // New data received
    RCLCPP_INFO(node_->get_logger(), "CbUasOdom: Received counter uas odometry data.");
  }
  else
  {
    bHasUasOdomResCallback_ = false; // No new data
    RCLCPP_WARN(node_->get_logger(), "CbUasOdom: No valid odometry data after processing.");
  }
}

void CounterUasOffboardCtrlLib::CbTgtOdom(const nav_msgs::msg::Odometry::SharedPtr msgOdom)
{
  if (msgOdom->pose.empty())
  {
    RCLCPP_WARN(node_->get_logger(), "CbTgtOdom: No target odometry data received.");
    bHasTgtOdomResCallback_ = false; // No new data
    return;
  }

  vecTgtPos_ << msgOdom->pose.pose.position.x, msgOdom->pose.pose.position.y, msgOdom->pose.pose.position.z;

  if (!vecTgtPos_.empty())
  {
    bHasTgtOdomResCallback_ = true; // New data received
    RCLCPP_INFO(node_->get_logger(), "CbTgtOdom: Received target odometry data.");
  }
  else
  {
    bHasTgtOdomResCallback_ = false; // No new data
    RCLCPP_WARN(node_->get_logger(), "CbTgtOdom: No valid odometry data after processing.");
  }
}

void CounterUasOffboardCtrlLib::CbMountAng(const mavros_msgs::msg::MountControl::SharedPtr msgMountAng)
{
  if (msgMountAng->yaw.empty())
  {
    RCLCPP_WARN(node_->get_logger(), "CbMountAng: No mount angle data received.");
    bHasMountAngResCallback_ = false; // No new data
    return; // No new data
  }
  dMountYawAng_ = msgMountAng->yaw * D2R; // Convert to radians

  if (dMountYawAng_ != 0.0)
  {
    bHasMountAngResCallback_ = true; // New data received
    RCLCPP_INFO(node_->get_logger(), "CbMountAng: Received mount angle data.");
  }
  else
  {
    bHasMountAngResCallback_ = false; // No new data
    RCLCPP_WARN(node_->get_logger(), "CbMountAng: No valid mount angle data after processing.");
  }
  // RCLCPP_INFO(node_->get_logger(), "CbMountAng: Received mount angle data: %f", dMountYawAng_ * R2D);
}

void CounterUasOffboardCtrlLib::LaunchGuidance()
{
  // Check the current mode and arm status of counter-uas
  if (currentState_.mode != "OFFBOARD" && (node_->now() - lastReq_).seconds() > 5.0) 
  {
    auto req = make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = "OFFBOARD";
    cliSetMode_->async_send_request(req);
    lastReq_ = node_->now();
  } 
  else if (!currentState_.armed && (node_->now() - lastReq_).seconds() > 5.0) 
  {
    auto req = make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = true;
    cliArm_->async_send_request(req);
    lastReq_ = node_->now();
  }

  if (currentState_.mode == "OFFBOARD" && currentState_.armed) {
      bOffboardMode_ = true;
      bArmState_ = true;
  }

  vecLosVec_ = vecTgtPos_ - vecUasPos_;
  if (vecLosVec_.norm() < 1e-3)
  {
    RCLCPP_WARN(node_->get_logger(), "Los vector norm is too small, skipping control...");
    return;
  }

  Eigen::Vector3d euler = quatUasOri_.toRotationMatrix().eulerAngles(0, 1, 2);
  
  dUasYawAng_ = euler.z();
  dLosYawAng_ = atan2(vecLosVec_.y(), vecLosVec_.x());

  dYawRate_ = yawRateCtrl_->dUpdateYawRate(dUasYawAng_, dLosYawAng_);
  dYawRate_ = std::clamp(dYawRate_, -dMaxYawRate_, dMaxYawRate_);

  dYawErr_ = yawRateCtrl_->dGetLastYawErr();
  
  // dCuiVelX_ = 
  // dCuiVelY_ = 
  dCuiVelZ_ = std::clamp(dKpAlt_ * (dTakeoffAlt_ - vecUasPos_.z()), -1 * cfgParam_.dAltVelLmt, cfgParam_.dAltVelLmt);

  mavros_msgs::msg::PositionTarget posTgtMsg;
  posTgtMsg.header.stamp = node_->now();
  posTgtMsg.header.frame_id = "counter_uas_offboard_ctrl";
  posTgtMsg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  posTgtMsg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
                        mavros_msgs::msg::PositionTarget::IGNORE_PY |
                        mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                        mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                        mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                        mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::msg::PositionTarget::IGNORE_YAW;
  
  if (bOffboardMode_ && bArmState_ && !bTakeoffState)
  {
    posTgtMsg.velocity.x = 0.0; // dCuiVelX_;
    posTgtMsg.velocity.y = 0.0; // dCuiVelY_;
    posTgtMsg.velocity.z = dCuiVelZ_;
    posTgtMsg.yaw_rate = 0.0;
    pubPosTgt_->publish(posTgtMsg);
    bTakeoffState_ = true; // Takeoff state is set to true after publishing the position target
  }
  else if (bTakeoffState_ && bHasTgtOdomResCallback_)
  {
    posTgtMsg.velocity.x = 1.0; // dCuiVelX_;
    posTgtMsg.velocity.y = 1.0; // dCuiVelY_;
    posTgtMsg.velocity.z = dCuiVelZ_;
    posTgtMsg.yaw_rate = dYawRate_;
    pubPosTgt_->publish(posTgtMsg);
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "Counter-uas is not in OFFBOARD mode or not armed, skipping control...");
    return; // Skip control if not in OFFBOARD mode or not armed
  }

  RCLCPP_INFO(node_->get_logger(), "Published PositionTarget: Vel(%f, %f, %f), YawRate(%f)",
              dCuiVelX_, dCuiVelY_, dCuiVelZ_, dYawRate_, dYawErr_);
}

void CounterUasOffboardCtrlLib::MainOffboardLoop()
{
  if (!bHasUasOdomResCallback_)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                          "Waiting for counter-uas odometry data, bHasUasOdomResCallback_: %d",
                          (int)(bHasUasOdomResCallback_));
    return;
  }

  if (!bHasMountAngResCallback_)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                          "Waiting for mount angle data, bHasMountAngResCallback_: %d",
                          (int)(bHasMountAngResCallback_));
    return;
  }

  if (!bOffboardMode_)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                          "Counter-uas is not in OFFBOARD mode, bOffboardMode_: %d",
                          (int)(bOffboardMode_));
    return;
  }

  if (!bArmState_)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                          "Counter-uas is not armed, bArmState_: %d",
                          (int)(bArmState_));
    return;
  }

  LaunchGuidance();
  RCLCPP_INFO(node_->get_logger(), "Counter-uas offboard control loop executed successfully.");
}