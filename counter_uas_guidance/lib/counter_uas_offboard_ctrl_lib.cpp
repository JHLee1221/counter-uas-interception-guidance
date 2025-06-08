#include "counter_uas_offboard_ctrl_lib.h"  

using namespace std;
using namespace rclcpp;
using counter_uas_guidance::msg::GimbalMode;

CounterUasOffboardCtrlLib::CounterUasOffboardCtrlLib(const ConfigParamOffboardCtrl& cfg, shared_ptr<rclcpp::Node> node)
  : cfgParam_(cfg)
  , node_(node)
  , phase_(GuidancePhase::INIT)
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
  
  RCLCPP_INFO(node_->get_logger(), "Publisher(counter-uas's gimbal mode): %s", cfgParam_.strUasGimbalMdDst.c_str());
  pubGimbalMd_ = node_->create_publisher<counter_uas_guidance::msg::GimbalMode>(
    cfgParam_.strUasGimbalMdDst, QoS(10));

  RCLCPP_INFO(node_->get_logger(), "Create counter uas offboard ctrl timer");
  timer_ = node_->create_wall_timer(chrono::milliseconds(cfgParam_.nFrameRate), bind(&CounterUasOffboardCtrlLib::MainGuidanceLoop, this));

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
  RCLCPP_INFO_ONCE(node_->get_logger(), "CbUasState: Received FCU state update. Mode: %s, Armed: %d",
                   currentState_.mode.c_str(), currentState_.armed);
}


void CounterUasOffboardCtrlLib::CbUasOdom(const nav_msgs::msg::Odometry::SharedPtr msgOdom)
{
  if (std::isnan(msgOdom->pose.pose.position.x))
  {
    RCLCPP_WARN(node_->get_logger(), "CbUasOdom: No counter uas odometry data received.");
    bHasUasOdomResCallback_ = false; // No new data
    return;
  }

  const auto& pos = msgOdom->pose.pose.position;
  vecUasPos_ << pos.y, pos.x, -pos.z;  // NED: (North, East, Down)
  
  const auto& q = msgOdom->pose.pose.orientation;

  convertQuaternionToEuler(q.x, q.y, q.z, q.w, dRoll, dPitch, dYaw);
  
  // ENU 
  dRollRadENU_ = dRoll;
  dPitchRadENU_ = dPitch;
  dYawRadENU_ = dYaw;

  // ENU to NED
  dYawRadNED_ = -dYawRadENU_;
  dPitchRadNED_ = -dPitchRadENU_;
  dRollRadNED_ = dRollRadENU_;

  // NED rad to NED deg
  dRollDegNED_ = dRollRadNED_ * R2D;
  dPitchDegNED_ = dPitchRadNED_ * R2D;
  dYawDegNED_ = dYawRadNED_ * R2D;


  RCLCPP_INFO(node_->get_logger(),
    "Drone Attitude | Roll: %.2f deg | Pitch: %.2f deg | Yaw: %.2f deg",
    dRollDegNED_, dPitchDegNED_, dYawDegNED_);

  if (!vecUasPos_.isZero())
  {
    bHasUasOdomResCallback_ = true; // New data received
    RCLCPP_INFO_ONCE(node_->get_logger(), "CbUasOdom: Received counter uas odometry data.");
  }
  else
  {
    bHasUasOdomResCallback_ = false; // No new data
    RCLCPP_WARN(node_->get_logger(), "CbUasOdom: No valid odometry data after processing.");
  }
}

void CounterUasOffboardCtrlLib::CbTgtOdom(const nav_msgs::msg::Odometry::SharedPtr msgOdom)
{
  if (std::isnan(msgOdom->pose.pose.position.x))
  {
    RCLCPP_WARN(node_->get_logger(), "CbTgtOdom: No target odometry data received.");
    bHasTgtOdomResCallback_ = false; // No new data
    return;
  }

  const auto& pos = msgOdom->pose.pose.position;
  vecTgtPos_ << pos.x, -pos.y, -pos.z;  // NED: (North, East, Down)
  
  if (!vecTgtPos_.isZero())
  {
    bHasTgtOdomResCallback_ = true; // New data received
    RCLCPP_INFO_ONCE(node_->get_logger(), "CbTgtOdom: Received target odometry data.");
  }
  else
  {
    bHasTgtOdomResCallback_ = false; // No new data
    RCLCPP_WARN(node_->get_logger(), "CbTgtOdom: No valid odometry data after processing.");
  }
}

void CounterUasOffboardCtrlLib::CbMountAng(const mavros_msgs::msg::MountControl::SharedPtr msgMountAng)
{
  if (std::isnan(msgMountAng->yaw))
  {
    RCLCPP_WARN(node_->get_logger(), "CbMountAng: No mount angle data received.");
    bHasMountAngResCallback_ = false; // No new data
    return; // No new data
  }
  dMountYawAng_ = msgMountAng->yaw * D2R; // Convert to radians

  if (dMountYawAng_ != 0.0)
  {
    bHasMountAngResCallback_ = true; // New data received
    RCLCPP_INFO_ONCE(node_->get_logger(), "CbMountAng: Received mount angle data.");
  }
  else
  {
    bHasMountAngResCallback_ = false; // No new data
    RCLCPP_WARN(node_->get_logger(), "CbMountAng: No valid mount angle data after processing.");
  }
}

void CounterUasOffboardCtrlLib::PubGimbalMode(uint8_t state)
{
  counter_uas_guidance::msg::GimbalMode msg;
  msg.header.stamp = rclcpp::Clock().now();
  msg.state = state;
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                      "Current Gimbal Mode: %d", msg.state);
  pubGimbalMd_->publish(msg);
}

void CounterUasOffboardCtrlLib::MsgMavrosPositionTgt(mavros_msgs::msg::PositionTarget& msg)
{
  msg.type_mask = 
    mavros_msgs::msg::PositionTarget::IGNORE_PX |
    mavros_msgs::msg::PositionTarget::IGNORE_PY |
    mavros_msgs::msg::PositionTarget::IGNORE_PZ |
    mavros_msgs::msg::PositionTarget::IGNORE_AFX |
    mavros_msgs::msg::PositionTarget::IGNORE_AFY |
    mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
    mavros_msgs::msg::PositionTarget::IGNORE_YAW;
}

void CounterUasOffboardCtrlLib::PubKeepAliveSetPt()
{
  mavros_msgs::msg::PositionTarget keepAliveMsg;
  keepAliveMsg.header.stamp = node_->now();
  keepAliveMsg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

  MsgMavrosPositionTgt(keepAliveMsg);

  keepAliveMsg.velocity.x = 0.0;
  keepAliveMsg.velocity.y = 0.0;
  keepAliveMsg.velocity.z = 0.0;
  keepAliveMsg.yaw_rate = 0.0;
  pubPosTgt_->publish(keepAliveMsg);
}

void CounterUasOffboardCtrlLib::ReqOffboardMode()
{
  RCLCPP_INFO_ONCE(node_->get_logger(), "Sending OFFBOARD mode request...");
  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->custom_mode = "OFFBOARD";

  cliSetMode_->async_send_request(req,
    [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
    {
      auto res = future.get();
      if (res->mode_sent)
        RCLCPP_INFO_ONCE(node_->get_logger(), "OFFBOARD mode accepted.");
      else
        RCLCPP_ERROR(node_->get_logger(), "OFFBOARD mode rejected.");
    });

  lastReq_ = node_->now();
}

void CounterUasOffboardCtrlLib::ReqArming()
{
  RCLCPP_INFO_ONCE(node_->get_logger(), "Sending ARM request...");
  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = true;

  cliArm_->async_send_request(req,
    [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future)
    {
      auto res = future.get();
      if (res->success)
        RCLCPP_INFO_ONCE(node_->get_logger(), "Drone armed.");
      else
        RCLCPP_ERROR(node_->get_logger(), "Drone arming failed.");
    });

  lastReq_ = node_->now();
}

void CounterUasOffboardCtrlLib::CmdTakeoff()
{
  dCuiVelZ_ = std::clamp(dKpAlt_ * (dTakeoffAlt_ + vecUasPos_.z()), -1 * cfgParam_.dAltVelLmt, cfgParam_.dAltVelLmt);

  mavros_msgs::msg::PositionTarget posTgtMsg;
  posTgtMsg.header.stamp = node_->now();
  posTgtMsg.header.frame_id = "counter_uas_offboard_ctrl";
  posTgtMsg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

  MsgMavrosPositionTgt(posTgtMsg);

  posTgtMsg.velocity.x = 0.0;
  posTgtMsg.velocity.y = 0.0;
  posTgtMsg.velocity.z = dCuiVelZ_;
  posTgtMsg.yaw_rate = 0.0;
  pubPosTgt_->publish(posTgtMsg);
  PubGimbalMode(GimbalMode::SEARCH);
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                       "[DEBUG] Current altitude: %.2f", vecUasPos_.z());
}

void CounterUasOffboardCtrlLib::CmdBoostPhase()
{
  vecLosVec_ = vecTgtPos_ - vecUasPos_;

  RCLCPP_INFO(node_->get_logger(),
    "\n[LOS Vector Calculation Debug]"
    "\n  ▷ UAS Pos   : (X: %.2f, Y: %.2f, Z: %.2f)"
    "\n  ▷ TGT Pos   : (X: %.2f, Y: %.2f, Z: %.2f)"
    "\n  ▷ LOS Vec   : (X: %.2f, Y: %.2f, Z: %.2f)",
    vecUasPos_.x(), vecUasPos_.y(), vecUasPos_.z(),
    vecTgtPos_.x(), vecTgtPos_.y(), vecTgtPos_.z(),
    vecLosVec_.x(), vecLosVec_.y(), vecLosVec_.z());
    
  if (vecLosVec_.norm() < 1e-3)
  {
    RCLCPP_WARN(node_->get_logger(), "Los vector norm too small.");
    return;
  }

  dUasYawAng_ = dYawRadNED_;

  dLosYawAng_ = atan2(vecLosVec_.y(), vecLosVec_.x());
  dYawRate_ = yawRateCtrl_->dUpdateYawRate(dUasYawAng_, dLosYawAng_);
  dYawErr_ = yawRateCtrl_->dGetLastYawErr();
    
  dCuiVelZ_ = std::clamp(dKpAlt_ * (dTakeoffAlt_ + vecUasPos_.z()), -1 * cfgParam_.dAltVelLmt, cfgParam_.dAltVelLmt);
  // dCuiVelZ_ = std::clamp(dKpAlt_ * (vecTgtPos_.z() - vecUasPos_.z()), -1 * cfgParam_.dAltVelLmt, cfgParam_.dAltVelLmt);


  mavros_msgs::msg::PositionTarget posTgtMsg;
  posTgtMsg.header.stamp = node_->now();
  posTgtMsg.header.frame_id = "counter_uas_offboard_ctrl";
  posTgtMsg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

  MsgMavrosPositionTgt(posTgtMsg);

  posTgtMsg.velocity.x = 0.0;
  posTgtMsg.velocity.y = 0.0;
  posTgtMsg.velocity.z = dCuiVelZ_;
  posTgtMsg.yaw_rate = -dYawRate_;
  pubPosTgt_->publish(posTgtMsg);
  PubGimbalMode(GimbalMode::ALIGN);

  RCLCPP_INFO(node_->get_logger(),
              "Yaw Debug | UAS Yaw: %.2f deg | LOS Yaw: %.2f deg | YawErr: %.2f deg | YawRate: %.2f deg/s | GimbalAng: %2.f deg",
              dUasYawAng_ * R2D,
              dLosYawAng_ * R2D,
              dYawErr_ * R2D,
              dYawRate_ * R2D,
              dMountYawAng_ * R2D);
}

void CounterUasOffboardCtrlLib::MainGuidanceLoop()
{
  if (!bHasUasOdomResCallback_)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                          "Waiting for counter-uas odometry data, bHasUasOdomResCallback_: %d",
                          (int)(bHasUasOdomResCallback_));
    return;
  }

  // 상태 로그 출력
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000, 
                      "Mode: [%s], Armed: [%d]", 
                      currentState_.mode.c_str(), currentState_.armed);

  // 1. 지속적 keep-alive setpoint 발행
  PubKeepAliveSetPt();

  // 2. OFFBOARD 모드 요청 (최초 또는 주기적 재시도)
  if (currentState_.mode != "OFFBOARD" && (node_->now() - lastReq_).seconds() > 5.0)
  {
    ReqOffboardMode();
    return;
  }

  // 3. Arm 요청 (OFFBOARD 진입 후만)
  if (currentState_.mode == "OFFBOARD" && !currentState_.armed && (node_->now() - lastReq_).seconds() > 5.0)
  {
    ReqArming();
    return;
  }

  // 4. 진입 성공 플래그 업데이트
  if (currentState_.mode == "OFFBOARD" && currentState_.armed) 
  {
    bOffboardMode_ = true;
    bArmState_ = true;
  }

  switch (phase_)
  {
    case GuidancePhase::INIT:
      if (bOffboardMode_ && bArmState_)
      {
        phase_ = GuidancePhase::TAKEOFF;
      }
      break;
    
    case GuidancePhase::TAKEOFF:
      CmdTakeoff();
      if (abs(vecUasPos_.z() + dTakeoffAlt_) < 0.5)
      {
        phase_ = GuidancePhase::BOOST;
      }
      break;
    
    case GuidancePhase::BOOST:
      if (bHasTgtOdomResCallback_ && bHasMountAngResCallback_) 
      {
        CmdBoostPhase();
      }
      break;
  }
}

void CounterUasOffboardCtrlLib::convertQuaternionToEuler(double qx, double qy, double qz, double qw,
                                                        double& roll, double& pitch, double& yaw)
{
  // 1. tf2 Quaternion 생성
  tf2::Quaternion q(qx, qy, qz, qw);

  // 2. Matrix3x3으로 오일러 각 추출
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);  // RPY 순서: Roll(X), Pitch(Y), Yaw(Z)
}