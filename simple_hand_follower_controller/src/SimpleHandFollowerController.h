#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/MetaTaskLoader.h>

#include "api.h"

struct SimpleHandFollowerController_DLLAPI SimpleHandFollowerController : public mc_control::MCController 
{
  SimpleHandFollowerController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config);

  bool run() override;

  void reset(const mc_control::ControllerResetData &reset_data) override;

  void switch_target();

  void switch_com_target();

  void move_hand(Eigen::Vector3d, Eigen::Quaterniond, bool);

  void return_left_hand();

  void head_look_at_target(mc_rbdyn::Robot &robot, const Eigen::Vector3d &target);

private:
  mc_rtc::Configuration config_;
  
  int jointIndex = 0;
  int state = 0;
  int movementDoneCounter = 0;

  bool goingLeft = true;
  bool handAtTarget = true;
  bool comDown = false;
  bool isLeftHand = false;
  bool isRightHand = false;
  bool bothHands = false;
  bool leftHandAtTarget = true;
  bool rightHandAtTarget = true;
  bool leftWasAtTarget = false;
  bool rightWasAtTarget = false;
  bool bothWereAtTarget = false;
  bool leftDone = false;
  bool rightDone = false;
  bool movementDone = false;
  bool previousMovementDone = false;

  std::shared_ptr<mc_tasks::CoMTask> comTask;
  std::shared_ptr<mc_tasks::EndEffectorTask> efrTask;
  std::shared_ptr<mc_tasks::EndEffectorTask> eflTask;
  std::shared_ptr<mc_tasks::OrientationTask> headTask;

  Eigen::Vector3d comZero;
  Eigen::Vector3d left_hand_initial_position;
  Eigen::Vector3d left_hand_target_position;
  Eigen::Vector3d right_hand_initial_position;
  Eigen::Vector3d right_hand_target_position;
  Eigen::Quaterniond left_hand_initial_quaternion;
  Eigen::Quaterniond left_hand_target_quaternion;
  Eigen::Quaterniond right_hand_initial_quaternion;
  Eigen::Quaterniond right_hand_target_quaternion;
};