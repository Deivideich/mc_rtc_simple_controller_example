#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/LookAtTask.h>
#include <mc_tasks/MetaTaskLoader.h>

#include "api.h"

enum class HandState
{
  RaiseLeft,
  LowerLeft,
  RaiseRight,
  LowerRight,
  RaiseBoth,
  LowerBoth
};

struct HandPose
{
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

struct SimpleHandFollowerController_DLLAPI SimpleHandFollowerController : public mc_control::MCController
{
  SimpleHandFollowerController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config);

  bool run() override;
  void reset(const mc_control::ControllerResetData &reset_data) override;

private:
  void createTasks();
  void move_hand(const HandPose &pose, bool isLeft);

private:
  mc_rtc::Configuration config_;

  HandState state = HandState::RaiseLeft;
  bool previousMovementDone = false;
  bool movementDone = false;
  float movement_tolerance = 0.1;

  HandPose left_hand_initial, left_hand_target;
  HandPose right_hand_initial, right_hand_target;

  std::shared_ptr<mc_tasks::CoMTask> comTask;
  std::shared_ptr<mc_tasks::EndEffectorTask> eflTask;
  std::shared_ptr<mc_tasks::EndEffectorTask> efrTask;
  std::shared_ptr<mc_tasks::LookAtTask> lookAtTask;

  Eigen::Vector3d comZero;
};
