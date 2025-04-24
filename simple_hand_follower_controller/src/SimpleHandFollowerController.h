#pragma once

#include <mc_control/mc_controller.h>

#include "api.h"

struct SimpleHandFollowerController_DLLAPI SimpleHandFollowerController : public mc_control::MCController 
{
  SimpleHandFollowerController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config);

  bool run() override;

  void reset(const mc_control::ControllerResetData &reset_data) override;

  void switch_target();
private:
  mc_rtc::Configuration config_;
  std::string jointName_ = "NECK_Y";
  int jointIndex = 0;
  bool goingLeft = true;
  
};