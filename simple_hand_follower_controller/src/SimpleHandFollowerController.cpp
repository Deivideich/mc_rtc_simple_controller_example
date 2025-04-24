#include "SimpleHandFollowerController.h"

SimpleHandFollowerController::SimpleHandFollowerController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addTask(postureTask);
  solver().setContacts({{}});

  jointIndex = robot().jointIndexByName("NECK_Y");

  mc_rtc::log::success("SimpleHandFollowerController init done ");
}

bool SimpleHandFollowerController::run()
{
  if(std::abs(postureTask->posture()[jointIndex][0] - robot().mbc().q[jointIndex][0]) < 0.05)
  {
    switch_target();
  }
  return mc_control::MCController::run();
}
void SimpleHandFollowerController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
}

void SimpleHandFollowerController::switch_target()
{
  if(goingLeft)
  {
    postureTask->target({{"NECK_Y", robot().qu()[jointIndex]}});
  }
  else
  {
    postureTask->target({{"NECK_Y", robot().ql()[jointIndex]}});
  }
  goingLeft = !goingLeft;
}

CONTROLLER_CONSTRUCTOR("SimpleHandFollowerController", SimpleHandFollowerController)
