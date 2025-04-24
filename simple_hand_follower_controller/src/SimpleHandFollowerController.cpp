#include "SimpleHandFollowerController.h"

SimpleHandFollowerController::SimpleHandFollowerController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addTask(postureTask);
  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});
  comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0, 10.0, 1000.0);
  solver().addTask(comTask);
  postureTask->stiffness(1);
  jointIndex = robot().jointIndexByName("NECK_Y");
  mc_rtc::log::success("SimpleHandFollowerController init done ");
}

bool SimpleHandFollowerController::run()
{
  if(comTask->eval().norm() < 0.01)
  {
    switch_com_target();
  }

  if(std::abs(postureTask->posture()[jointIndex][0] - robot().mbc().q[jointIndex][0]) < 0.05)
  {
    switch_target();
  }
  
  return mc_control::MCController::run();
}

void SimpleHandFollowerController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
  comTask->reset();
  comZero = comTask->com();
}

void SimpleHandFollowerController::switch_com_target()
{
  // comZero is obtained by doing:
  // comZero = comTask->com();
  // in the reset function
  if(comDown) { comTask->com(comZero - Eigen::Vector3d{0, 0, 0.2}); }
  else { comTask->com(comZero); }
  comDown = !comDown;
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
