#include "SimpleHandFollowerController.h"

SimpleHandFollowerController::SimpleHandFollowerController(
    mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : mc_control::MCController(rm, dt),
      movementDone(false),
      previousMovementDone(false),
      state(HandState::RaiseLeft)
{
  config_.load(config);
  createTasks();

  mc_rtc::log::info(config_.dump(true));

  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addTask(postureTask);
  solver().addTask(eflTask);
  solver().addTask(efrTask);
  solver().addTask(comTask);
  solver().addTask(lookAtTask);

  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});

  postureTask->stiffness(1);

  left_hand_target = {{0.5, 0.25, 1.1}, {0.7, 0.0, 0.7, 0.0}};
  right_hand_target = {{0.5, -0.25, 1.1}, {0.7, 0.0, 0.7, 0.0}};

  mc_rtc::log::success("SimpleHandFollowerController init done");
}

void SimpleHandFollowerController::createTasks()
{
  comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0, 10.0, 1000.0);
  eflTask = std::make_shared<mc_tasks::EndEffectorTask>("l_wrist", robots(), 0, 5.0, 500.0);
  efrTask = std::make_shared<mc_tasks::EndEffectorTask>("r_wrist", robots(), 0, 5.0, 500.0);

  auto &headFrame = robot().frame("NECK_P_S");
  lookAtTask = std::make_shared<mc_tasks::LookAtTask>(headFrame, Eigen::Vector3d{1.0, 0.0, 0.0}, 1.0, 500.0);
  lookAtTask->stiffness(10.0);
  lookAtTask->damping(10.0);

}

bool SimpleHandFollowerController::run()
{
  bool isLeft = false, isRight = false, isBoth = false;

  switch (state)
  {
    case HandState::RaiseLeft:
      isLeft = true;
      move_hand(left_hand_target, true);
      break;
    case HandState::LowerLeft:
      isLeft = true;
      move_hand(left_hand_initial, true);
      break;
    case HandState::RaiseRight:
      isRight = true;
      move_hand(right_hand_target, false);
      break;
    case HandState::LowerRight:
      isRight = true;
      move_hand(right_hand_initial, false);
      break;
    case HandState::RaiseBoth:
      isBoth = true;
      move_hand(left_hand_target, true);
      move_hand(right_hand_target, false);
      break;
    case HandState::LowerBoth:
      isBoth = true;
      move_hand(left_hand_initial, true);
      move_hand(right_hand_initial, false);
      break;
  }

  if (isBoth)
  {
    movementDone = eflTask->eval().norm() < movement_tolerance && efrTask->eval().norm() < movement_tolerance;
  }
  else if (isLeft)
  {
    movementDone = eflTask->eval().norm() < movement_tolerance;
  }
  else if (isRight)
  {
    movementDone = efrTask->eval().norm() < movement_tolerance;
  }

  if (movementDone && !previousMovementDone)
  {
    state = static_cast<HandState>((static_cast<int>(state) + 1) % 6);
  }
  previousMovementDone = movementDone;

  if (isBoth)
  {
    Eigen::Vector3d forward = robot().frame("NECK_Y_S").position().translation() + Eigen::Vector3d{1.0, 0.0, 0.0};
    lookAtTask->target(forward);
  }
  else if (isLeft)
  {
    lookAtTask->target(robot().frame("l_wrist").position().translation());
  }
  else if (isRight)
  {
    lookAtTask->target(robot().frame("r_wrist").position().translation());
  }

  return mc_control::MCController::run();
}

void SimpleHandFollowerController::reset(const mc_control::ControllerResetData &reset_data)
{
  mc_control::MCController::reset(reset_data);
  eflTask->reset();
  efrTask->reset();
  comTask->reset();
  comTask->com(Eigen::Vector3d{0, 0, 1.0});

  const auto &lwrist = robot().frame("l_wrist").position();
  left_hand_initial.position = lwrist.translation();
  left_hand_initial.orientation = Eigen::Quaterniond(lwrist.rotation()).normalized();

  const auto &rwrist = robot().frame("r_wrist").position();
  right_hand_initial.position = rwrist.translation();
  right_hand_initial.orientation = Eigen::Quaterniond(rwrist.rotation()).normalized();
}

void SimpleHandFollowerController::move_hand(const HandPose &pose, bool isLeft)
{
  Eigen::Matrix3d rot = pose.orientation.normalized().toRotationMatrix();
  sva::PTransformd pt(rot, pose.position);
  if (isLeft)
    eflTask->set_ef_pose(pt);
  else
    efrTask->set_ef_pose(pt);
}

CONTROLLER_CONSTRUCTOR("SimpleHandFollowerController", SimpleHandFollowerController)
