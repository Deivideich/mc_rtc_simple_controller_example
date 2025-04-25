#include "SimpleHandFollowerController.h"

SimpleHandFollowerController::SimpleHandFollowerController(
    mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : mc_control::MCController(rm, dt) {
  config_.load(config);
  comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0, 10.0, 1000.0);
  eflTask = std::make_shared<mc_tasks::EndEffectorTask>("l_wrist", robots(), 0,
                                                        5.0, 500.0);
  efrTask = std::make_shared<mc_tasks::EndEffectorTask>("r_wrist", robots(), 0,
                                                        5.0, 500.0);

  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addTask(postureTask);
  solver().addTask(eflTask);
  solver().addTask(efrTask);

  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});

  solver().addTask(comTask);
  postureTask->stiffness(1);
  jointIndex = robot().jointIndexByName("NECK_Y");

  left_hand_target_position = Eigen::Vector3d(0.5, 0.25, 1.1); // Translation
  left_hand_target_quaternion =
      Eigen::Quaterniond(0.7071, 0.0, 0.7071, 0.0); // 90 deg around Y

  right_hand_target_position = Eigen::Vector3d(0.5, -0.25, 1.1);
  right_hand_target_quaternion =
      Eigen::Quaterniond(0.7071, 0.0, 0.7071, 0.0); // 90 deg around Y

  mc_rtc::log::success("SimpleHandFollowerController init done ");
  isLeftHand = false;
  isRightHand = false;
  bothHands = false;

  state = 0;
}
bool SimpleHandFollowerController::run() {

  switch (state) {
  case 0:
    isLeftHand = true;
    isRightHand = false;
    bothHands = false;
    break;
  case 1:
    isLeftHand = false;
    isRightHand = true;
    bothHands = false;
    break;
  case 2:
    isLeftHand = false;
    isRightHand = false;
    bothHands = true;
    break;
  }

  if (bothHands) {
    // Always move the hands based on current state
    if (leftHandAtTarget) {
      move_hand(left_hand_target_position, left_hand_target_quaternion, true);
    } else {
      move_hand(left_hand_initial_position, left_hand_initial_quaternion, true);
    }

    if (rightHandAtTarget) {
      move_hand(right_hand_target_position, right_hand_target_quaternion,
                false);
    } else {
      move_hand(right_hand_initial_position, right_hand_initial_quaternion,
                false);
    }

    // Evaluate both hand errors
    double left_error = eflTask->eval().norm();
    double right_error = efrTask->eval().norm();

    // Only toggle when both hands have reached the target
    if (left_error < 0.05 && right_error < 0.05 && !bothWereAtTarget) {
      leftHandAtTarget = !leftHandAtTarget;
      rightHandAtTarget = !rightHandAtTarget;
      bothWereAtTarget = true;
    } else if (left_error >= 0.05 || right_error >= 0.05) {
      bothWereAtTarget = false;
    }
  }

  else {
    if (isRightHand) {
      if (rightHandAtTarget) {
        move_hand(right_hand_target_position, right_hand_target_quaternion,
                  false);
      } else {
        move_hand(right_hand_initial_position, right_hand_initial_quaternion,
                  false);
      }
      // Check if the hand is close to the current target
      sva::PTransformd current = robot().frame("r_wrist").position();
      Eigen::Vector3d current_pos = current.translation();
      double dist =
          (current_pos - (rightHandAtTarget ? right_hand_target_position
                                            : right_hand_initial_position))
              .norm();

      if (dist < 0.01) {                        // You can tune this threshold
        rightHandAtTarget = !rightHandAtTarget; // Toggle
      }
    }
    // Move the left hand
    if (isLeftHand) {
      if (leftHandAtTarget) {
        move_hand(left_hand_target_position, left_hand_target_quaternion, true);
      } else {
        move_hand(left_hand_initial_position, left_hand_initial_quaternion,
                  true);
      }
      // Check if the hand is close to the current target
      sva::PTransformd current = robot().frame("l_wrist").position();
      Eigen::Vector3d current_pos = current.translation();
      double dist =
          (current_pos - (leftHandAtTarget ? left_hand_target_position
                                           : left_hand_initial_position))
              .norm();
      if (dist < 0.01) {                      // You can tune this threshold
        leftHandAtTarget = !leftHandAtTarget; // Toggle
      }
    }
  }

  // Default inactive hands as "done"
  bool leftDone = (!isLeftHand && !bothHands) ? true : false;
  bool rightDone = (!isRightHand && !bothHands) ? true : false;


  // Check actual hand movement completion
  if (isLeftHand) {
    sva::PTransformd current = robot().frame("l_wrist").position();
    Eigen::Vector3d current_pos = current.translation();
    double dist =
        (current_pos - (leftHandAtTarget ? left_hand_target_position
                                         : left_hand_initial_position))
            .norm();
    leftDone = (dist < 0.01);
  }

  if (isRightHand) {
    sva::PTransformd current = robot().frame("r_wrist").position();
    Eigen::Vector3d current_pos = current.translation();
    double dist =
        (current_pos - (rightHandAtTarget ? right_hand_target_position
                                          : right_hand_initial_position))
            .norm();
    rightDone = (dist < 0.01);
  }

  if (bothHands) {
    double left_error = eflTask->eval().norm();
    double right_error = efrTask->eval().norm();
    leftDone = (left_error < 0.05);
    rightDone = (right_error < 0.05);
  }

  // Advance to next state only if motion is done
  if (leftDone && rightDone) {
    state++;
    if (state > 2) {
      state = 0;
    }
    mc_rtc::log::info("Switching to state: {}", state);
  }

  // Switch the target for the neck
  if (std::abs(postureTask->posture()[jointIndex][0] -
               robot().mbc().q[jointIndex][0]) < 0.05) {
    switch_target();
  }

  return mc_control::MCController::run();
}

void SimpleHandFollowerController::reset(
    const mc_control::ControllerResetData &reset_data) {
  mc_control::MCController::reset(reset_data);
  eflTask->reset();
  efrTask->reset();
  comTask->reset();
  comZero = comTask->com();

  const auto &X_0_lwrist = robot().frame("l_wrist").position();
  left_hand_initial_position = X_0_lwrist.translation();
  left_hand_initial_quaternion = Eigen::Quaterniond(X_0_lwrist.rotation());
  left_hand_initial_quaternion.normalize();

  const auto &X_0_rwrist = robot().frame("r_wrist").position();
  right_hand_initial_position = X_0_rwrist.translation();
  right_hand_initial_quaternion = Eigen::Quaterniond(X_0_rwrist.rotation());
  right_hand_initial_quaternion.normalize();
}

void SimpleHandFollowerController::switch_com_target() {
  // comZero is obtained by doing:
  // comZero = comTask->com();
  // in the reset function
  if (comDown) {
    comTask->com(comZero - Eigen::Vector3d{0, 0, 0.2});
  } else {
    comTask->com(comZero);
  }
  comDown = !comDown;
}

void SimpleHandFollowerController::move_hand(
    Eigen::Vector3d left_hand_target_position,
    Eigen::Quaterniond left_hand_target_quaternion, bool left_hand) {
  // Normalize just in case
  left_hand_target_quaternion.normalize();

  // Convert quaternion to rotation matrix
  Eigen::Matrix3d rot = left_hand_target_quaternion.toRotationMatrix();

  // Create transform
  sva::PTransformd pose(rot, left_hand_target_position);

  // Set the pose
  if (left_hand) {
    eflTask->set_ef_pose(pose);
  } else {
    efrTask->set_ef_pose(pose);
  }
}

void SimpleHandFollowerController::switch_target() {
  if (goingLeft) {
    postureTask->target({{"NECK_Y", robot().qu()[jointIndex]}});
  } else {
    postureTask->target({{"NECK_Y", robot().ql()[jointIndex]}});
  }
  goingLeft = !goingLeft;
}

CONTROLLER_CONSTRUCTOR("SimpleHandFollowerController",
                       SimpleHandFollowerController)
