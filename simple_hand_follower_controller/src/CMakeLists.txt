set(controller_SRC
  SimpleHandFollowerController.cpp
)

set(controller_HDR
  SimpleHandFollowerController.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSimpleHandFollowerController_EXPORTS")

