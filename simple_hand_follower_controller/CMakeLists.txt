cmake_minimum_required(VERSION 3.10)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME SimpleHandFollowerController)
set(PROJECT_DESCRIPTION "SimpleHandFollowerController")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/SimpleHandFollowerController.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/SimpleHandFollowerController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/SimpleHandFollowerController.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

