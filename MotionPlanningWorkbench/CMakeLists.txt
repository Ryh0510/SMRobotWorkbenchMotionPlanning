cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME MotionPlanningWorkbench)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_subdirectory(Editor)

add_library(${TARGET_NAME} INTERFACE)
add_library(SMRobotWorkbenchMotionPlanning::${TARGET_NAME} ALIAS ${TARGET_NAME})

target_link_libraries(${TARGET_NAME}
    INTERFACE
        SMRobotWorkbenchCommon::WorkbenchCommon
        $<BUILD_INTERFACE:MotionPlanningEditor>
        $<INSTALL_INTERFACE:SMRobotWorkbenchMotionPlanning::MotionPlanningEditor>
)

target_compile_features(${TARGET_NAME}
    INTERFACE
        cxx_std_17
)

set(${TARGET_NAME}_INSTALL_PUBLIC_DEPENDENCIES
    SMRobotWorkbenchCommon::WorkbenchCommon
    SMRobotWorkbenchMotionPlanning::MotionPlanningEditor
    SMRobotMotionPlanning::ProjectMotionPlanning
)

set_target_properties(${TARGET_NAME}
    PROPERTIES
        FOLDER SMRobotWorkbenchMotionPlanning
        SMROBOT_WORKBENCH_REGISTRAR_HEADER "MotionPlanningWorkbenchLifecycle.h"
        SMROBOT_WORKBENCH_REGISTRAR_FUNCTION "robot_qt_viewer::registerMotionPlanningWorkbenchContribution"
        EXPORT_PROPERTIES "SMROBOT_WORKBENCH_REGISTRAR_HEADER;SMROBOT_WORKBENCH_REGISTRAR_FUNCTION"
)

set(${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR ${CMAKE_CURRENT_BINARY_DIR}/install_headers)
file(MAKE_DIRECTORY ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR})

function_InstallTarget(
    TARGET_NAME ${TARGET_NAME}
    PACKAGE_NAME ${PACKAGE_NAME}
    PACKAGE_INTERFACE_HEADERS ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR}
)
