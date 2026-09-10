cmake_minimum_required(VERSION 3.20)

set(${TARGET_NAME}_RequiredLibsPublic
    SMRobotWorkbenchCommon::WorkbenchCommon
    SMRobotWorkbenchMotionPlanning::MotionPlanningEditor
    SMRobotMotionPlanning::ProjectMotionPlanning
)

set(${TARGET_NAME}_RequiredLibsPrivate)
