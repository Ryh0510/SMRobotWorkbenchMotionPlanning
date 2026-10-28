cmake_minimum_required(VERSION 3.20)

message(STATUS "!!!!!!Package ${PACKAGE_NAME} configuration is set here!!!!!!")

set(${PACKAGE_NAME}_PublicComponents
    MotionPlanningWorkbench
    MotionPlanningEditor
)

set(${PACKAGE_NAME}_CompatibilityComponents)
set(${PACKAGE_NAME}_ExportComponents
    ${${PACKAGE_NAME}_PublicComponents}
    ${${PACKAGE_NAME}_CompatibilityComponents}
)

set(${PACKAGE_NAME}_Components
    MotionPlanningWorkbench
)

rs_begin_package_requirements(${PACKAGE_NAME})
foreach(TARGET_NAME ${${PACKAGE_NAME}_Components})
    include(${CMAKE_CURRENT_LIST_DIR}/${TARGET_NAME}/TargetConfigSetting.cmake)
    rs_collect_component_requirements(${PACKAGE_NAME} ${TARGET_NAME})
    set(${PACKAGE_NAME}${TARGET_NAME}_RequiredLibsPublic ${${TARGET_NAME}_RequiredLibsPublic})
    set(${PACKAGE_NAME}${TARGET_NAME}_RequiredLibsPrivate ${${TARGET_NAME}_RequiredLibsPrivate})
endforeach()

list(REMOVE_DUPLICATES ${PACKAGE_NAME}_RequiredLibsPublic)
list(REMOVE_DUPLICATES ${PACKAGE_NAME}_RequiredLibsPrivate)
list(APPEND ${PACKAGE_NAME}_RequiredLibs ${${PACKAGE_NAME}_RequiredLibsPublic} ${${PACKAGE_NAME}_RequiredLibsPrivate})
list(REMOVE_DUPLICATES ${PACKAGE_NAME}_RequiredLibs)
resolve_dependencies(${PACKAGE_NAME}_RequiredLibs)
