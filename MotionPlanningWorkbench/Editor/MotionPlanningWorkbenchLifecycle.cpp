#include "MotionPlanningWorkbenchLifecycle.h"

#include "MotionPlanningModuleController.h"

namespace robot_qt_viewer
{
    bool registerMotionPlanningWorkbenchContribution(
        RobotQtViewerWorkbenchPackageRegistry& catalog,
        RobotQtViewerWorkbenchPackageSource source)
    {
        const QString packageId = QStringLiteral("smrobot.workbench.motion-planning");
        if(!catalog.registerPackage(makeRobotQtViewerWorkbenchPackage(
               packageId, QStringLiteral("Motion Planning"), source)) ||
            !catalog.registerMode(makeRobotQtViewerWorkbenchMode(
               packageId,
               RobotQtViewerWorkbenchKind::TrajectoryPlanning,
               QStringLiteral("motionPlanningWorkbench"),
               50,
               { QStringLiteral("smrobot.feature.motion-planning") },
               { robotQtViewerWorkbenchId(RobotQtViewerWorkbenchKind::Browse) }))) {
            return false;
        }
        return catalog.registerFeature(makeRobotQtViewerWorkbenchFeature(
            QStringLiteral("smrobot.feature.motion-planning"),
            QStringLiteral("Motion Planning"),
            packageId,
            { robotQtViewerWorkbenchId(RobotQtViewerWorkbenchKind::TrajectoryPlanning) }));
    }

    MotionPlanningWorkbenchLifecycle::MotionPlanningWorkbenchLifecycle(
        MotionPlanningModuleController& controller)
        : m_controller(controller)
    {
    }

    RobotQtViewerWorkbenchTransitionResult
    MotionPlanningWorkbenchLifecycle::prepareDeactivate(
        const RobotQtViewerWorkbenchTransitionContext&)
    {
        return workbenchTransitionSucceeded();
    }

    RobotQtViewerWorkbenchTransitionResult MotionPlanningWorkbenchLifecycle::deactivate(
        const RobotQtViewerWorkbenchTransitionContext&)
    {
        return workbenchTransitionSucceeded();
    }

    RobotQtViewerWorkbenchTransitionResult MotionPlanningWorkbenchLifecycle::activate(
        const RobotQtViewerWorkbenchActivationContext&)
    {
        return workbenchTransitionSucceeded();
    }

    void MotionPlanningWorkbenchLifecycle::releaseProject(
        const RobotQtViewerWorkbenchProjectReleaseContext&) noexcept
    {
    }

    void MotionPlanningWorkbenchLifecycle::shutdown(
        const RobotQtViewerWorkbenchShutdownContext&) noexcept
    {
    }
}
