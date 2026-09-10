#pragma once

#include "RobotQtViewerWorkbenchLifecycle.h"
#include "RobotQtViewerWorkbenchPackageRegistry.h"

namespace robot_qt_viewer
{
    class MotionPlanningModuleController;

    class MotionPlanningWorkbenchLifecycle final : public IRobotQtViewerWorkbenchLifecycle
    {
    public:
        explicit MotionPlanningWorkbenchLifecycle(
            MotionPlanningModuleController& controller);

        RobotQtViewerWorkbenchTransitionResult prepareDeactivate(
            const RobotQtViewerWorkbenchTransitionContext& context) override;
        RobotQtViewerWorkbenchTransitionResult deactivate(
            const RobotQtViewerWorkbenchTransitionContext& context) override;
        RobotQtViewerWorkbenchTransitionResult activate(
            const RobotQtViewerWorkbenchActivationContext& context) override;
        void releaseProject(
            const RobotQtViewerWorkbenchProjectReleaseContext& context) noexcept override;
        void shutdown(
            const RobotQtViewerWorkbenchShutdownContext& context) noexcept override;

    private:
        MotionPlanningModuleController& m_controller;
    };

    bool registerMotionPlanningWorkbenchContribution(
        RobotQtViewerWorkbenchPackageRegistry& catalog,
        RobotQtViewerWorkbenchPackageSource source);
}
