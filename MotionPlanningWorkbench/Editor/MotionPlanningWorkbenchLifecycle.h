#pragma once

#include "RobotQtViewerWorkbenchLifecycle.h"
#include "RobotQtViewerWorkbenchContribution.h"
#include "RobotQtViewerWorkbenchPackageRegistry.h"

#include <functional>

class QObject;
class QWidget;

namespace robot_qt_viewer
{
    class RobotQtViewerDocumentContext;
    class RobotQtViewerDocumentViewRegistry;
    class MotionPlanningModuleController;

    struct MotionPlanningWorkbenchComposition
    {
        RobotQtViewerDocumentContext* documentContext = nullptr;
        RobotQtViewerDocumentViewRegistry* documentViewRegistry = nullptr;
        std::function<void(const QString&, int)> showStatus;
    };

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
        RobotQtViewerWorkbenchPackageRegistry& catalog);
    RobotQtViewerWorkbenchRuntimeContributionFactoryDesc
        makeMotionPlanningWorkbenchRuntimeContributionFactory(
            MotionPlanningModuleController& controller,
            QWidget& taskPanel,
            QObject& languageRoot);
    RobotQtViewerWorkbenchRuntimeContributionFactoryDesc
        makeOwnedMotionPlanningWorkbenchRuntimeContributionFactory(
            MotionPlanningWorkbenchComposition composition);
}
