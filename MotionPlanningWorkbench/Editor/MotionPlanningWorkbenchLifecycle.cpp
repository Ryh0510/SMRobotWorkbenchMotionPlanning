#include "MotionPlanningWorkbenchLifecycle.h"

#include "MotionPlanningModuleController.h"
#include "MotionPlanningEditorWidget.h"

#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerDocumentViewRegistry.h"

#include <QFrame>
#include <QScrollArea>
#include <QSizePolicy>

namespace robot_qt_viewer
{
    RobotQtViewerWorkbenchRuntimeContributionFactoryDesc
    makeOwnedMotionPlanningWorkbenchRuntimeContributionFactory(
        MotionPlanningWorkbenchComposition composition)
    {
        const QString workbenchId = robotQtViewerWorkbenchId(
            RobotQtViewerWorkbenchKind::TrajectoryPlanning);
        return {
            workbenchId,
            [workbenchId, composition = std::move(composition)](QWidget* panelParent) {
                if(panelParent == nullptr || composition.documentContext == nullptr ||
                    composition.documentViewRegistry == nullptr) {
                    return std::unique_ptr<RobotQtViewerWorkbenchRuntimeContribution>();
                }
                auto* panel = new QScrollArea(panelParent);
                panel->setMinimumWidth(0);
                panel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
                panel->setWidgetResizable(true);
                panel->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
                panel->setFrameShape(QFrame::NoFrame);
                auto* widget = new MotionPlanningEditorWidget(panel);
                auto* controller = new MotionPlanningModuleController(
                    *widget,
                    *composition.documentContext,
                    panel);
                panel->setWidget(widget);
                QObject::connect(
                    controller,
                    &MotionPlanningModuleController::statusMessageRequested,
                    panel,
                    [showStatus = composition.showStatus](const QString& message, int timeoutMs) {
                        if(showStatus) {
                            showStatus(message, timeoutMs);
                        }
                    });
                RobotQtViewerDocumentViewRegistry* const registry =
                    composition.documentViewRegistry;
                registry->registerModule(
                    QStringLiteral("motionPlanning"),
                    controller,
                    [controller](const RobotQtViewerEvent& event) {
                        controller->handleEvent(event);
                    });
                auto language = std::make_unique<RobotQtViewerWidgetLanguageParticipant>();
                language->addRoot(*widget);
                return std::unique_ptr<RobotQtViewerWorkbenchRuntimeContribution>(
                    std::make_unique<RobotQtViewerBasicWorkbenchRuntimeContribution>(
                    workbenchId,
                    [panel]() { return panel; },
                    std::make_unique<MotionPlanningWorkbenchLifecycle>(*controller),
                    RobotQtViewerWorkbenchLifecyclePolicy{
                        RobotQtViewerWorkbenchExecutionPolicy::NoOwnedExecution,
                        RobotQtViewerWorkbenchReactivationPolicy::RestoreUiOnly },
                    std::move(language),
                    false,
                    RobotQtViewerWorkbenchRuntimeContribution::PanelTitleResolver{},
                    [registry, controller, panel]() {
                        registry->unregisterModule(controller);
                        delete panel;
                    }));
            }
        };
    }

    RobotQtViewerWorkbenchRuntimeContributionFactoryDesc
    makeMotionPlanningWorkbenchRuntimeContributionFactory(
        MotionPlanningModuleController& controller,
        QWidget& taskPanel,
        QObject& languageRoot)
    {
        const QString workbenchId = robotQtViewerWorkbenchId(
            RobotQtViewerWorkbenchKind::TrajectoryPlanning);
        return {
            workbenchId,
            [workbenchId, &controller, &taskPanel, &languageRoot](QWidget*) {
                auto language =
                    std::make_unique<RobotQtViewerWidgetLanguageParticipant>();
                language->addRoot(languageRoot);
                return std::make_unique<RobotQtViewerBasicWorkbenchRuntimeContribution>(
                    workbenchId,
                    [&taskPanel]() { return &taskPanel; },
                    std::make_unique<MotionPlanningWorkbenchLifecycle>(controller),
                    RobotQtViewerWorkbenchLifecyclePolicy{
                        RobotQtViewerWorkbenchExecutionPolicy::NoOwnedExecution,
                        RobotQtViewerWorkbenchReactivationPolicy::RestoreUiOnly },
                    std::move(language));
            }
        };
    }

    bool registerMotionPlanningWorkbenchContribution(
        RobotQtViewerWorkbenchPackageRegistry& catalog)
    {
        const QString packageId = QStringLiteral("smrobot.workbench.motion-planning");
        if(!catalog.registerPackage(makeRobotQtViewerWorkbenchPackage(
               packageId, QStringLiteral("Motion Planning"))) ||
            !catalog.registerWorkbench(makeRobotQtViewerWorkbench(
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
