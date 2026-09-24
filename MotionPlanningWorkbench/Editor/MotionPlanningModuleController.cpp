#include "MotionPlanningModuleController.h"

#include "MotionPlanningEditorWidget.h"
#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerDocumentController.h"
#include "RobotQtViewerSelectionModel.h"
#include "RobotQtViewerViewportServices.h"
#include "RobotQtViewerViewportPreviewState.h"

#include <RobotQtViewerFileDialog.h>
#include <MotionPlanningCore/MotionPlanning.h>
#include <ProjectMotionPlanning/ProjectMotionPlanning.h>
#include <ProjectMotionPlanning/CdfJointAngleImport.h>
#include <ProjectMotionPlanning/CdfQpTrajectoryRepair.h>
#include <ProjectMotionPlanning/TrajectoryControlPointEditing.h>
#include <ProjectMotionPlanning/TrajectoryImport.h>
#include <ProjectMotionPlanning/TrajectoryInverseKinematics.h>
#include <SimulationProject/ProjectDocumentService.h>
#include <SimulationProject/RuntimePaths.h>

#include <QFileDialog>
#include <QLocale>
#include <QSaveFile>
#include <QStringList>
#include <QTimer>
#include <QThread>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace
{
    constexpr double kPi = 3.14159265358979323846;

    bool parseJointVector(
        const QString& text,
        std::vector<double>& values,
        QString& error)
    {
        values.clear();
        const QStringList parts = text.split(',', Qt::SkipEmptyParts);
        for(const QString& part : parts) {
            bool ok = false;
            const double value = part.trimmed().toDouble(&ok);
            if(!ok) {
                error = QStringLiteral("Invalid joint value: %1").arg(part.trimmed());
                return false;
            }
            values.push_back(value);
        }
        if(values.empty()) {
            error = QStringLiteral("Enter at least one joint value.");
            return false;
        }
        return true;
    }

    std::vector<std::string> selectedRobotJointNames(
        const simulation_project::ProjectDocument& document,
        const QString& robotId)
    {
        const auto robotIt = std::find_if(
            document.robots.begin(),
            document.robots.end(),
            [&](const simulation_project::RobotDesc& robot) {
                return robot.id == robotId.toStdString();
            });
        if(robotIt == document.robots.end()) {
            return {};
        }

        std::vector<std::string> names;
        names.reserve(robotIt->initialJoints.size());
        for(const simulation_project::JointValueDesc& joint : robotIt->initialJoints) {
            names.push_back(joint.jointName);
        }
        if(names.empty() &&
            (robotIt->sourcePath.find("ABB4600") != std::string::npos ||
             robotIt->sourcePath.find("4600") != std::string::npos ||
             robotIt->id.find("4600") != std::string::npos)) {
            names = motion_planning::ProjectTrajectoryInverseKinematics::defaultIrb4600JointNames();
        }
        return names;
    }

    QString defaultCdfPlanningRobotId(
        const simulation_project::ProjectDocument& document,
        const QString& selectedRobotId)
    {
        const auto exactIt = std::find_if(
            document.robots.begin(),
            document.robots.end(),
            [](const simulation_project::RobotDesc& robot) {
                return robot.id == "ABB4600_urdf";
            });
        if(exactIt != document.robots.end()) {
            return QString::fromStdString(exactIt->id);
        }

        const auto sourceIt = std::find_if(
            document.robots.begin(),
            document.robots.end(),
            [](const simulation_project::RobotDesc& robot) {
                return robot.id.find("4600") != std::string::npos ||
                    robot.name.find("4600") != std::string::npos ||
                    robot.sourcePath.find("ABB4600") != std::string::npos;
            });
        if(sourceIt != document.robots.end()) {
            return QString::fromStdString(sourceIt->id);
        }

        return selectedRobotId;
    }

    bool detectorTargetsRobot(
        const simulation_project::CollisionDetectorDesc& detector,
        const std::string& robotId)
    {
        return std::any_of(
            detector.targets.begin(),
            detector.targets.end(),
            [&](const simulation_project::CollisionDetectorTargetDesc& target) {
                return target.robotId == robotId;
            });
    }

    bool detectorHasCdfPairGenerator(
        const simulation_project::CollisionDetectorDesc& detector,
        const std::string& robotId,
        const std::string& obstacleId)
    {
        return std::any_of(
            detector.pairGenerators.begin(),
            detector.pairGenerators.end(),
            [&](const simulation_project::CollisionPairGeneratorDesc& generator) {
                if(generator.type == "RobotRobot") {
                    return (generator.robotA == robotId && generator.robotB == obstacleId) ||
                        (generator.robotA == obstacleId && generator.robotB == robotId);
                }
                if(generator.type == "RobotObject") {
                    return generator.robotId == robotId && generator.objectId == obstacleId;
                }
                return false;
            });
    }

    bool cdfDetectorIsConfigured(
        const simulation_project::ProjectDocument& document,
        const std::string& robotId,
        const motion_planning::ProjectCdfQpRepairOptions& options)
    {
        const auto detectorIt = std::find_if(
            document.collision.detectors.begin(),
            document.collision.detectors.end(),
            [&](const simulation_project::CollisionDetectorDesc& detector) {
                return detector.id == options.detectorId;
            });
        if(detectorIt == document.collision.detectors.end()) {
            return false;
        }

        return detectorIt->enabled &&
            detectorIt->distance &&
            detectorIt->nearestPoints &&
            detectorTargetsRobot(*detectorIt, robotId) &&
            detectorTargetsRobot(*detectorIt, options.obstacleId) &&
            detectorHasCdfPairGenerator(*detectorIt, robotId, options.obstacleId);
    }

    std::vector<std::string> playbackCollisionDetectorIds(
        const simulation_project::ProjectDocument& document,
        const QString& robotId)
    {
        const motion_planning::ProjectCdfQpRepairOptions cdfOptions;
        const std::string robotIdText = robotId.toStdString();
        if(cdfDetectorIsConfigured(document, robotIdText, cdfOptions)) {
            return { cdfOptions.detectorId };
        }

        const auto cdfDetectorIt = std::find_if(
            document.collision.detectors.begin(),
            document.collision.detectors.end(),
            [&](const simulation_project::CollisionDetectorDesc& detector) {
                return detector.id == cdfOptions.detectorId;
            });
        if(cdfDetectorIt != document.collision.detectors.end()) {
            return {};
        }

        std::vector<std::string> detectorIds;
        detectorIds.reserve(document.collision.detectors.size());

        auto detectorUsesRobot = [&](const simulation_project::CollisionDetectorDesc& detector) {
            for(const simulation_project::CollisionDetectorTargetDesc& target : detector.targets) {
                if(target.robotId == robotIdText) {
                    return true;
                }
            }
            for(const simulation_project::CollisionPairGeneratorDesc& generator : detector.pairGenerators) {
                if(generator.robotId == robotIdText ||
                    generator.robotA == robotIdText ||
                    generator.robotB == robotIdText) {
                    return true;
                }
            }
            return false;
        };

        for(const simulation_project::CollisionDetectorDesc& detector : document.collision.detectors) {
            if(detector.enabled && detectorUsesRobot(detector)) {
                detectorIds.push_back(detector.id);
            }
        }

        if(detectorIds.empty()) {
            for(const simulation_project::CollisionDetectorDesc& detector : document.collision.detectors) {
                if(detector.enabled) {
                    detectorIds.push_back(detector.id);
                }
            }
        }

        return detectorIds;
    }

    bool usesIrb4600RobotSystemJointSigns(
        const simulation_project::ProjectDocument& document,
        const QString& robotId)
    {
        const auto robotIt = std::find_if(
            document.robots.begin(),
            document.robots.end(),
            [&](const simulation_project::RobotDesc& robot) {
                return robot.id == robotId.toStdString();
            });
        if(robotIt == document.robots.end()) {
            return false;
        }
        return robotIt->sourcePath.find("ABB4600") != std::string::npos ||
            robotIt->sourcePath.find("4600") != std::string::npos ||
            robotIt->id.find("4600") != std::string::npos;
    }

    QString formatDouble(double value)
    {
        return QString::number(value, 'g', 8);
    }

    QString formatJointValues(
        const std::vector<std::string>& jointNames,
        const std::vector<double>& values)
    {
        QStringList parts;
        for(std::size_t index = 0; index < values.size(); ++index) {
            const QString name = index < jointNames.size()
                ? QString::fromStdString(jointNames[index])
                : QStringLiteral("q%1").arg(static_cast<qulonglong>(index + 1));
            parts.push_back(QStringLiteral("%1=%2").arg(name, formatDouble(values[index])));
        }
        return parts.join(QStringLiteral(", "));
    }

    std::vector<double> degreesToRadians(const std::vector<double>& degrees)
    {
        std::vector<double> radians;
        radians.reserve(degrees.size());
        for(const double value : degrees) {
            radians.push_back(value * kPi / 180.0);
        }
        return radians;
    }

    std::vector<double> radiansToDegrees(const std::vector<double>& radians)
    {
        std::vector<double> degrees;
        degrees.reserve(radians.size());
        for(const double value : radians) {
            degrees.push_back(value * 180.0 / kPi);
        }
        return degrees;
    }

    QString formatCartesianPose(const robottrajectory::TimedCartesianPoint& point)
    {
        const Eigen::Vector3d translation = point.tcpPose.translation();
        return QStringLiteral("x=%1, y=%2, z=%3")
            .arg(formatDouble(translation.x()))
            .arg(formatDouble(translation.y()))
            .arg(formatDouble(translation.z()));
    }

    QString formatCartesianEuler(const robottrajectory::TimedCartesianPoint& point)
    {
        const Eigen::Vector3d euler = point.tcpPose.linear().eulerAngles(2, 1, 0);
        const double yawDeg = euler[0] * 180.0 / kPi;
        const double pitchDeg = euler[1] * 180.0 / kPi;
        const double rollDeg = euler[2] * 180.0 / kPi;
        return QStringLiteral("roll=%1, pitch=%2, yaw=%3")
            .arg(formatDouble(rollDeg))
            .arg(formatDouble(pitchDeg))
            .arg(formatDouble(yawDeg));
    }

    simulation_project::TransformDesc transformDescFromPose(const Eigen::Isometry3d& pose)
    {
        simulation_project::TransformDesc transform;
        transform.x = pose.translation().x();
        transform.y = pose.translation().y();
        transform.z = pose.translation().z();
        const Eigen::Vector3d euler = pose.linear().eulerAngles(2, 1, 0);
        transform.yaw = euler[0];
        transform.pitch = euler[1];
        transform.roll = euler[2];
        return transform;
    }

    std::vector<simulation_project::TransformDesc> cartesianControlPointTransforms(
        const motion_planning::StoredMotionPlan& plan)
    {
        std::vector<simulation_project::TransformDesc> transforms;
        transforms.reserve(plan.cartesianControlPoints.points.size());
        for(const robottrajectory::TimedCartesianPoint& point : plan.cartesianControlPoints.points) {
            transforms.push_back(transformDescFromPose(point.tcpPose));
        }
        return transforms;
    }

    const motion_planning::StoredMotionPlan* findMotionPlan(
        const std::vector<motion_planning::StoredMotionPlan>& plans,
        const QString& robotId,
        const QString& trajectoryId)
    {
        if(robotId.isEmpty() || trajectoryId.isEmpty()) {
            return nullptr;
        }

        const std::string selectedRobotId = robotId.toStdString();
        const std::string selectedTrajectoryId = trajectoryId.toStdString();
        const auto planIt = std::find_if(
            plans.begin(),
            plans.end(),
            [&](const motion_planning::StoredMotionPlan& plan) {
                return plan.id == selectedTrajectoryId && plan.robotId == selectedRobotId;
            });
        return planIt == plans.end() ? nullptr : &(*planIt);
    }

    MotionPlanningEditorWidget::ControlPointPoseEditorData editorDataFromCartesianPoint(
        const robottrajectory::TimedCartesianPoint& point)
    {
        MotionPlanningEditorWidget::ControlPointPoseEditorData data;
        data.time = point.time;
        const Eigen::Vector3d translation = point.tcpPose.translation();
        data.x = translation.x();
        data.y = translation.y();
        data.z = translation.z();

        const Eigen::Vector3d euler = point.tcpPose.linear().eulerAngles(2, 1, 0);
        data.yawDeg = euler[0] * 180.0 / kPi;
        data.pitchDeg = euler[1] * 180.0 / kPi;
        data.rollDeg = euler[2] * 180.0 / kPi;
        return data;
    }

    robottrajectory::TimedCartesianPoint cartesianPointFromEditorData(
        const MotionPlanningEditorWidget::ControlPointPoseEditorData& data,
        const robottrajectory::TimedCartesianPoint& templatePoint)
    {
        robottrajectory::TimedCartesianPoint point = templatePoint;
        point.time = data.time;
        point.tcpPose = Eigen::Isometry3d::Identity();
        point.tcpPose.translation() = Eigen::Vector3d(data.x, data.y, data.z);

        const double roll = data.rollDeg * kPi / 180.0;
        const double pitch = data.pitchDeg * kPi / 180.0;
        const double yaw = data.yawDeg * kPi / 180.0;
        point.tcpPose.linear() = (
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
        return point;
    }

    std::filesystem::path toFilesystemPath(const QString& path)
    {
#ifdef _WIN32
        return std::filesystem::path(path.toStdWString());
#else
        return std::filesystem::path(path.toStdString());
#endif
    }

    std::filesystem::path projectBasePath(const simulation_project::ProjectSession& session)
    {
        return session.path().empty()
            ? simulation_project::RuntimePaths::applicationRoot()
            : session.path().parent_path();
    }

    QString firstDiagnosticMessage(
        const std::vector<motion_planning::MotionPlanningDiagnostic>& diagnostics,
        const QString& fallback)
    {
        return diagnostics.empty()
            ? fallback
            : QString::fromStdString(diagnostics.front().message);
    }

    std::vector<double> maybeMapIrb4600JointSigns(
        const simulation_project::ProjectDocument& document,
        const QString& robotId,
        const std::vector<double>& joints)
    {
        return usesIrb4600RobotSystemJointSigns(document, robotId)
            ? motion_planning::ProjectTrajectoryInverseKinematics::irb4600RobotSystemJointValues(joints)
            : joints;
    }

    QString cdfRepairSummary(
        const motion_planning::ProjectCdfQpRepairResult& result,
        const QString& planId)
    {
        return QStringLiteral("%1 %2 as %3. min phi: %4 -> %5, iterations=%6, qp_iters=%7, slack=%8, queries=%9")
            .arg(result.success
                ? QStringLiteral("Stored repaired APF + CDF/QP trajectory")
                : QStringLiteral("Stored collision-free APF + CDF/QP trajectory below requested clearance"))
            .arg(static_cast<int>(result.plan.trajectory.points.size()))
            .arg(planId)
            .arg(formatDouble(result.statistics.initialMinimumPhi))
            .arg(formatDouble(result.statistics.finalMinimumPhi))
            .arg(result.statistics.iterations)
            .arg(result.statistics.qpIterations)
            .arg(formatDouble(result.statistics.maximumSlack))
            .arg(result.statistics.collisionQueries);
    }

    bool isCdfQpTrajectory(const motion_planning::StoredMotionPlan& plan)
    {
        return plan.id.find("_cdf_qp_") != std::string::npos &&
            !plan.trajectory.empty();
    }
}

namespace robot_qt_viewer
{
    MotionPlanningModuleController::MotionPlanningModuleController(
        MotionPlanningEditorWidget& widget,
        RobotQtViewerDocumentContext& context,
        QObject* parent)
        : QObject(parent)
        , m_widget(widget)
        , m_context(context)
    {
        m_playbackTimer = new QTimer(this);
        m_playbackTimer->setTimerType(Qt::PreciseTimer);
        connect(&m_widget, &MotionPlanningEditorWidget::planRequested,
            this, &MotionPlanningModuleController::planTrajectory);
        connect(&m_widget, &MotionPlanningEditorWidget::importTrajectoryRequested,
            this, &MotionPlanningModuleController::importTrajectory);
        connect(&m_widget, &MotionPlanningEditorWidget::importCdfJointAnglesRequested,
            this, &MotionPlanningModuleController::importCdfJointAngles);
        connect(&m_widget, &MotionPlanningEditorWidget::multiIkRequested,
            this, &MotionPlanningModuleController::solveMultiIk);
        connect(&m_widget, &MotionPlanningEditorWidget::multiIkCancelRequested, this, [this]() {
            if(m_multiIkCancel) { m_multiIkCancel->store(true); }
        });
        connect(&m_widget, &MotionPlanningEditorWidget::multiIkTargetChanged,
            this, &MotionPlanningModuleController::invalidateMultiIk);
        connect(&m_widget, &MotionPlanningEditorWidget::multiIkPointChanged,
            this, &MotionPlanningModuleController::showMultiIkPoint);
        connect(&m_widget, &MotionPlanningEditorWidget::multiIkApplyRequested,
            this, &MotionPlanningModuleController::applyMultiIk);
        connect(&m_widget, &MotionPlanningEditorWidget::multiIkSelectRequested,
            this, &MotionPlanningModuleController::selectMultiIk);
        connect(&m_widget, &MotionPlanningEditorWidget::multiIkPlaybackRequested,
            this, &MotionPlanningModuleController::playMultiIk);
        connect(&m_widget, &MotionPlanningEditorWidget::inverseKinematicsRequested,
            this, &MotionPlanningModuleController::solveInverseKinematics);
        connect(&m_widget, &MotionPlanningEditorWidget::applySelectedJointPointRequested,
            this, &MotionPlanningModuleController::applySelectedJointPoint);
        connect(&m_widget, &MotionPlanningEditorWidget::applySelectedCdfJointAnglesRequested,
            this, &MotionPlanningModuleController::applySelectedCdfJointAngles);
        connect(&m_widget, &MotionPlanningEditorWidget::repairImportedCdfTrajectoryRequested,
            this, &MotionPlanningModuleController::repairImportedCdfTrajectory);
        connect(&m_widget, &MotionPlanningEditorWidget::exportJointTrajectoryRequested,
            this, [this]() { exportJointTrajectory(false); });
        connect(&m_widget, &MotionPlanningEditorWidget::trajectoryPointsVisibilityChanged,
            this, [this](bool visible) {
                m_trajectoryPointsVisible = visible;
                refreshTrajectoryView();
            });
        connect(&m_widget, &MotionPlanningEditorWidget::exportCdfTrajectoryRequested,
            this, &MotionPlanningModuleController::exportCdfTrajectory);
        connect(&m_widget, &MotionPlanningEditorWidget::insertControlPointBeforeRequested,
            this, &MotionPlanningModuleController::insertControlPointBefore);
        connect(&m_widget, &MotionPlanningEditorWidget::insertControlPointAfterRequested,
            this, &MotionPlanningModuleController::insertControlPointAfter);
        connect(&m_widget, &MotionPlanningEditorWidget::deleteControlPointRequested,
            this, &MotionPlanningModuleController::deleteControlPoint);
        connect(&m_widget, &MotionPlanningEditorWidget::editControlPointRequested,
            this, &MotionPlanningModuleController::editControlPoint);
        connect(&m_widget, &MotionPlanningEditorWidget::playbackRequested,
            this, &MotionPlanningModuleController::startJointPlayback);
        connect(&m_widget, &MotionPlanningEditorWidget::playbackStopRequested,
            this, &MotionPlanningModuleController::stopJointPlayback);
        connect(&m_widget, &MotionPlanningEditorWidget::sprayRangeVisibilityChanged,
            this, &MotionPlanningModuleController::setSprayRangeVisible);
        connect(&m_widget, &MotionPlanningEditorWidget::endEffectorTraceVisibilityChanged,
            this, &MotionPlanningModuleController::setEndEffectorTraceVisible);
        connect(&m_widget, &MotionPlanningEditorWidget::sprayMeasurementEnabledChanged,
            this, &MotionPlanningModuleController::setSprayMeasurementEnabled);
        connect(&m_widget, &MotionPlanningEditorWidget::exportSprayMeasurementsRequested,
            this, &MotionPlanningModuleController::exportSprayMeasurements);
        connect(&m_widget, &MotionPlanningEditorWidget::plotSprayMeasurementsRequested,
            this, &MotionPlanningModuleController::plotSprayMeasurements);
        connect(&m_widget, &MotionPlanningEditorWidget::trajectorySelectionChanged,
            this, &MotionPlanningModuleController::setSelectedTrajectory);
        connect(m_playbackTimer, &QTimer::timeout,
            this, &MotionPlanningModuleController::advanceJointPlayback);
        setSelectedRobot(m_context.selectionModel().state().robotId);
        ensurePersistentCdfCollisionSetup();
        refreshTrajectoryView();
    }

    MotionPlanningModuleController::~MotionPlanningModuleController()
    {
        if(m_multiIkCancel) { m_multiIkCancel->store(true); }
        if(m_multiIkThread) { m_multiIkThread->wait(); }
    }

    void MotionPlanningModuleController::ensurePersistentCdfCollisionSetup()
    {
        const motion_planning::ProjectCdfQpRepairOptions options;
        const QString cdfRobotId =
            defaultCdfPlanningRobotId(m_context.document(), m_selectedRobotId);
        if(cdfRobotId.isEmpty()) {
            return;
        }

        const std::string robotId = cdfRobotId.toStdString();
        if(cdfDetectorIsConfigured(m_context.document(), robotId, options)) {
            return;
        }

        std::vector<motion_planning::MotionPlanningDiagnostic> setupDiagnostics;
        const ProjectMutationResult setupMutation = m_context.documentController().mutateProject(
            QStringLiteral("motionPlanningPersistentCdfCollisionSetup"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                if(cdfDetectorIsConfigured(service.document(), robotId, options)) {
                    changed = false;
                    return true;
                }

                std::string detectorId;
                if(!motion_planning::ProjectCdfQpTrajectoryRepairService::ensureCollisionSetup(
                       service.document(),
                       robotId,
                       options,
                       &detectorId,
                       &setupDiagnostics)) {
                    error = setupDiagnostics.empty()
                        ? std::string("Failed to configure persistent ABB4600_urdf-burnner CDF collision detector.")
                        : setupDiagnostics.front().message;
                    return false;
                }
                changed = true;
                return true;
            });

        if(!setupMutation.success) {
            emit statusMessageRequested(setupMutation.message, 7000);
        }
    }

    void MotionPlanningModuleController::handleEvent(const RobotQtViewerEvent& event)
    {
        // Applying joints refreshes ToolSetup, which republishes pinned-frame display
        // settings. Only geometric previews invalidate the FK used by multi IK.
        bool kinematicPreviewChanged = false;
        if(event.kind == RobotQtViewerEventKind::ViewportPreviewChanged) {
            const auto& preview = m_context.viewportPreviewState().lastMutation();
            kinematicPreviewChanged = preview.previewRobotBaseTransform ||
                preview.previewRobotMountTransform || preview.upsertPreviewRobotMount ||
                preview.removePreviewRobotMount || preview.upsertPreviewObjectFrame ||
                preview.previewObjectFrameTransform || preview.previewSceneObjectTransform;
        }
        if(event.kind == RobotQtViewerEventKind::ProjectOpened ||
            event.kind == RobotQtViewerEventKind::ToolSetupChanged ||
            event.kind == RobotQtViewerEventKind::AttachmentChanged ||
            kinematicPreviewChanged ||
            (event.kind == RobotQtViewerEventKind::ProjectDocumentChanged &&
             event.sourceId != QStringLiteral("motionPlanningMultiIkApply") &&
             event.sourceId != QStringLiteral("motionPlanningPersistentCdfCollisionSetup"))) {
            invalidateMultiIk();
        }
        if(event.kind == RobotQtViewerEventKind::SelectionChanged) {
            setSelectedRobot(event.selection.robotId);
        } else if(event.kind == RobotQtViewerEventKind::ProjectOpened) {
            stopJointPlayback();
            clearSprayMeasurements();
            clearEndEffectorTrace();
            setSelectedRobot(m_context.selectionModel().state().robotId);
            ensurePersistentCdfCollisionSetup();
            refreshTrajectoryView();
        } else if(event.kind == RobotQtViewerEventKind::ProjectDocumentChanged) {
            setSprayRangeVisible(m_sprayRangeVisible);
            clearEndEffectorTrace();
            setEndEffectorTraceVisible(m_endEffectorTraceVisible);
            refreshTrajectoryView();
        }
    }

    void MotionPlanningModuleController::planTrajectory(
        const QString& startJoints,
        const QString& goalJoints,
        const QString& jointNames,
        double duration,
        int sampleCount)
    {
        if(m_selectedRobotId.isEmpty()) {
            m_widget.setResult(QStringLiteral("Select a robot before planning."), false);
            return;
        }

        motion_planning::MotionPlanningRequest request;
        request.robotId = m_selectedRobotId.toStdString();
        request.constraint.duration = duration;
        request.constraint.sampleCount = static_cast<std::size_t>(sampleCount);
        const QStringList jointNameParts = jointNames.split(',', Qt::SkipEmptyParts);
        for(const QString& jointName : jointNameParts) {
            request.jointNames.push_back(jointName.trimmed().toStdString());
        }

        QString parseError;
        if(!parseJointVector(startJoints, request.startJoints, parseError) ||
            !parseJointVector(goalJoints, request.goalJoints, parseError)) {
            m_widget.setResult(parseError, false);
            return;
        }

        const motion_planning::LinearJointMotionPlanner planner;
        const motion_planning::MotionPlanningResult planningResult = planner.plan(request);
        if(!planningResult.succeeded()) {
            const QString message = planningResult.diagnostics.empty()
                ? QStringLiteral("Motion planning failed.")
                : QString::fromStdString(planningResult.diagnostics.front().message);
            m_widget.setResult(message, false);
            emit statusMessageRequested(message, 5000);
            return;
        }

        motion_planning::StoredMotionPlan plan;
        plan.id = request.robotId + "_linear_plan";
        plan.name = plan.id;
        plan.robotId = request.robotId;
        plan.jointNames = request.jointNames;
        plan.trajectory = planningResult.trajectory;

        const ProjectMutationResult mutation = m_context.documentController().mutateProject(
            QStringLiteral("motionPlanning"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                if(!motion_planning::MotionPlanningProjectStore::upsertPlan(
                       service.document(), plan, &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutation.success) {
            m_widget.setResult(mutation.message, false);
            emit statusMessageRequested(mutation.message, 5000);
            return;
        }

        const QString planId = QString::fromStdString(plan.id);
        const QString summary = QStringLiteral("Stored %1 points as %2")
            .arg(static_cast<int>(plan.trajectory.points.size()))
            .arg(planId);
        m_selectedTrajectoryId = planId;
        refreshTrajectoryView();
        m_widget.setResult(summary, true);
        emit trajectoryPlanned(planId);
        emit statusMessageRequested(summary, 4000);
    }

    void MotionPlanningModuleController::importTrajectory()
    {
        if(m_selectedRobotId.isEmpty()) {
            m_widget.setResult(QStringLiteral("Select a robot before importing a trajectory."), false);
            return;
        }

        const QString filename = QFileDialog::getOpenFileName(
            &m_widget,
            QStringLiteral("Import trajectory"),
            QString(),
            QStringLiteral("Trajectory Files (*.csv *.txt *.json *.kf *.mod);;CSV Files (*.csv);;Text Files (*.txt);;JSON Files (*.json);;ABB Robot Programs (*.mod);;KeyFrame Files (*.kf);;All Files (*)"));
        if(filename.isEmpty()) {
            return;
        }

        motion_planning::TrajectoryImportOptions options;
        options.robotId = m_selectedRobotId.toStdString();
        options.jointNames = selectedRobotJointNames(m_context.document(), m_selectedRobotId);

        const motion_planning::TrajectoryImportResult importResult =
            motion_planning::ProjectTrajectoryImporter::importFile(
                toFilesystemPath(filename),
                options);
        if(!importResult.success) {
            const QString message = importResult.diagnostics.empty()
                ? QStringLiteral("Trajectory import failed.")
                : QString::fromStdString(importResult.diagnostics.front().message);
            m_widget.setResult(message, false);
            emit statusMessageRequested(message, 6000);
            return;
        }

        const motion_planning::StoredMotionPlan plan = importResult.plan;
        const ProjectMutationResult mutation = m_context.documentController().mutateProject(
            QStringLiteral("motionPlanningImport"),
            ProjectDirtyPolicy::UserEdit,
            [plan](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                if(!motion_planning::MotionPlanningProjectStore::upsertPlan(
                       service.document(), plan, &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutation.success) {
            m_widget.setResult(mutation.message, false);
            emit statusMessageRequested(mutation.message, 6000);
            return;
        }

        const int pointCount = !plan.trajectory.empty()
            ? static_cast<int>(plan.trajectory.points.size())
            : static_cast<int>(plan.cartesianControlPoints.points.size());
        const QString kindText = !plan.trajectory.empty()
            ? QStringLiteral("joint trajectory")
            : QStringLiteral("cartesian control points");
        const QString planId = QString::fromStdString(plan.id);
        m_selectedTrajectoryId = planId;
        refreshTrajectoryView();

        QString summary = QStringLiteral("Imported %1 %2 as %3")
            .arg(pointCount)
            .arg(kindText)
            .arg(planId);
        if(!importResult.diagnostics.empty()) {
            summary += QStringLiteral(". %1")
                .arg(QString::fromStdString(importResult.diagnostics.front().message));
        }
        m_widget.setResult(summary, true);
        emit trajectoryPlanned(planId);
        emit statusMessageRequested(summary, 5000);
    }

    void MotionPlanningModuleController::importCdfJointAngles()
    {
        if(m_selectedRobotId.isEmpty()) {
            m_widget.setCdfResult(QStringLiteral("Select a robot before importing CDF joint angles."), false);
            return;
        }

        const QString filename = QFileDialog::getOpenFileName(
            &m_widget,
            QStringLiteral("Import CDF joint angles"),
            QString(),
            QStringLiteral("Text Files (*.txt *.csv);;All Files (*)"));
        if(filename.isEmpty()) {
            return;
        }

        const motion_planning::CdfJointAngleImportResult importResult =
            motion_planning::ProjectCdfJointAngleImporter::importFile(
                toFilesystemPath(filename));
        if(!importResult.success) {
            const QString message = importResult.diagnostics.empty()
                ? QStringLiteral("CDF joint angle import failed.")
                : QString::fromStdString(importResult.diagnostics.front());
            m_widget.setCdfResult(message, false);
            emit statusMessageRequested(message, 6000);
            return;
        }

        m_cdfSourceName = QString::fromStdString(importResult.sourceName);
        m_cdfJointNames = importResult.jointNames;
        m_cdfJointPoints.clear();
        m_cdfJointPoints.reserve(importResult.points.size());
        for(const motion_planning::CdfJointAnglePoint& point : importResult.points) {
            ImportedCdfJointPoint copy;
            copy.timeSeconds = point.timeSeconds;
            copy.jointAnglesDegrees = point.jointAnglesDegrees;
            m_cdfJointPoints.push_back(std::move(copy));
        }
        refreshCdfJointAngleView();
        m_widget.setCdfExportAvailable(false);

        const QString summary = QStringLiteral("Imported %1 CDF joint angle points from %2")
            .arg(static_cast<int>(m_cdfJointPoints.size()))
            .arg(m_cdfSourceName);
        m_widget.setCdfResult(summary, true);
        emit statusMessageRequested(summary, 5000);
    }

    void MotionPlanningModuleController::solveInverseKinematics(bool useToolTransform)
    {
        if(m_selectedRobotId.isEmpty()) {
            m_widget.setResult(QStringLiteral("Select a robot before solving IK."), false);
            return;
        }
        if(m_selectedTrajectoryId.isEmpty()) {
            m_widget.setResult(QStringLiteral("Select an imported cartesian trajectory before solving IK."), false);
            return;
        }

        const std::vector<motion_planning::StoredMotionPlan> plans =
            motion_planning::MotionPlanningProjectStore::plans(m_context.document());
        const std::string selectedId = m_selectedTrajectoryId.toStdString();
        const auto planIt = std::find_if(
            plans.begin(),
            plans.end(),
            [&](const motion_planning::StoredMotionPlan& plan) {
                return plan.id == selectedId && plan.robotId == m_selectedRobotId.toStdString();
            });
        if(planIt == plans.end()) {
            m_widget.setResult(QStringLiteral("Selected trajectory is not available for this robot."), false);
            return;
        }
        if(planIt->cartesianControlPoints.empty()) {
            m_widget.setResult(QStringLiteral("Selected trajectory has no cartesian control points."), false);
            return;
        }

        motion_planning::CartesianIkOptions options;
        options.robotId = m_selectedRobotId.toStdString();
        options.jointNames = selectedRobotJointNames(m_context.document(), m_selectedRobotId);
        options.toolMode = useToolTransform
            ? motion_planning::CartesianIkToolMode::FixedTool
            : motion_planning::CartesianIkToolMode::Flange;
        options.maxIterations = 5000;
        options.tolerance = 1.0e-6;
        options.stepSize = 1.0;
        options.damping = 0.001;

        if(RobotQtViewerViewportServices* viewportServices = m_context.viewportServices()) {
            for(const std::string& jointName : options.jointNames) {
                bool ok = false;
                const double value = viewportServices->robotJointValue(
                    m_selectedRobotId,
                    QString::fromStdString(jointName),
                    &ok);
                if(!ok) {
                    options.seedJoints.clear();
                    break;
                }
                options.seedJoints.push_back(value);
            }
        }

        auto* viewportServices = m_context.viewportServices();
        const auto runtimeFk = viewportServices
            ? viewportServices->robotForwardKinematics(m_selectedRobotId, options.jointNames, useToolTransform)
            : RobotQtViewerViewportServices::RobotForwardKinematics{};
        if(!runtimeFk) {
            m_widget.setResult(QStringLiteral("Cannot obtain the selected robot model and TCP for IK."), false);
            return;
        }
        const bool mapSigns = usesIrb4600RobotSystemJointSigns(m_context.document(), m_selectedRobotId);
        if(mapSigns) {
            options.seedJoints = motion_planning::ProjectTrajectoryInverseKinematics::irb4600RobotSystemJointValues(
                options.seedJoints);
        }
        options.worldForwardKinematics = [runtimeFk, mapSigns](const std::vector<double>& joints) {
            return runtimeFk(mapSigns
                ? motion_planning::ProjectTrajectoryInverseKinematics::irb4600RobotSystemJointValues(joints)
                : joints);
        };
        stopJointPlayback();
        const motion_planning::CartesianIkResult ikResult =
            motion_planning::ProjectTrajectoryInverseKinematics::solveCartesianControlPoints(
                m_context.document(),
                *planIt,
                options);
        if(!ikResult.success) {
            QString message = ikResult.diagnostics.empty()
                ? QStringLiteral("Inverse kinematics failed.")
                : QString::fromStdString(ikResult.diagnostics.front().message);
            message += QStringLiteral(" Solved %1/%2 points.")
                .arg(static_cast<int>(ikResult.solvedPointCount()))
                .arg(static_cast<int>(ikResult.points.size()));
            m_widget.setResult(message, false);
            emit statusMessageRequested(message, 7000);
            return;
        }

        const motion_planning::StoredMotionPlan solvedPlan = ikResult.plan;
        const ProjectMutationResult mutation = m_context.documentController().mutateProject(
            QStringLiteral("motionPlanningIK"),
            ProjectDirtyPolicy::UserEdit,
            [solvedPlan](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                if(!motion_planning::MotionPlanningProjectStore::upsertPlan(
                       service.document(), solvedPlan, &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutation.success) {
            m_widget.setResult(mutation.message, false);
            emit statusMessageRequested(mutation.message, 7000);
            return;
        }

        m_selectedTrajectoryId = QString::fromStdString(solvedPlan.id);
        refreshTrajectoryView();

        if(!solvedPlan.trajectory.points.empty()) {
            applyJointValuesToRobot(
                solvedPlan.jointNames,
                solvedPlan.trajectory.points.front().q,
                QStringLiteral("motionPlanningIKApply"));
        }

        const QString mode = useToolTransform
            ? QStringLiteral("with tool")
            : QStringLiteral("flange");
        const QString summary = QStringLiteral("Solved IK for %1 points (%2) as %3")
            .arg(static_cast<int>(solvedPlan.trajectory.points.size()))
            .arg(mode)
            .arg(QString::fromStdString(solvedPlan.id));
        m_widget.setResult(summary, true);
        emit trajectoryPlanned(QString::fromStdString(solvedPlan.id));
        emit statusMessageRequested(summary, 5000);
    }

    void MotionPlanningModuleController::applySelectedJointPoint(int pointIndex)
    {
        stopJointPlayback();
        if(pointIndex < 0 || m_selectedRobotId.isEmpty() || m_selectedTrajectoryId.isEmpty()) {
            return;
        }

        const std::vector<motion_planning::StoredMotionPlan> plans =
            motion_planning::MotionPlanningProjectStore::plans(m_context.document());
        const std::string selectedId = m_selectedTrajectoryId.toStdString();
        const auto planIt = std::find_if(
            plans.begin(),
            plans.end(),
            [&](const motion_planning::StoredMotionPlan& plan) {
                return plan.id == selectedId && plan.robotId == m_selectedRobotId.toStdString();
            });
        if(planIt == plans.end() || planIt->trajectory.empty()) {
            m_widget.setResult(QStringLiteral("Selected trajectory has no joint values to apply."), false);
            return;
        }
        const std::size_t index = static_cast<std::size_t>(pointIndex);
        if(index >= planIt->trajectory.points.size()) {
            m_widget.setResult(QStringLiteral("Selected trajectory point is out of range."), false);
            return;
        }

        if(applyJointValuesToRobot(
               planIt->jointNames,
               planIt->trajectory.points[index].q,
               QStringLiteral("motionPlanningApplyJointPoint"))) {
            const QString summary = QStringLiteral("Applied joint point %1 to %2")
                .arg(pointIndex + 1)
                .arg(m_selectedRobotId);
            m_widget.setResult(summary, true);
            emit statusMessageRequested(summary, 4000);
        }
    }

    void MotionPlanningModuleController::applySelectedCdfJointAngles(int pointIndex)
    {
        stopJointPlayback();
        if(pointIndex < 0 || m_selectedRobotId.isEmpty()) {
            return;
        }
        if(m_cdfJointPoints.empty()) {
            m_widget.setCdfResult(QStringLiteral("Import a CDF joint angle file before applying joint angles."), false);
            return;
        }

        const std::size_t index = static_cast<std::size_t>(pointIndex);
        if(index >= m_cdfJointPoints.size()) {
            m_widget.setCdfResult(QStringLiteral("Selected CDF joint angle row is out of range."), false);
            return;
        }

        const std::vector<std::string> robotJointNames =
            selectedRobotJointNames(m_context.document(), m_selectedRobotId);
        if(robotJointNames.size() != m_cdfJointPoints[index].jointAnglesDegrees.size()) {
            const QString message = QStringLiteral("CDF joint angle count (%1) does not match robot joint count (%2).")
                .arg(static_cast<int>(m_cdfJointPoints[index].jointAnglesDegrees.size()))
                .arg(static_cast<int>(robotJointNames.size()));
            m_widget.setCdfResult(message, false);
            emit statusMessageRequested(message, 6000);
            return;
        }

        const std::vector<double> jointValuesRadians =
            degreesToRadians(m_cdfJointPoints[index].jointAnglesDegrees);
        if(applyJointValuesToRobot(
               robotJointNames,
               jointValuesRadians,
               QStringLiteral("motionPlanningApplyCdfJointAngles"))) {
            const QString summary = QStringLiteral("Applied CDF joint angle row %1 to %2")
                .arg(pointIndex + 1)
                .arg(m_selectedRobotId);
            m_widget.setCdfResult(summary, true);
            emit statusMessageRequested(summary, 4000);
        }
    }

    void MotionPlanningModuleController::repairImportedCdfTrajectory()
    {
        if(m_selectedRobotId.isEmpty()) {
            m_widget.setCdfResult(QStringLiteral("Select ABB4600_urdf before running CDF/QP repair."), false);
            return;
        }
        if(m_cdfJointPoints.size() < 2) {
            m_widget.setCdfResult(QStringLiteral("Import at least two CDF joint angle rows before repair."), false);
            return;
        }

        stopJointPlayback();

        const std::vector<std::string> robotJointNames =
            selectedRobotJointNames(m_context.document(), m_selectedRobotId);
        if(robotJointNames.empty()) {
            m_widget.setCdfResult(QStringLiteral("Selected robot has no movable joints for CDF/QP repair."), false);
            return;
        }

        for(std::size_t index = 0; index < m_cdfJointPoints.size(); ++index) {
            if(m_cdfJointPoints[index].jointAnglesDegrees.size() != robotJointNames.size()) {
                const QString message = QStringLiteral("CDF joint angle row %1 has %2 joints, but robot %3 has %4 movable joints.")
                    .arg(static_cast<int>(index + 1))
                    .arg(static_cast<int>(m_cdfJointPoints[index].jointAnglesDegrees.size()))
                    .arg(m_selectedRobotId)
                    .arg(static_cast<int>(robotJointNames.size()));
                m_widget.setCdfResult(message, false);
                emit statusMessageRequested(message, 7000);
                return;
            }
        }

        robottrajectory::JointTrajectory seedTrajectory;
        seedTrajectory.name = "cdf_import_seed";
        seedTrajectory.interpolation = robottrajectory::TrajectoryInterpolation::Linear;
        seedTrajectory.points.reserve(m_cdfJointPoints.size());
        for(const ImportedCdfJointPoint& importedPoint : m_cdfJointPoints) {
            robottrajectory::TimedJointPoint point;
            point.time = importedPoint.timeSeconds;
            point.q = maybeMapIrb4600JointSigns(
                m_context.document(),
                m_selectedRobotId,
                degreesToRadians(importedPoint.jointAnglesDegrees));
            seedTrajectory.points.push_back(std::move(point));
        }
        seedTrajectory.sortByTime();

        const MotionPlanningEditorWidget::CdfQpRepairSettings settings =
            m_widget.cdfQpRepairSettings();
        motion_planning::ProjectCdfQpRepairOptions options;
        options.safetyMargin = settings.safetyMargin;
        options.targetClearance = settings.targetClearance;
        options.finiteDifferenceStep = settings.finiteDifferenceStep;
        options.distanceThreshold = settings.distanceThreshold;
        options.trustRegion = settings.trustRegion;
        options.seedCorridor = settings.seedCorridor;
        options.seedTrackingWeight = settings.seedTrackingWeight;
        options.segmentIntermediateSamples = settings.segmentIntermediateSamples;
        options.maxIterations = settings.maxIterations;
        options.keepEndpoints = settings.keepEndpoints;

        const std::string robotId = m_selectedRobotId.toStdString();
        if(!cdfDetectorIsConfigured(m_context.document(), robotId, options)) {
            const QString message = QStringLiteral(
                "CDF/QP collision detector is not configured. Reopen the project or select ABB4600_urdf to initialize cdf_abb4600_burnner_detector.");
            m_widget.setCdfResult(message, false);
            emit statusMessageRequested(message, 7000);
            return;
        }

        const motion_planning::ProjectCdfQpTrajectoryRepairService repairService;
        motion_planning::ProjectCdfQpRepairResult repairResult =
            repairService.repair(
                m_context.document(),
                projectBasePath(m_context.projectSession()),
                robotId,
                robotJointNames,
                seedTrajectory,
                options);

        if(repairResult.plan.trajectory.empty()) {
            const QString message = repairResult.diagnostics.empty()
                ? QStringLiteral("APF + CDF/QP repair failed before producing a trajectory.")
                : QString::fromStdString(repairResult.diagnostics.back().message);
            m_widget.setCdfResult(message, false);
            emit statusMessageRequested(message, 7000);
            return;
        }

        motion_planning::StoredMotionPlan storedPlan = repairResult.plan;
        if(!repairResult.success) {
            storedPlan.id = robotId + "_cdf_qp_partial";
            storedPlan.name = "APF + CDF/QP trajectory (clearance not reached)";
            storedPlan.trajectory.name = storedPlan.id;
        }
        for(robottrajectory::TimedJointPoint& point : storedPlan.trajectory.points) {
            point.q = maybeMapIrb4600JointSigns(m_context.document(), m_selectedRobotId, point.q);
        }

        if(!commitMotionPlanUpdate(storedPlan, QStringLiteral("motionPlanningCdfQpRepair"))) {
            m_widget.setCdfResult(QStringLiteral("CDF/QP repair produced a trajectory but storing it failed."), false);
            return;
        }

        m_cdfSourceName = QStringLiteral("%1 repaired by APF + CDF/QP").arg(m_cdfSourceName.isEmpty()
            ? QStringLiteral("Imported trajectory")
            : m_cdfSourceName);
        m_cdfJointNames = robotJointNames;
        m_cdfJointPoints.clear();
        m_cdfJointPoints.reserve(storedPlan.trajectory.points.size());
        for(const robottrajectory::TimedJointPoint& point : storedPlan.trajectory.points) {
            ImportedCdfJointPoint importedPoint;
            importedPoint.timeSeconds = point.time;
            importedPoint.jointAnglesDegrees = radiansToDegrees(point.q);
            m_cdfJointPoints.push_back(std::move(importedPoint));
        }
        refreshCdfJointAngleView();
        refreshTrajectoryView();

        const QString planId = QString::fromStdString(storedPlan.id);
        const QString diagnosticText = repairResult.success
            ? QString()
            : QStringLiteral(" %1").arg(repairResult.diagnostics.empty()
                ? QStringLiteral("The repaired path still violates the requested clearance.")
                : QString::fromStdString(repairResult.diagnostics.back().message));
        const QString summary = cdfRepairSummary(repairResult, planId) + diagnosticText;
        m_widget.setCdfResult(summary, repairResult.success);
        emit trajectoryPlanned(planId);
        emit statusMessageRequested(summary, repairResult.success ? 6000 : 9000);
    }

    void MotionPlanningModuleController::exportCdfTrajectory()
    {
        exportJointTrajectory(true);
    }

    void MotionPlanningModuleController::exportJointTrajectory(bool cdfOnly)
    {
        const auto showResult = [this, cdfOnly](const QString& message, bool success) {
            if(cdfOnly) { m_widget.setCdfResult(message, success); }
            else { m_widget.setResult(message, success); }
            emit statusMessageRequested(message, 6000);
        };
        const auto plans = motion_planning::MotionPlanningProjectStore::plans(m_context.document());
        const auto* selectedPlan = findMotionPlan(plans, m_selectedRobotId, m_selectedTrajectoryId);
        if(selectedPlan == nullptr || selectedPlan->trajectory.empty() || selectedPlan->jointNames.empty()) {
            showResult(QStringLiteral("Select a solved joint trajectory before exporting."), false);
            return;
        }
        if(cdfOnly && !isCdfQpTrajectory(*selectedPlan)) {
            showResult(QStringLiteral("Select a trajectory produced by APF + CDF/QP repair before exporting."), false);
            return;
        }
        // Validate every source row, including rows omitted from the sampled UI table.
        for(const auto& point : selectedPlan->trajectory.points) {
            if(point.q.size() != selectedPlan->jointNames.size() || !std::isfinite(point.time) ||
                !std::all_of(point.q.begin(), point.q.end(), [](double q) {
                    return std::isfinite(q) && std::isfinite(q * 180.0 / kPi);
                })) {
                showResult(QStringLiteral("The trajectory contains an invalid joint row."), false);
                return;
            }
        }
        QString outputPath = getSaveFileName(
            cdfOnly ? QStringLiteral("motionPlanning.cdfTrajectory.save")
                    : QStringLiteral("motionPlanning.jointTrajectory.save"),
            &m_widget,
            cdfOnly ? QStringLiteral("Export APF + CDF/QP trajectory")
                    : QStringLiteral("\u5bfc\u51fa\u5173\u8282\u8f68\u8ff9"),
            cdfOnly ? QStringLiteral("%1_apf_cdf_qp_trajectory.txt").arg(m_selectedRobotId)
                    : QStringLiteral("ik_joint_angles.txt"),
            QStringLiteral("Text Files (*.txt);;All Files (*)"));
        if(outputPath.isEmpty()) { return; }
        if(!outputPath.endsWith(QStringLiteral(".txt"), Qt::CaseInsensitive)) {
            outputPath += QStringLiteral(".txt");
        }
        QSaveFile file(outputPath);
        if(!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            showResult(QStringLiteral("Failed to open trajectory export file: %1").arg(file.errorString()), false);
            return;
        }
        QTextStream text(&file);
        text.setLocale(QLocale::c());
        text.setRealNumberNotation(QTextStream::FixedNotation);
        text.setRealNumberPrecision(6);
        text << "# IK joint angle export\n";
        text << "# Time unit: seconds\n";
        text << "# Joint angle unit: degrees\n";
        text << "time_s";
        for(std::size_t index = 0; index < selectedPlan->jointNames.size(); ++index) {
            text << '\t' << "J" << static_cast<qulonglong>(index + 1) << "_deg";
        }
        text << '\n';
        for(const auto& point : selectedPlan->trajectory.points) {
            text << point.time;
            for(double jointValueRadians : point.q) {
                // Export stored IK convention, before any runtime joint-sign mapping.
                text << '\t' << (jointValueRadians * 180.0 / kPi);
            }
            text << '\n';
        }
        text.flush();
        if(text.status() != QTextStream::Ok || !file.commit()) {
            showResult(QStringLiteral("Failed to save trajectory export: %1").arg(file.errorString()), false);
            return;
        }
        showResult(QStringLiteral("Exported %1 joint angle points to %2")
            .arg(static_cast<qulonglong>(selectedPlan->trajectory.points.size())).arg(outputPath), true);
    }

    void MotionPlanningModuleController::insertControlPointBefore(int pointIndex)
    {
        if(pointIndex < 0) {
            return;
        }
        stopJointPlayback();

        const std::vector<motion_planning::StoredMotionPlan> plans =
            motion_planning::MotionPlanningProjectStore::plans(m_context.document());
        const motion_planning::StoredMotionPlan* selectedPlan =
            findMotionPlan(plans, m_selectedRobotId, m_selectedTrajectoryId);
        if(selectedPlan == nullptr || selectedPlan->cartesianControlPoints.empty()) {
            m_widget.setResult(QStringLiteral("Selected trajectory has no cartesian control points to edit."), false);
            return;
        }

        robottrajectory::TimedCartesianPoint insertedPoint;
        std::string error;
        if(!motion_planning::ProjectTrajectoryControlPointEditor::makeInsertedCartesianControlPoint(
               *selectedPlan,
               static_cast<std::size_t>(pointIndex),
               motion_planning::CartesianControlPointInsertLocation::Before,
               insertedPoint,
               &error)) {
            const QString message = QString::fromStdString(error);
            m_widget.setResult(message, false);
            emit statusMessageRequested(message, 5000);
            return;
        }

        MotionPlanningEditorWidget::ControlPointPoseEditorData editorData =
            editorDataFromCartesianPoint(insertedPoint);
        if(!m_widget.editControlPointPose(
               editorData,
               QStringLiteral("Insert Control Point Before %1").arg(pointIndex + 1))) {
            return;
        }

        motion_planning::StoredMotionPlan updatedPlan = *selectedPlan;
        insertedPoint = cartesianPointFromEditorData(editorData, insertedPoint);
        if(!motion_planning::ProjectTrajectoryControlPointEditor::insertCartesianControlPoint(
               updatedPlan,
               static_cast<std::size_t>(pointIndex),
               motion_planning::CartesianControlPointInsertLocation::Before,
               insertedPoint,
               &error)) {
            const QString message = QString::fromStdString(error);
            m_widget.setResult(message, false);
            emit statusMessageRequested(message, 5000);
            return;
        }

        if(commitMotionPlanUpdate(updatedPlan, QStringLiteral("motionPlanningInsertControlPoint"))) {
            refreshTrajectoryView();
            const QString summary = QStringLiteral("Inserted control point before %1")
                .arg(pointIndex + 1);
            m_widget.setResult(summary, true);
            emit statusMessageRequested(summary, 4000);
        }
    }

    void MotionPlanningModuleController::insertControlPointAfter(int pointIndex)
    {
        if(pointIndex < 0) {
            return;
        }
        stopJointPlayback();

        const std::vector<motion_planning::StoredMotionPlan> plans =
            motion_planning::MotionPlanningProjectStore::plans(m_context.document());
        const motion_planning::StoredMotionPlan* selectedPlan =
            findMotionPlan(plans, m_selectedRobotId, m_selectedTrajectoryId);
        if(selectedPlan == nullptr || selectedPlan->cartesianControlPoints.empty()) {
            m_widget.setResult(QStringLiteral("Selected trajectory has no cartesian control points to edit."), false);
            return;
        }

        robottrajectory::TimedCartesianPoint insertedPoint;
        std::string error;
        if(!motion_planning::ProjectTrajectoryControlPointEditor::makeInsertedCartesianControlPoint(
               *selectedPlan,
               static_cast<std::size_t>(pointIndex),
               motion_planning::CartesianControlPointInsertLocation::After,
               insertedPoint,
               &error)) {
            const QString message = QString::fromStdString(error);
            m_widget.setResult(message, false);
            emit statusMessageRequested(message, 5000);
            return;
        }

        MotionPlanningEditorWidget::ControlPointPoseEditorData editorData =
            editorDataFromCartesianPoint(insertedPoint);
        if(!m_widget.editControlPointPose(
               editorData,
               QStringLiteral("Insert Control Point After %1").arg(pointIndex + 1))) {
            return;
        }

        motion_planning::StoredMotionPlan updatedPlan = *selectedPlan;
        insertedPoint = cartesianPointFromEditorData(editorData, insertedPoint);
        if(!motion_planning::ProjectTrajectoryControlPointEditor::insertCartesianControlPoint(
               updatedPlan,
               static_cast<std::size_t>(pointIndex),
               motion_planning::CartesianControlPointInsertLocation::After,
               insertedPoint,
               &error)) {
            const QString message = QString::fromStdString(error);
            m_widget.setResult(message, false);
            emit statusMessageRequested(message, 5000);
            return;
        }

        if(commitMotionPlanUpdate(updatedPlan, QStringLiteral("motionPlanningInsertControlPoint"))) {
            refreshTrajectoryView();
            const QString summary = QStringLiteral("Inserted control point after %1")
                .arg(pointIndex + 1);
            m_widget.setResult(summary, true);
            emit statusMessageRequested(summary, 4000);
        }
    }

    void MotionPlanningModuleController::deleteControlPoint(int pointIndex)
    {
        if(pointIndex < 0) {
            return;
        }
        stopJointPlayback();

        const std::vector<motion_planning::StoredMotionPlan> plans =
            motion_planning::MotionPlanningProjectStore::plans(m_context.document());
        const motion_planning::StoredMotionPlan* selectedPlan =
            findMotionPlan(plans, m_selectedRobotId, m_selectedTrajectoryId);
        if(selectedPlan == nullptr || selectedPlan->cartesianControlPoints.empty()) {
            m_widget.setResult(QStringLiteral("Selected trajectory has no cartesian control points to edit."), false);
            return;
        }

        motion_planning::StoredMotionPlan updatedPlan = *selectedPlan;
        std::string error;
        if(!motion_planning::ProjectTrajectoryControlPointEditor::removeCartesianControlPoint(
               updatedPlan,
               static_cast<std::size_t>(pointIndex),
               &error)) {
            const QString message = QString::fromStdString(error);
            m_widget.setResult(message, false);
            emit statusMessageRequested(message, 5000);
            return;
        }

        if(commitMotionPlanUpdate(updatedPlan, QStringLiteral("motionPlanningDeleteControlPoint"))) {
            refreshTrajectoryView();
            const QString summary = QStringLiteral("Deleted control point %1")
                .arg(pointIndex + 1);
            m_widget.setResult(summary, true);
            emit statusMessageRequested(summary, 4000);
        }
    }

    void MotionPlanningModuleController::editControlPoint(int pointIndex)
    {
        if(pointIndex < 0) {
            return;
        }
        stopJointPlayback();

        const std::vector<motion_planning::StoredMotionPlan> plans =
            motion_planning::MotionPlanningProjectStore::plans(m_context.document());
        const motion_planning::StoredMotionPlan* selectedPlan =
            findMotionPlan(plans, m_selectedRobotId, m_selectedTrajectoryId);
        if(selectedPlan == nullptr || selectedPlan->cartesianControlPoints.empty()) {
            m_widget.setResult(QStringLiteral("Selected trajectory has no cartesian control points to edit."), false);
            return;
        }

        const std::size_t index = static_cast<std::size_t>(pointIndex);
        if(index >= selectedPlan->cartesianControlPoints.points.size()) {
            m_widget.setResult(QStringLiteral("Selected control point is out of range."), false);
            return;
        }

        const robottrajectory::TimedCartesianPoint sourcePoint =
            selectedPlan->cartesianControlPoints.points[index];
        MotionPlanningEditorWidget::ControlPointPoseEditorData editorData =
            editorDataFromCartesianPoint(sourcePoint);
        if(!m_widget.editControlPointPose(
               editorData,
               QStringLiteral("Edit Control Point %1").arg(pointIndex + 1))) {
            return;
        }

        motion_planning::StoredMotionPlan updatedPlan = *selectedPlan;
        const robottrajectory::TimedCartesianPoint updatedPoint =
            cartesianPointFromEditorData(editorData, sourcePoint);

        std::string error;
        if(!motion_planning::ProjectTrajectoryControlPointEditor::updateCartesianControlPoint(
               updatedPlan,
               index,
               updatedPoint,
               &error)) {
            const QString message = QString::fromStdString(error);
            m_widget.setResult(message, false);
            emit statusMessageRequested(message, 5000);
            return;
        }

        if(commitMotionPlanUpdate(updatedPlan, QStringLiteral("motionPlanningEditControlPoint"))) {
            refreshTrajectoryView();
            const QString summary = QStringLiteral("Updated control point %1")
                .arg(pointIndex + 1);
            m_widget.setResult(summary, true);
            emit statusMessageRequested(summary, 4000);
        }
    }

    void MotionPlanningModuleController::startJointPlayback(double durationSeconds)
    {
        if(m_selectedRobotId.isEmpty() || m_selectedTrajectoryId.isEmpty()) {
            m_widget.setResult(QStringLiteral("Select a solved joint trajectory before playback."), false);
            return;
        }
        // Multi-IK playback is an unplanned debug sequence. Do not synchronously
        // build/enable a full collision scene merely to inspect configurations.
        if(!m_multiIkPlayback) { ensurePersistentCdfCollisionSetup(); }

        const std::vector<motion_planning::StoredMotionPlan> plans =
            motion_planning::MotionPlanningProjectStore::plans(m_context.document());
        const motion_planning::StoredMotionPlan* selectedPlan =
            m_multiIkPlayback ? m_multiIkPlayback.get() :
            findMotionPlan(plans, m_selectedRobotId, m_selectedTrajectoryId);
        if(selectedPlan == nullptr || selectedPlan->trajectory.empty()) {
            m_widget.setResult(QStringLiteral("Selected trajectory has no inverse kinematics joint values to play."), false);
            return;
        }

        const int pointCount = static_cast<int>(selectedPlan->trajectory.points.size());
        if(pointCount <= 0) {
            m_widget.setResult(QStringLiteral("Selected trajectory has no inverse kinematics joint values to play."), false);
            return;
        }

        m_playbackCollisionSamples = 0;
        m_playbackCollisionHits = 0;
        m_playbackInvalidSamples = 0;
        m_playbackFinishedNaturally = false;
        m_playbackCollisionScene.reset();

        const motion_planning::ProjectCdfQpRepairOptions cdfOptions;
        if(!m_multiIkPlayback && cdfDetectorIsConfigured(m_context.document(), m_selectedRobotId.toStdString(), cdfOptions)) {
            if(RobotQtViewerViewportServices* viewportServices = m_context.viewportServices()) {
                viewportServices->refreshCollisionConfiguration(
                    m_context.document(),
                    projectBasePath(m_context.projectSession()));
                viewportServices->setCollisionQueriesEnabled(true);
                viewportServices->setCollisionDetectorEnabled(
                    QString::fromStdString(cdfOptions.detectorId),
                    true);
                viewportServices->setActiveCollisionDetector(QString::fromStdString(cdfOptions.detectorId));
            }
        }

        const std::vector<std::string> collisionDetectorIds =
            playbackCollisionDetectorIds(m_context.document(), m_selectedRobotId);
        if(!m_multiIkPlayback && !collisionDetectorIds.empty()) {
            motion_planning::ProjectPlanningRequest validationRequest;
            validationRequest.robotId = selectedPlan->robotId;
            validationRequest.jointNames = selectedPlan->jointNames;
            validationRequest.start = selectedPlan->trajectory.points.front().q;
            validationRequest.goal = selectedPlan->trajectory.points.back().q;
            validationRequest.collisionDetectorIds = collisionDetectorIds;
            validationRequest.validation.maxJointStep = 0.01;

            std::string validationError;
            m_playbackCollisionScene = motion_planning::ProjectPlanningSceneBuilder::build(
                m_context.document(),
                projectBasePath(m_context.projectSession()),
                validationRequest,
                &validationError);
            if(m_playbackCollisionScene == nullptr) {
                emit statusMessageRequested(
                    QStringLiteral("Playback collision checks are unavailable: %1")
                        .arg(QString::fromStdString(validationError)),
                    6000);
            }
        }

        clearEndEffectorTrace();
        setEndEffectorTraceVisible(m_endEffectorTraceVisible);
        m_playbackPointIndex = 0;
        m_spraySamples.clear();
        m_spraySamples.reserve(selectedPlan->trajectory.points.size());
        m_sprayJointNames = selectedPlan->jointNames;
        m_sprayRobotId = m_selectedRobotId;
        m_sprayTrajectoryId = m_selectedTrajectoryId;
        m_sprayPlaybackActive = true;
        m_widget.setSprayRecordingState(false, true, m_sprayExportPending);
        m_widget.setPlaybackActive(true);
        const int intervalMs = pointCount <= 1
            ? 1
            : std::max(1, static_cast<int>(std::round(durationSeconds * 1000.0 / static_cast<double>(pointCount - 1))));
        if(m_playbackTimer != nullptr) {
            m_playbackTimer->setInterval(intervalMs);
        }
        advanceJointPlayback();
        if(m_playbackTimer != nullptr &&
            pointCount > 1 &&
            m_playbackPointIndex > 0 &&
            m_playbackPointIndex < pointCount) {
            m_playbackTimer->start();
        }
    }

    void MotionPlanningModuleController::stopJointPlayback()
    {
        const bool sprayWasActive = m_sprayPlaybackActive;
        m_sprayPlaybackActive = false;
        const bool wasActive = m_playbackTimer != nullptr && m_playbackTimer->isActive();
        if(m_playbackTimer != nullptr) {
            m_playbackTimer->stop();
        }
        m_widget.setPlaybackActive(false);
        if(wasActive || m_playbackFinishedNaturally) {
            m_widget.setResult(playbackCollisionSummary(), true);
        }
        if(sprayWasActive && m_sprayExportPending) {
            m_sprayExportPending = false;
            if(m_playbackFinishedNaturally) {
                QTimer::singleShot(0, this, [this]() {
                    if(!m_sprayPlaybackActive && !m_spraySamples.empty()) {
                        exportSprayMeasurements();
                    }
                });
            } else {
                emit statusMessageRequested(QStringLiteral("Playback interrupted; partial spray samples retained. Export request cancelled."), 6000);
            }
        }
        m_widget.setSprayRecordingState(!m_spraySamples.empty(), false, m_sprayExportPending);
        m_multiIkPlayback.reset();
    }

    void MotionPlanningModuleController::advanceJointPlayback()
    {
        const std::vector<motion_planning::StoredMotionPlan> plans =
            motion_planning::MotionPlanningProjectStore::plans(m_context.document());
        const motion_planning::StoredMotionPlan* selectedPlan =
            m_multiIkPlayback ? m_multiIkPlayback.get() :
            findMotionPlan(plans, m_selectedRobotId, m_selectedTrajectoryId);
        if(selectedPlan == nullptr || selectedPlan->trajectory.empty()) {
            stopJointPlayback();
            m_widget.setResult(QStringLiteral("Playback stopped because the selected trajectory is no longer available."), false);
            return;
        }

        if(m_playbackPointIndex < 0 ||
            m_playbackPointIndex >= static_cast<int>(selectedPlan->trajectory.points.size())) {
            if(m_playbackTimer != nullptr) {
                m_playbackTimer->stop();
            }
            m_widget.setPlaybackActive(false);
            m_widget.setResult(QStringLiteral("Playback finished."), true);
            return;
        }

        const robottrajectory::TimedJointPoint& point =
            selectedPlan->trajectory.points[static_cast<std::size_t>(m_playbackPointIndex)];
        if(!applyJointValuesToRobotRuntime(
               selectedPlan->jointNames,
               point.q,
               QStringLiteral("motionPlanningPlayback"))) {
            stopJointPlayback();
            return;
        }

        // Sample only after every joint in this playback point has been applied.
        if(m_endEffectorTraceVisible) {
            if(auto* services = m_context.viewportServices()) {
                services->appendEndEffectorTraceSample();
            }
        }
        if(m_sprayMeasurementEnabled) {
            SprayMeasurementSample spraySample;
            spraySample.pointIndex = m_playbackPointIndex;
            spraySample.timeSeconds = point.time;
            spraySample.jointValues = point.q;
            spraySample.runtimeJointValues = maybeMapIrb4600JointSigns(
                m_context.document(), m_selectedRobotId, point.q);
            spraySample.measurement = m_currentSprayMeasurement;
            m_spraySamples.push_back(std::move(spraySample));
        }

        ++m_playbackCollisionSamples;
        if(m_playbackCollisionScene != nullptr) {
            const std::vector<double> runtimeJointValues = maybeMapIrb4600JointSigns(
                m_context.document(),
                m_selectedRobotId,
                point.q);
            const motion_planning::StateValidationResult validation =
                m_playbackCollisionScene->validateState(runtimeJointValues);
            if(validation.valid) {
                // Nothing to count.
            } else if(validation.diagnosticCode == "state_in_collision") {
                ++m_playbackCollisionHits;
            } else {
                ++m_playbackInvalidSamples;
            }
        }

        ++m_playbackPointIndex;
        if(m_playbackPointIndex >= static_cast<int>(selectedPlan->trajectory.points.size())) {
            m_playbackFinishedNaturally = true;
            stopJointPlayback();
        }
    }

    void MotionPlanningModuleController::setSelectedTrajectory(const QString& trajectoryId)
    {
        if(m_selectedTrajectoryId == trajectoryId) {
            return;
        }
        invalidateMultiIk();
        stopJointPlayback();
        m_selectedTrajectoryId = trajectoryId;
        clearSprayMeasurements();
        clearEndEffectorTrace();
        refreshTrajectoryView();
    }

    void MotionPlanningModuleController::setSelectedRobot(const QString& robotId)
    {
        if(m_selectedRobotId != robotId) {
            invalidateMultiIk();
            stopJointPlayback();
            clearSprayMeasurements();
            clearEndEffectorTrace();
        }
        m_selectedRobotId = robotId;
        setSprayRangeVisible(m_sprayRangeVisible);
        setEndEffectorTraceVisible(m_endEffectorTraceVisible);
        m_widget.setRobotId(robotId);
        if(robotId.isEmpty()) {
            refreshTrajectoryView();
            return;
        }
        const simulation_project::ProjectDocument& document = m_context.document();
        const auto robotIt = std::find_if(
            document.robots.begin(),
            document.robots.end(),
            [&](const simulation_project::RobotDesc& robot) {
                return robot.id == robotId.toStdString();
            });
        if(robotIt == document.robots.end() || robotIt->initialJoints.empty()) {
            refreshTrajectoryView();
            return;
        }
        QStringList names;
        QStringList values;
        for(const simulation_project::JointValueDesc& joint : robotIt->initialJoints) {
            names.push_back(QString::fromStdString(joint.jointName));
            values.push_back(QString::number(joint.value, 'g', 12));
        }
        m_widget.setJointDefaults(names.join(QStringLiteral(", ")), values.join(QStringLiteral(", ")));
        refreshTrajectoryView();
    }

    void MotionPlanningModuleController::setSprayRangeVisible(bool visible)
    {
        m_sprayRangeVisible = visible;
        if(RobotQtViewerViewportServices* viewportServices = m_context.viewportServices()) {
            viewportServices->setSprayRangeVisible(m_selectedRobotId, visible);
        }
    }

    void MotionPlanningModuleController::setEndEffectorTraceVisible(bool visible)
    {
        m_endEffectorTraceVisible = visible;
        if(auto* services = m_context.viewportServices()) {
            services->setEndEffectorTraceVisible(m_selectedRobotId, visible);
        }
    }

    void MotionPlanningModuleController::clearEndEffectorTrace()
    {
        if(auto* services = m_context.viewportServices()) {
            services->clearEndEffectorTrace();
        }
    }

    void MotionPlanningModuleController::setSprayMeasurementEnabled(bool enabled)
    {
        if(m_sprayMeasurementEnabled == enabled) {
            return;
        }

        m_sprayMeasurementEnabled = enabled;
        if(!enabled) {
            m_currentSprayMeasurement = SprayMeasurementResult{};
            m_spraySamples.clear();
            m_sprayExportPending = false;
        }
        if(enabled) {
            updateSprayMeasurement();
        } else {
            m_widget.setSprayMeasurementText(
                QStringLiteral("Spray distance: -- mm\nSpray angle: -- deg\nCalculation disabled"));
        }
        m_widget.setSprayRecordingState(
            !m_spraySamples.empty(), m_sprayPlaybackActive, m_sprayExportPending);
    }

    void MotionPlanningModuleController::updateSprayMeasurement()
    {
        if(!m_sprayMeasurementEnabled) {
            return;
        }
        const auto* services = m_context.viewportServices();
        m_currentSprayMeasurement = services
            ? services->sprayMeasurement(m_selectedRobotId) : SprayMeasurementResult{};
        const auto& result = m_currentSprayMeasurement;
        m_widget.setSprayMeasurementText(result.valid
            ? QStringLiteral("Spray distance: %1 mm\nSpray angle: %2 deg")
                .arg(result.distanceMeters * 1000.0, 0, 'f', 3)
                .arg(result.angleDegrees, 0, 'f', 3)
            : QStringLiteral("Spray distance: -- mm\nSpray angle: -- deg\n%1").arg(result.errorMessage));
    }

    void MotionPlanningModuleController::clearSprayMeasurements()
    {
        m_spraySamples.clear();
        m_sprayExportPending = false;
        m_sprayPlaybackActive = false;
        m_playbackFinishedNaturally = false;
        m_widget.setSprayRecordingState(false, false, false);
        m_widget.setSprayMeasurementText(m_sprayMeasurementEnabled
            ? QStringLiteral("Spray distance: -- mm\nSpray angle: -- deg")
            : QStringLiteral("Spray distance: -- mm\nSpray angle: -- deg\nCalculation disabled"));
    }

    void MotionPlanningModuleController::exportSprayMeasurements()
    {
        if(!m_sprayMeasurementEnabled) {
            return;
        }
        if(m_sprayPlaybackActive || m_spraySamples.empty()) {
            m_sprayExportPending = true;
            m_widget.setSprayRecordingState(!m_spraySamples.empty(), m_sprayPlaybackActive, true);
            emit statusMessageRequested(QStringLiteral("Spray results will be exported after playback completes."), 5000);
            return;
        }
        QString path = getSaveFileName(QStringLiteral("motionPlanning.sprayMeasurements.save"),
            &m_widget, QStringLiteral("Export spray distance and angle"),
            m_sprayRobotId + QStringLiteral("_spray_measurements.txt"),
            QStringLiteral("Text Files (*.txt);;All Files (*)"));
        if(path.isEmpty()) { return; }
        if(!path.endsWith(QStringLiteral(".txt"), Qt::CaseInsensitive)) { path += QStringLiteral(".txt"); }
        QSaveFile file(path);
        if(!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            m_widget.setResult(file.errorString(), false);
            return;
        }
        QTextStream text(&file);
        text.setCodec("UTF-8");
        text.setLocale(QLocale::c());
        text.setRealNumberNotation(QTextStream::ScientificNotation);
        text.setRealNumberPrecision(10);
        text << "# robot=" << m_sprayRobotId << " trajectory=" << m_sprayTrajectoryId << " target=burnner\n";
        text << "# playback_complete=" << (m_playbackFinishedNaturally ? 1 : 0) << '\n';
        text << "# Angle: 0=normal incidence; +/-90=grazing. STL normal n faces nozzle; sign=(n cross ray) dot nozzle_local_X.\n";
        text << "# time_s is source trajectory time. Joint values are radians (prismatic: meters).\n";
        text << "index\ttime_s";
        for(const auto& name : m_sprayJointNames) { text << '\t' << QString::fromStdString(name) << "_source"; }
        for(const auto& name : m_sprayJointNames) { text << '\t' << QString::fromStdString(name) << "_runtime"; }
        text << "\tdistance_m\tangle_deg\tvalid\tstatus\n";
        for(const auto& sample : m_spraySamples) {
            text << sample.pointIndex + 1 << '\t' << sample.timeSeconds;
            for(double value : sample.jointValues) { text << '\t' << value; }
            for(double value : sample.runtimeJointValues) { text << '\t' << value; }
            if(sample.measurement.valid) {
                text << '\t' << sample.measurement.distanceMeters << '\t' << sample.measurement.angleDegrees << "\t1\tOK\n";
            } else {
                QString reason = sample.measurement.errorMessage;
                reason.replace('\t', ' ').replace('\r', ' ').replace('\n', ' ');
                text << "\tnan\tnan\t0\t" << reason << '\n';
            }
        }
        text.flush();
        if(text.status() != QTextStream::Ok || !file.commit()) {
            m_widget.setResult(QStringLiteral("Spray export failed: %1").arg(file.errorString()), false);
            return;
        }
        m_widget.setResult(QStringLiteral("Exported %1 spray samples to %2").arg(m_spraySamples.size()).arg(path), true);
    }

    void MotionPlanningModuleController::plotSprayMeasurements()
    {
        if(!m_sprayMeasurementEnabled || m_spraySamples.empty() || m_sprayPlaybackActive) { return; }
        QVector<double> distances, angles;
        for(const auto& sample : m_spraySamples) {
            distances.push_back(sample.measurement.valid ? sample.measurement.distanceMeters * 1000.0
                : std::numeric_limits<double>::quiet_NaN());
            angles.push_back(sample.measurement.valid ? sample.measurement.angleDegrees
                : std::numeric_limits<double>::quiet_NaN());
        }
        m_widget.showSprayMeasurementPlot(distances, angles,
            QStringLiteral("Spray measurements - %1 / %2 (%3 points)")
                .arg(m_sprayRobotId, m_sprayTrajectoryId).arg(m_spraySamples.size()));
    }

    void MotionPlanningModuleController::refreshTrajectoryView()
    {
        QVector<MotionPlanningEditorWidget::TrajectoryListItem> items;
        QVector<MotionPlanningEditorWidget::TrajectoryPointRow> poseRows;
        QVector<MotionPlanningEditorWidget::TrajectoryPointRow> jointRows;

        const std::vector<motion_planning::StoredMotionPlan> plans =
            motion_planning::MotionPlanningProjectStore::plans(m_context.document());

        const motion_planning::StoredMotionPlan* selectedPlan = nullptr;
        for(const motion_planning::StoredMotionPlan& plan : plans) {
            if(!m_selectedRobotId.isEmpty() && plan.robotId != m_selectedRobotId.toStdString()) {
                continue;
            }

            MotionPlanningEditorWidget::TrajectoryListItem item;
            item.id = QString::fromStdString(plan.id);
            item.label = QString::fromStdString(plan.name.empty() ? plan.id : plan.name);
            if(!plan.trajectory.empty()) {
                item.kind = !plan.cartesianControlPoints.empty()
                    ? QStringLiteral("joint+cartesian")
                    : QStringLiteral("joint");
                item.pointCount = static_cast<int>(plan.trajectory.points.size());
            } else if(!plan.cartesianControlPoints.empty()) {
                item.kind = QStringLiteral("cartesian");
                item.pointCount = static_cast<int>(plan.cartesianControlPoints.points.size());
            } else {
                continue;
            }
            items.push_back(item);

            if(item.id == m_selectedTrajectoryId) {
                selectedPlan = &plan;
            }
            if(selectedPlan == nullptr && m_selectedTrajectoryId.isEmpty()) {
                selectedPlan = &plan;
                m_selectedTrajectoryId = item.id;
            }
        }

        if(selectedPlan == nullptr && !items.empty()) {
            m_selectedTrajectoryId = items.front().id;
            const std::string selectedId = m_selectedTrajectoryId.toStdString();
            const auto selectedIt = std::find_if(
                plans.begin(),
                plans.end(),
                [&](const motion_planning::StoredMotionPlan& plan) {
                    return plan.id == selectedId;
                });
            if(selectedIt != plans.end()) {
                selectedPlan = &(*selectedIt);
            }
        }
        if(items.empty()) {
            m_selectedTrajectoryId.clear();
        }
        m_widget.setCdfExportAvailable(
            selectedPlan != nullptr && isCdfQpTrajectory(*selectedPlan));

        if(selectedPlan != nullptr && !selectedPlan->cartesianControlPoints.empty()) {
            for(std::size_t index = 0; index < selectedPlan->cartesianControlPoints.points.size(); ++index) {
                const robottrajectory::TimedCartesianPoint& point =
                    selectedPlan->cartesianControlPoints.points[index];
                MotionPlanningEditorWidget::TrajectoryPointRow row;
                row.index = static_cast<int>(index + 1);
                row.timeText = formatDouble(point.time);
                row.valueText = formatCartesianPose(point);
                row.orientationText = formatCartesianEuler(point);
                poseRows.push_back(row);
            }
        }

        if(selectedPlan != nullptr && !selectedPlan->trajectory.empty()) {
            const std::vector<std::string>& jointNames = selectedPlan->jointNames;
            for(std::size_t index = 0; index < selectedPlan->trajectory.points.size(); ++index) {
                const robottrajectory::TimedJointPoint& point =
                    selectedPlan->trajectory.points[index];
                MotionPlanningEditorWidget::TrajectoryPointRow row;
                row.index = static_cast<int>(index + 1);
                row.timeText = formatDouble(point.time);
                row.valueText = formatJointValues(jointNames, point.q);
                row.orientationText.clear();
                jointRows.push_back(row);
            }
        }

        const QString emptyPoseText = items.empty()
            ? QStringLiteral("No stored trajectory for the selected robot.")
            : QStringLiteral("No cartesian control point poses in the selected trajectory.");
        const QString emptyJointText = items.empty()
            ? QStringLiteral("No stored trajectory for the selected robot.")
            : QStringLiteral("No inverse kinematics joint values in the selected trajectory.");
        m_widget.setTrajectoryView(
            items,
            m_selectedTrajectoryId,
            poseRows,
            jointRows,
            emptyPoseText,
            emptyJointText);

        if(RobotQtViewerViewportServices* viewportServices = m_context.viewportServices()) {
            if(selectedPlan != nullptr && !selectedPlan->cartesianControlPoints.empty()) {
                viewportServices->setTrajectoryControlPointOverlay(
                    QString::fromStdString(selectedPlan->id),
                    cartesianControlPointTransforms(*selectedPlan), m_trajectoryPointsVisible);
            } else {
                viewportServices->clearTrajectoryControlPointOverlay();
            }
        }
    }

    void MotionPlanningModuleController::refreshCdfJointAngleView()
    {
        QVector<QString> jointNames;
        jointNames.reserve(static_cast<int>(m_cdfJointNames.size()));
        for(const std::string& jointName : m_cdfJointNames) {
            jointNames.push_back(QString::fromStdString(jointName));
        }

        QVector<MotionPlanningEditorWidget::CdfJointAngleRow> jointRows;
        jointRows.reserve(static_cast<int>(m_cdfJointPoints.size()));

        for(std::size_t index = 0; index < m_cdfJointPoints.size(); ++index) {
            const ImportedCdfJointPoint& point = m_cdfJointPoints[index];
            MotionPlanningEditorWidget::CdfJointAngleRow row;
            row.index = static_cast<int>(index + 1);
            row.timeText = formatDouble(point.timeSeconds);
            row.jointAngleTexts.reserve(static_cast<int>(point.jointAnglesDegrees.size()));
            for(const double angle : point.jointAnglesDegrees) {
                row.jointAngleTexts.push_back(formatDouble(angle));
            }
            jointRows.push_back(row);
        }

        const QString sourceText = m_cdfSourceName.isEmpty()
            ? QStringLiteral("CDF")
            : m_cdfSourceName;
        const QString emptyText = QStringLiteral("No CDF joint angles imported.");
        m_widget.setCdfJointAngleView(sourceText, jointNames, jointRows, emptyText);
    }

    QString MotionPlanningModuleController::playbackCollisionSummary() const
    {
        const QString prefix = m_playbackFinishedNaturally
            ? QStringLiteral("Playback finished.")
            : QStringLiteral("Playback stopped.");
        if(m_playbackCollisionScene == nullptr || m_playbackCollisionSamples <= 0) {
            return prefix + QStringLiteral(" Collision statistics unavailable.");
        }

        const double collisionRate =
            100.0 * static_cast<double>(m_playbackCollisionHits) /
            static_cast<double>(m_playbackCollisionSamples);
        return QStringLiteral("%1 Collision states: %2 / %3 (%4%). Invalid states: %5.")
            .arg(prefix)
            .arg(m_playbackCollisionHits)
            .arg(m_playbackCollisionSamples)
            .arg(QString::number(collisionRate, 'f', 2))
            .arg(m_playbackInvalidSamples);
    }

    bool MotionPlanningModuleController::commitMotionPlanUpdate(
        const motion_planning::StoredMotionPlan& plan,
        const QString& sourceId)
    {
        const ProjectMutationResult mutation = m_context.documentController().mutateProject(
            sourceId,
            ProjectDirtyPolicy::UserEdit,
            [plan](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                if(!motion_planning::MotionPlanningProjectStore::upsertPlan(
                       service.document(),
                       plan,
                       &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutation.success) {
            m_widget.setResult(mutation.message, false);
            emit statusMessageRequested(mutation.message, 6000);
            return false;
        }

        m_selectedTrajectoryId = QString::fromStdString(plan.id);
        return true;
    }

    bool MotionPlanningModuleController::applyJointValuesToRobotRuntime(
        const std::vector<std::string>& jointNames,
        const std::vector<double>& jointValues,
        const QString& sourceId)
    {
        if(m_selectedRobotId.isEmpty()) {
            m_widget.setResult(QStringLiteral("Select a robot before applying joint values."), false);
            return false;
        }
        if(jointNames.size() != jointValues.size() || jointNames.empty()) {
            m_widget.setResult(QStringLiteral("Joint names must match joint values."), false);
            return false;
        }

        RobotQtViewerViewportServices* viewportServices = m_context.viewportServices();
        if(viewportServices == nullptr) {
            m_widget.setResult(QStringLiteral("Viewport is not available."), false);
            return false;
        }

        const std::vector<double> robotJointValues = usesIrb4600RobotSystemJointSigns(
            m_context.document(),
            m_selectedRobotId)
            ? motion_planning::ProjectTrajectoryInverseKinematics::irb4600RobotSystemJointValues(jointValues)
            : jointValues;
        for(std::size_t index = 0; index < jointNames.size(); ++index) {
            viewportServices->setRobotJointValue(
                m_selectedRobotId,
                QString::fromStdString(jointNames[index]),
                robotJointValues[index]);
        }
        m_context.documentController().publishRobotRuntimeChanged(sourceId);
        if(m_sprayMeasurementEnabled) {
            updateSprayMeasurement();
        }
        return true;
    }

    bool MotionPlanningModuleController::applyJointValuesToRobot(
        const std::vector<std::string>& jointNames,
        const std::vector<double>& jointValues,
        const QString& sourceId)
    {
        if(m_selectedRobotId.isEmpty()) {
            m_widget.setResult(QStringLiteral("Select a robot before applying joint values."), false);
            return false;
        }
        if(jointNames.size() != jointValues.size() || jointNames.empty()) {
            m_widget.setResult(QStringLiteral("Joint names must match joint values."), false);
            return false;
        }

        const std::string robotId = m_selectedRobotId.toStdString();
        const std::vector<double> robotJointValues = usesIrb4600RobotSystemJointSigns(
            m_context.document(),
            m_selectedRobotId)
            ? motion_planning::ProjectTrajectoryInverseKinematics::irb4600RobotSystemJointValues(jointValues)
            : jointValues;
        const ProjectMutationResult mutation = m_context.documentController().mutateProject(
            sourceId,
            ProjectDirtyPolicy::UserEdit,
            [&, robotJointValues](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                if(!motion_planning::ProjectTrajectoryInverseKinematics::applyJointValuesToRobotInitialJoints(
                       service.document(),
                       robotId,
                       jointNames,
                       robotJointValues,
                       &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutation.success) {
            m_widget.setResult(mutation.message, false);
            emit statusMessageRequested(mutation.message, 6000);
            return false;
        }

        return applyJointValuesToRobotRuntime(jointNames, jointValues, sourceId);
    }
}

namespace robot_qt_viewer
{

    void MotionPlanningModuleController::invalidateMultiIk()
    {
        if(m_multiIkCancel) { m_multiIkCancel->store(true); }
        if(m_multiIkPlayback) { stopJointPlayback(); }
        m_multiIkResult.reset();
        m_multiIkSelections.clear();
        m_widget.setMultiIkPoints({}, QStringLiteral("\u8bf7\u5bf9\u5f53\u524d\u672b\u7aef\u8f68\u8ff9\u6267\u884c\u5168\u9006\u89e3\u3002"), false);
    }

    void MotionPlanningModuleController::solveMultiIk(bool useTool,
        const QVector<double>& lowerDegrees, const QVector<double>& upperDegrees, int seeds)
    {
        if(m_multiIkThread) { return; }
        const auto plans = motion_planning::MotionPlanningProjectStore::plans(m_context.document());
        const auto* source = findMotionPlan(plans, m_selectedRobotId, m_selectedTrajectoryId);
        if(!source || source->cartesianControlPoints.empty()) {
            m_widget.setResult(QStringLiteral("Import/select cartesian control points first."), false); return;
        }
        motion_planning::CartesianMultiIkOptions options;
        options.model.robotId = m_selectedRobotId.toStdString();
        options.model.jointNames = selectedRobotJointNames(m_context.document(), m_selectedRobotId);
        if(options.model.jointNames.size() != 6 || lowerDegrees.size() != 6 || upperDegrees.size() != 6) {
            m_widget.setResult(QStringLiteral("Multi IK requires six revolute joints and six search ranges."), false); return;
        }
        auto* services = m_context.viewportServices();
        auto fk = services ? services->robotForwardKinematics(m_selectedRobotId, options.model.jointNames, useTool) :
            RobotQtViewerViewportServices::RobotForwardKinematics{};
        if(!fk) { m_widget.setResult(QStringLiteral("Actual robot/TCP FK is unavailable."), false); return; }
        const bool mapSigns = usesIrb4600RobotSystemJointSigns(m_context.document(), m_selectedRobotId);
        for(int j = 0; j < 6; ++j) {
            if(!std::isfinite(lowerDegrees[j]) || !std::isfinite(upperDegrees[j]) || lowerDegrees[j] > upperDegrees[j]) {
                m_widget.setResult(QStringLiteral("Joint search minimum must not exceed maximum."), false); return;
            }
            options.lower.push_back(lowerDegrees[j] * kPi / 180);
            options.upper.push_back(upperDegrees[j] * kPi / 180);
            bool ok = false;
            const double value = services->robotJointValue(m_selectedRobotId,
                QString::fromStdString(options.model.jointNames[j]), &ok);
            options.model.seedJoints.push_back(ok ? value : 0.0);
        }
        if(mapSigns) { options.model.seedJoints = motion_planning::ProjectTrajectoryInverseKinematics::irb4600RobotSystemJointValues(options.model.seedJoints); }
        options.model.worldForwardKinematics = [fk, mapSigns](const auto& q) {
            return fk(mapSigns ? motion_planning::ProjectTrajectoryInverseKinematics::irb4600RobotSystemJointValues(q) : q);
        };
        options.seedCount = seeds;
        stopJointPlayback();
        invalidateMultiIk();
        auto cancel = std::make_shared<std::atomic_bool>(false);
        m_multiIkCancel = cancel;
        options.cancelled = [cancel]() { return cancel->load(); };
        options.progress = [this, cancel](std::size_t done, std::size_t count) {
            QMetaObject::invokeMethod(this, [this, cancel, done, count]() {
                if(!cancel->load()) { m_widget.setMultiIkBusy(true,
                    QStringLiteral("\u5168\u9006\u89e3\u8ba1\u7b97 %1 / %2").arg(static_cast<qulonglong>(done)).arg(static_cast<qulonglong>(count))); }
            }, Qt::QueuedConnection);
        };
        auto output = std::make_shared<motion_planning::CartesianMultiIkResult>();
        const auto document = m_context.document();
        const auto basePath = projectBasePath(m_context.projectSession());
        const auto plan = *source;
        m_widget.setMultiIkBusy(true, QStringLiteral("\u6b63\u5728\u8bfb\u53d6\u6a21\u578b\u9650\u4f4d\u5e76\u641c\u7d22\u591a\u89e3..."));
        m_multiIkThread = QThread::create([document, basePath, plan, options, output, mapSigns, cancel]() mutable {
            try {
                std::vector<double> lower, upper;
                if(cancel->load()) { return; }
                if(!motion_planning::ProjectTrajectoryInverseKinematics::readRevoluteJointLimits(
                    document, basePath, options.model.robotId, options.model.jointNames, lower, upper, output->message)) { return; }
                if(mapSigns) {
                    lower = motion_planning::ProjectTrajectoryInverseKinematics::irb4600RobotSystemJointValues(lower);
                    upper = motion_planning::ProjectTrajectoryInverseKinematics::irb4600RobotSystemJointValues(upper);
                    for(std::size_t j = 0; j < lower.size(); ++j) { if(lower[j] > upper[j]) { std::swap(lower[j], upper[j]); } }
                }
                for(std::size_t j = 0; j < lower.size(); ++j) {
                    options.lower[j] = std::max(options.lower[j], lower[j]);
                    options.upper[j] = std::min(options.upper[j], upper[j]);
                }
                *output = motion_planning::ProjectTrajectoryInverseKinematics::solveAllCartesianControlPoints(plan, options);
            } catch(const std::exception& error) { output->message = error.what(); }
            catch(...) { output->message = "Multi IK failed unexpectedly."; }
        });
        m_multiIkThread->setParent(this);
        connect(m_multiIkThread, &QThread::finished, this, [this, cancel, output]() {
            m_multiIkThread->deleteLater();
            m_multiIkThread = nullptr;
            m_multiIkCancel.reset();
            if(cancel->load()) {
                m_widget.setMultiIkBusy(false, QStringLiteral("\u5168\u9006\u89e3\u5df2\u53d6\u6d88\uff0c\u672a\u4fdd\u7559\u4e0d\u5b8c\u6574\u8f68\u8ff9\u3002")); return;
            }
            m_multiIkResult = std::make_unique<motion_planning::CartesianMultiIkResult>(std::move(*output));
            m_multiIkSelections.assign(m_multiIkResult->layers.size(), 0);
            // Nearest numeric (unwrapped) choice is a debug default, not graph planning.
            for(std::size_t i = 1; i < m_multiIkSelections.size(); ++i) {
                const auto& previous = m_multiIkResult->layers[i - 1].candidates;
                const auto& next = m_multiIkResult->layers[i].candidates;
                if(previous.empty() || next.empty()) { continue; }
                const auto& q = previous[m_multiIkSelections[i - 1]].joints;
                double best = std::numeric_limits<double>::infinity();
                for(std::size_t c = 0; c < next.size(); ++c) {
                    double cost = 0;
                    for(std::size_t j = 0; j < q.size(); ++j) { cost += std::pow(next[c].joints[j] - q[j], 2); }
                    if(cost < best) { best = cost; m_multiIkSelections[i] = c; }
                }
            }
            QVector<MotionPlanningEditorWidget::MultiIkPointRow> rows;
            for(std::size_t i = 0; i < m_multiIkResult->layers.size(); ++i) { rows.push_back({multiIkPointLabel(i)}); }
            const QString summary = QString::fromStdString(m_multiIkResult->message) +
                QStringLiteral("\n\u89e3\u7f16\u53f7\u4ec5\u5728\u5f53\u524d\u70b9\u6709\u6548\uff1b\u89d2\u5ea6\u4e3a\u539f IK \u7b26\u53f7\u3002\u8c03\u8bd5\u64ad\u653e\u672a\u505a\u907f\u969c\u89c4\u5212\u3002");
            m_widget.setMultiIkPoints(rows, summary, m_multiIkResult->success);
            m_widget.setMultiIkBusy(false, summary);
            showMultiIkPoint(0);
        });
        m_multiIkThread->start();
    }

    QString MotionPlanningModuleController::multiIkPointLabel(std::size_t point) const
    {
        const auto& layer = m_multiIkResult->layers[point];
        return QStringLiteral("%1 | t=%2 | %3 \u7ec4 | \u64ad\u653e\u89e3: %4%5")
            .arg(static_cast<qulonglong>(point + 1)).arg(layer.time, 0, 'g', 8)
            .arg(static_cast<qulonglong>(layer.candidates.size()))
            .arg(layer.candidates.empty() ? QStringLiteral("--") : QString::number(m_multiIkSelections[point] + 1))
            .arg(layer.truncated ? QStringLiteral(" [\u5df2\u622a\u65ad]") : QString());
    }

    void MotionPlanningModuleController::showMultiIkPoint(int point)
    {
        QVector<MotionPlanningEditorWidget::MultiIkCandidateRow> rows;
        if(!m_multiIkResult || point < 0 || point >= static_cast<int>(m_multiIkResult->layers.size())) {
            m_widget.setMultiIkCandidates(rows, -1); return;
        }
        const auto& layer = m_multiIkResult->layers[point];
        for(std::size_t c = 0; c < layer.candidates.size(); ++c) {
            const auto& candidate = layer.candidates[c];
            QStringList columns;
            columns << QStringLiteral("%1%2").arg(static_cast<qulonglong>(c + 1))
                .arg(c == m_multiIkSelections[point] ? QStringLiteral(" *") : QString());
            for(double q : candidate.joints) { columns << QString::number(q * 180 / kPi, 'f', 5); }
            QStringList turns;
            for(int turn : candidate.turns) { turns << QString::number(turn); }
            columns << turns.join(QStringLiteral(",")) << QString::number(candidate.positionError * 1000, 'g', 5)
                << QString::number(candidate.orientationError * 180 / kPi, 'g', 5);
            rows.push_back({columns});
        }
        m_widget.setMultiIkCandidates(rows, rows.empty() ? -1 : static_cast<int>(m_multiIkSelections[point]));
    }

    void MotionPlanningModuleController::selectMultiIk(int point, int candidate, bool continueFollowing)
    {
        if(!m_multiIkResult || point < 0 || point >= static_cast<int>(m_multiIkResult->layers.size()) ||
            candidate < 0 || candidate >= static_cast<int>(m_multiIkResult->layers[point].candidates.size())) { return; }
        stopJointPlayback();
        m_multiIkSelections[point] = candidate;
        m_widget.setMultiIkPointLabel(point, multiIkPointLabel(point));
        if(continueFollowing) {
            for(std::size_t i = point + 1; i < m_multiIkResult->layers.size(); ++i) {
                const auto& previous = m_multiIkResult->layers[i - 1].candidates;
                const auto& next = m_multiIkResult->layers[i].candidates;
                if(previous.empty() || next.empty()) { break; }
                const auto& q = previous[m_multiIkSelections[i - 1]].joints;
                double best = std::numeric_limits<double>::infinity();
                for(std::size_t c = 0; c < next.size(); ++c) {
                    double cost = 0;
                    for(std::size_t j = 0; j < q.size(); ++j) { cost += std::pow(next[c].joints[j] - q[j], 2); }
                    if(cost < best) { best = cost; m_multiIkSelections[i] = c; }
                }
                m_widget.setMultiIkPointLabel(static_cast<int>(i), multiIkPointLabel(i));
            }
        }
        showMultiIkPoint(point);
    }

    void MotionPlanningModuleController::applyMultiIk(int point, int candidate)
    {
        if(!m_multiIkResult || point < 0 || point >= static_cast<int>(m_multiIkResult->layers.size()) ||
            candidate < 0 || candidate >= static_cast<int>(m_multiIkResult->layers[point].candidates.size())) { return; }
        stopJointPlayback();
        const auto names = m_multiIkResult->source.jointNames;
        const auto q = m_multiIkResult->layers[point].candidates[candidate].joints;
        applyJointValuesToRobot(names, q, QStringLiteral("motionPlanningMultiIkApply"));
    }

    void MotionPlanningModuleController::playMultiIk(double duration)
    {
        if(!m_multiIkResult) { return; }
        stopJointPlayback();
        auto plan = std::make_unique<motion_planning::StoredMotionPlan>();
        std::string error;
        if(!motion_planning::ProjectTrajectoryInverseKinematics::selectMultiIkTrajectory(
            *m_multiIkResult, m_multiIkSelections, *plan, error)) {
            m_widget.setResult(QString::fromStdString(error), false); return;
        }
        m_multiIkPlayback = std::move(plan);
        startJointPlayback(duration);
    }

}
