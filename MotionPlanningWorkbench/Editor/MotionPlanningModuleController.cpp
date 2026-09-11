#include "MotionPlanningModuleController.h"

#include "MotionPlanningEditorWidget.h"
#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerDocumentController.h"
#include "RobotQtViewerSelectionModel.h"
#include "RobotQtViewerViewportServices.h"

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
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <filesystem>
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
                ? QStringLiteral("Stored repaired OMPL + CDF/QP trajectory")
                : QStringLiteral("Stored partial OMPL + CDF/QP trajectory for inspection"))
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
        connect(&m_widget, &MotionPlanningEditorWidget::inverseKinematicsRequested,
            this, &MotionPlanningModuleController::solveInverseKinematics);
        connect(&m_widget, &MotionPlanningEditorWidget::applySelectedJointPointRequested,
            this, &MotionPlanningModuleController::applySelectedJointPoint);
        connect(&m_widget, &MotionPlanningEditorWidget::applySelectedCdfJointAnglesRequested,
            this, &MotionPlanningModuleController::applySelectedCdfJointAngles);
        connect(&m_widget, &MotionPlanningEditorWidget::repairImportedCdfTrajectoryRequested,
            this, &MotionPlanningModuleController::repairImportedCdfTrajectory);
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
        connect(&m_widget, &MotionPlanningEditorWidget::trajectorySelectionChanged,
            this, &MotionPlanningModuleController::setSelectedTrajectory);
        connect(m_playbackTimer, &QTimer::timeout,
            this, &MotionPlanningModuleController::advanceJointPlayback);
        setSelectedRobot(m_context.selectionModel().state().robotId);
        ensurePersistentCdfCollisionSetup();
        refreshTrajectoryView();
    }

    MotionPlanningModuleController::~MotionPlanningModuleController() = default;

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
        if(event.kind == RobotQtViewerEventKind::SelectionChanged) {
            setSelectedRobot(event.selection.robotId);
        } else if(event.kind == RobotQtViewerEventKind::ProjectOpened) {
            setSelectedRobot(m_context.selectionModel().state().robotId);
            ensurePersistentCdfCollisionSetup();
            refreshTrajectoryView();
        } else if(event.kind == RobotQtViewerEventKind::ProjectDocumentChanged) {
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
        options.stepSize = 0.01;
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
            const QString message = firstDiagnosticMessage(
                repairResult.diagnostics,
                QStringLiteral("CDF/QP repair failed before producing a trajectory."));
            m_widget.setCdfResult(message, false);
            emit statusMessageRequested(message, 7000);
            return;
        }

        motion_planning::StoredMotionPlan storedPlan = repairResult.plan;
        if(!repairResult.success) {
            storedPlan.id = robotId + "_cdf_qp_partial";
            storedPlan.name = "CDF/QP partial repaired trajectory";
            storedPlan.trajectory.name = storedPlan.id;
        }
        for(robottrajectory::TimedJointPoint& point : storedPlan.trajectory.points) {
            point.q = maybeMapIrb4600JointSigns(m_context.document(), m_selectedRobotId, point.q);
        }

        if(!commitMotionPlanUpdate(storedPlan, QStringLiteral("motionPlanningCdfQpRepair"))) {
            m_widget.setCdfResult(QStringLiteral("CDF/QP repair produced a trajectory but storing it failed."), false);
            return;
        }

        m_cdfSourceName = QStringLiteral("%1 repaired by CDF/QP").arg(m_cdfSourceName.isEmpty()
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
            : QStringLiteral(" %1").arg(firstDiagnosticMessage(
                repairResult.diagnostics,
                QStringLiteral("The repaired path still violates the requested clearance.")));
        const QString summary = cdfRepairSummary(repairResult, planId) + diagnosticText;
        m_widget.setCdfResult(summary, repairResult.success);
        emit trajectoryPlanned(planId);
        emit statusMessageRequested(summary, repairResult.success ? 6000 : 9000);
    }

    void MotionPlanningModuleController::exportCdfTrajectory()
    {
        if(m_selectedRobotId.isEmpty() || m_selectedTrajectoryId.isEmpty()) {
            m_widget.setCdfResult(QStringLiteral("Run CDF/QP repair before exporting a trajectory."), false);
            return;
        }

        const std::vector<motion_planning::StoredMotionPlan> plans =
            motion_planning::MotionPlanningProjectStore::plans(m_context.document());
        const motion_planning::StoredMotionPlan* selectedPlan =
            findMotionPlan(plans, m_selectedRobotId, m_selectedTrajectoryId);
        if(selectedPlan == nullptr || !isCdfQpTrajectory(*selectedPlan)) {
            m_widget.setCdfResult(
                QStringLiteral("Select a trajectory produced by OMPL + CDF/QP repair before exporting."),
                false);
            return;
        }
        if(selectedPlan->jointNames.size() != selectedPlan->trajectory.points.front().q.size()) {
            m_widget.setCdfResult(
                QStringLiteral("The repaired trajectory joint names do not match its joint values."),
                false);
            return;
        }

        const QString defaultFileName =
            QStringLiteral("%1_ompl_cdf_qp_trajectory.txt").arg(m_selectedRobotId);
        QString outputPath = robot_qt_viewer::getSaveFileName(
            QStringLiteral("motionPlanning.cdfTrajectory.save"),
            &m_widget,
            QStringLiteral("Export OMPL + CDF/QP trajectory"),
            defaultFileName,
            QStringLiteral("Text Files (*.txt);;All Files (*)"));
        if(outputPath.isEmpty()) {
            return;
        }
        if(!outputPath.endsWith(QStringLiteral(".txt"), Qt::CaseInsensitive)) {
            outputPath += QStringLiteral(".txt");
        }

        QSaveFile file(outputPath);
        if(!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            const QString message = QStringLiteral("Failed to open trajectory export file: %1")
                .arg(file.errorString());
            m_widget.setCdfResult(message, false);
            emit statusMessageRequested(message, 6000);
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

        for(const robottrajectory::TimedJointPoint& point : selectedPlan->trajectory.points) {
            if(point.q.size() != selectedPlan->jointNames.size()) {
                const QString message =
                    QStringLiteral("The repaired trajectory contains an invalid joint row.");
                m_widget.setCdfResult(message, false);
                emit statusMessageRequested(message, 6000);
                return;
            }

            text << point.time;
            for(const double jointValueRadians : point.q) {
                text << '\t' << (jointValueRadians * 180.0 / kPi);
            }
            text << '\n';
        }

        if(!file.commit()) {
            const QString message = QStringLiteral("Failed to save trajectory export: %1")
                .arg(file.errorString());
            m_widget.setCdfResult(message, false);
            emit statusMessageRequested(message, 6000);
            return;
        }

        const QString summary = QStringLiteral("Exported %1 optimized joint angle points to %2")
            .arg(static_cast<int>(selectedPlan->trajectory.points.size()))
            .arg(outputPath);
        m_widget.setCdfResult(summary, true);
        emit statusMessageRequested(summary, 6000);
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
        ensurePersistentCdfCollisionSetup();

        const std::vector<motion_planning::StoredMotionPlan> plans =
            motion_planning::MotionPlanningProjectStore::plans(m_context.document());
        const motion_planning::StoredMotionPlan* selectedPlan =
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
        if(cdfDetectorIsConfigured(m_context.document(), m_selectedRobotId.toStdString(), cdfOptions)) {
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
        if(!collisionDetectorIds.empty()) {
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

        m_playbackPointIndex = 0;
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
        const bool wasActive = m_playbackTimer != nullptr && m_playbackTimer->isActive();
        if(m_playbackTimer != nullptr) {
            m_playbackTimer->stop();
        }
        m_widget.setPlaybackActive(false);
        if(wasActive || m_playbackFinishedNaturally) {
            m_widget.setResult(playbackCollisionSummary(), true);
        }
    }

    void MotionPlanningModuleController::advanceJointPlayback()
    {
        const std::vector<motion_planning::StoredMotionPlan> plans =
            motion_planning::MotionPlanningProjectStore::plans(m_context.document());
        const motion_planning::StoredMotionPlan* selectedPlan =
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
        stopJointPlayback();
        m_selectedTrajectoryId = trajectoryId;
        refreshTrajectoryView();
    }

    void MotionPlanningModuleController::setSelectedRobot(const QString& robotId)
    {
        if(m_selectedRobotId != robotId) {
            stopJointPlayback();
        }
        m_selectedRobotId = robotId;
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
                    cartesianControlPointTransforms(*selectedPlan));
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
