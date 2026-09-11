#pragma once

#include "RobotQtViewerEvents.h"

#include <QObject>
#include <QString>

#include <memory>
#include <string>
#include <vector>

class MotionPlanningEditorWidget;
class QTimer;

namespace motion_planning
{
    struct StoredMotionPlan;
    class ProjectPlanningSceneSnapshot;
}

namespace robot_qt_viewer
{
    class RobotQtViewerDocumentContext;

    class MotionPlanningModuleController : public QObject
    {
        Q_OBJECT

    public:
        MotionPlanningModuleController(
            MotionPlanningEditorWidget& widget,
            RobotQtViewerDocumentContext& context,
            QObject* parent = nullptr);
        ~MotionPlanningModuleController() override;

        void handleEvent(const RobotQtViewerEvent& event);

    signals:
        void statusMessageRequested(const QString& message, int timeoutMs);
        void trajectoryPlanned(const QString& trajectoryId);

    private:
        void planTrajectory(
            const QString& startJoints,
            const QString& goalJoints,
            const QString& jointNames,
            double duration,
            int sampleCount);
        void importTrajectory();
        void importCdfJointAngles();
        void solveInverseKinematics(bool useToolTransform);
        void applySelectedJointPoint(int pointIndex);
        void applySelectedCdfJointAngles(int pointIndex);
        void repairImportedCdfTrajectory();
        void exportCdfTrajectory();
        void insertControlPointBefore(int pointIndex);
        void insertControlPointAfter(int pointIndex);
        void deleteControlPoint(int pointIndex);
        void editControlPoint(int pointIndex);
        void startJointPlayback(double durationSeconds);
        void stopJointPlayback();
        void advanceJointPlayback();
        void setSelectedTrajectory(const QString& trajectoryId);
        void setSelectedRobot(const QString& robotId);
        void ensurePersistentCdfCollisionSetup();
        void refreshTrajectoryView();
        void refreshCdfJointAngleView();
        bool commitMotionPlanUpdate(
            const motion_planning::StoredMotionPlan& plan,
            const QString& sourceId);
        bool applyJointValuesToRobotRuntime(
            const std::vector<std::string>& jointNames,
            const std::vector<double>& jointValues,
            const QString& sourceId);
        bool applyJointValuesToRobot(
            const std::vector<std::string>& jointNames,
            const std::vector<double>& jointValues,
            const QString& sourceId);
        QString playbackCollisionSummary() const;

        MotionPlanningEditorWidget& m_widget;
        RobotQtViewerDocumentContext& m_context;
        struct ImportedCdfJointPoint
        {
            double timeSeconds = 0.0;
            std::vector<double> jointAnglesDegrees;
        };
        QString m_selectedRobotId;
        QString m_selectedTrajectoryId;
        QString m_cdfSourceName;
        std::vector<std::string> m_cdfJointNames;
        std::vector<ImportedCdfJointPoint> m_cdfJointPoints;
        QTimer* m_playbackTimer = nullptr;
        int m_playbackPointIndex = 0;
        int m_playbackCollisionSamples = 0;
        int m_playbackCollisionHits = 0;
        int m_playbackInvalidSamples = 0;
        bool m_playbackFinishedNaturally = false;
        std::unique_ptr<motion_planning::ProjectPlanningSceneSnapshot> m_playbackCollisionScene;
    };
}
