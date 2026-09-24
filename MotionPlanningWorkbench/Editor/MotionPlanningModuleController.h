#pragma once

#include "RobotQtViewerEvents.h"
#include "RobotQtViewerViewportServices.h"

#include <QObject>
#include <QString>

#include <memory>
#include <atomic>
#include <string>
#include <vector>

class MotionPlanningEditorWidget;
class QTimer;
class QThread;

namespace motion_planning
{
    struct StoredMotionPlan;
    struct CartesianMultiIkResult;
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
        void solveMultiIk(bool useTool, const QVector<double>& lowerDegrees,
            const QVector<double>& upperDegrees, int seeds);
        void invalidateMultiIk();
        void showMultiIkPoint(int point);
        void selectMultiIk(int point, int candidate, bool continueFollowing);
        void applyMultiIk(int point, int candidate);
        void playMultiIk(double duration);
        QString multiIkPointLabel(std::size_t point) const;
        void solveInverseKinematics(bool useToolTransform);
        void applySelectedJointPoint(int pointIndex);
        void applySelectedCdfJointAngles(int pointIndex);
        void repairImportedCdfTrajectory();
        void exportCdfTrajectory();
        void exportJointTrajectory(bool cdfOnly);
        void insertControlPointBefore(int pointIndex);
        void insertControlPointAfter(int pointIndex);
        void deleteControlPoint(int pointIndex);
        void editControlPoint(int pointIndex);
        void startJointPlayback(double durationSeconds);
        void stopJointPlayback();
        void advanceJointPlayback();
        void setSprayRangeVisible(bool visible);
        void setEndEffectorTraceVisible(bool visible);
        void clearEndEffectorTrace();
        void setSprayMeasurementEnabled(bool enabled);
        void updateSprayMeasurement();
        void exportSprayMeasurements();
        void plotSprayMeasurements();
        void clearSprayMeasurements();
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
        QThread* m_multiIkThread = nullptr;
        std::shared_ptr<std::atomic_bool> m_multiIkCancel;
        std::unique_ptr<motion_planning::CartesianMultiIkResult> m_multiIkResult;
        std::vector<std::size_t> m_multiIkSelections;
        std::unique_ptr<motion_planning::StoredMotionPlan> m_multiIkPlayback;
        QTimer* m_playbackTimer = nullptr;
        int m_playbackPointIndex = 0;
        int m_playbackCollisionSamples = 0;
        int m_playbackCollisionHits = 0;
        int m_playbackInvalidSamples = 0;
        bool m_playbackFinishedNaturally = false;
        bool m_sprayRangeVisible = false;
        bool m_endEffectorTraceVisible = false;
        bool m_trajectoryPointsVisible = false;
        bool m_sprayMeasurementEnabled = true;
        struct SprayMeasurementSample
        {
            int pointIndex = 0;
            double timeSeconds = 0.0;
            std::vector<double> jointValues;
            std::vector<double> runtimeJointValues;
            SprayMeasurementResult measurement;
        };
        SprayMeasurementResult m_currentSprayMeasurement;
        std::vector<SprayMeasurementSample> m_spraySamples;
        std::vector<std::string> m_sprayJointNames;
        QString m_sprayRobotId;
        QString m_sprayTrajectoryId;
        bool m_sprayPlaybackActive = false;
        bool m_sprayExportPending = false;
        std::unique_ptr<motion_planning::ProjectPlanningSceneSnapshot> m_playbackCollisionScene;
    };
}
