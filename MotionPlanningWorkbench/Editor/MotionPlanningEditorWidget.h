#pragma once

#include <QString>
#include <QVector>
#include <QWidget>

class QDoubleSpinBox;
class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QTableWidget;
class QPoint;

class MotionPlanningEditorWidget : public QWidget
{
    Q_OBJECT

public:
    struct TrajectoryListItem
    {
        QString id;
        QString label;
        QString kind;
        int pointCount = 0;
    };

    struct TrajectoryPointRow
    {
        int index = 0;
        QString timeText;
        QString valueText;
        QString orientationText;
    };

    struct ControlPointPoseEditorData
    {
        double time = 0.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double rollDeg = 0.0;
        double pitchDeg = 0.0;
        double yawDeg = 0.0;
    };

    struct CdfJointAngleRow
    {
        int index = 0;
        QString timeText;
        QVector<QString> jointAngleTexts;
    };

    struct CdfQpRepairSettings
    {
        double safetyMargin = 0.01;
        double targetClearance = 0.0;
        double finiteDifferenceStep = 5.0e-4;
        double distanceThreshold = 0.30;
        double trustRegion = 0.012;
        double seedCorridor = 0.06;
        double seedTrackingWeight = 0.55;
        int segmentIntermediateSamples = 2;
        int maxIterations = 8;
        bool keepEndpoints = true;
    };

    explicit MotionPlanningEditorWidget(QWidget* parent = nullptr);

    void setRobotId(const QString& robotId);
    void setJointDefaults(const QString& jointNames, const QString& startJoints);
    void setResult(const QString& summary, bool success);
    void setTrajectoryView(
        const QVector<TrajectoryListItem>& trajectories,
        const QString& selectedTrajectoryId,
        const QVector<TrajectoryPointRow>& posePoints,
        const QVector<TrajectoryPointRow>& jointPoints,
        const QString& emptyPoseText,
        const QString& emptyJointText);
    void setCdfJointAngleView(
        const QString& sourceName,
        const QVector<QString>& jointNames,
        const QVector<CdfJointAngleRow>& jointRows,
        const QString& emptyText);
    void setCdfExportAvailable(bool available);
    void setCdfResult(const QString& summary, bool success);
    CdfQpRepairSettings cdfQpRepairSettings() const;
    bool editControlPointPose(ControlPointPoseEditorData& data, const QString& title);
    void setPlaybackActive(bool active);

signals:
    void planRequested(
        const QString& startJoints,
        const QString& goalJoints,
        const QString& jointNames,
        double duration,
        int sampleCount);
    void importTrajectoryRequested();
    void inverseKinematicsRequested(bool useToolTransform);
    void applySelectedJointPointRequested(int pointIndex);
    void importCdfJointAnglesRequested();
    void applySelectedCdfJointAnglesRequested(int pointIndex);
    void repairImportedCdfTrajectoryRequested();
    void exportCdfTrajectoryRequested();
    void insertControlPointBeforeRequested(int pointIndex);
    void insertControlPointAfterRequested(int pointIndex);
    void deleteControlPointRequested(int pointIndex);
    void editControlPointRequested(int pointIndex);
    void playbackRequested(double durationSeconds);
    void playbackStopRequested();
    void trajectorySelectionChanged(const QString& trajectoryId);

private:
    void updateTrajectoryActions();
    void updateCdfActions();
    void showControlPointContextMenu(const QPoint& pos);

    QLabel* m_robotValue = nullptr;
    QLineEdit* m_startJoints = nullptr;
    QLineEdit* m_goalJoints = nullptr;
    QLineEdit* m_jointNames = nullptr;
    QDoubleSpinBox* m_duration = nullptr;
    QSpinBox* m_sampleCount = nullptr;
    QPushButton* m_planButton = nullptr;
    QPushButton* m_importButton = nullptr;
    QComboBox* m_ikToolMode = nullptr;
    QPushButton* m_solveIkButton = nullptr;
    QPushButton* m_applyJointPointButton = nullptr;
    QDoubleSpinBox* m_playbackDuration = nullptr;
    QPushButton* m_playbackButton = nullptr;
    QComboBox* m_trajectoryCombo = nullptr;
    QTableWidget* m_poseTable = nullptr;
    QTableWidget* m_jointTable = nullptr;
    QPushButton* m_importCdfButton = nullptr;
    QTableWidget* m_cdfJointTable = nullptr;
    QPushButton* m_applyCdfJointButton = nullptr;
    QDoubleSpinBox* m_cdfSafetyMargin = nullptr;
    QDoubleSpinBox* m_cdfTargetClearance = nullptr;
    QDoubleSpinBox* m_cdfFiniteDifferenceStep = nullptr;
    QDoubleSpinBox* m_cdfDistanceThreshold = nullptr;
    QDoubleSpinBox* m_cdfTrustRegion = nullptr;
    QDoubleSpinBox* m_cdfSeedCorridor = nullptr;
    QDoubleSpinBox* m_cdfSeedTrackingWeight = nullptr;
    QSpinBox* m_cdfSegmentIntermediateSamples = nullptr;
    QSpinBox* m_cdfMaxIterations = nullptr;
    QCheckBox* m_cdfKeepEndpoints = nullptr;
    QPushButton* m_repairCdfTrajectoryButton = nullptr;
    QPushButton* m_exportCdfTrajectoryButton = nullptr;
    QLabel* m_cdfResult = nullptr;
    QLabel* m_result = nullptr;
    bool m_playbackActive = false;
    bool m_cdfExportAvailable = false;
};
