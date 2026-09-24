#pragma once

#include <QPointer>
#include <QString>
#include <QStringList>
#include <QVector>
#include <QWidget>

class QDialog;
class QDoubleSpinBox;
class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QTableWidget;
class QTabWidget;
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
        int maxIterations = 1;
        bool keepEndpoints = true;
    };

    explicit MotionPlanningEditorWidget(QWidget* parent = nullptr);

    struct MultiIkPointRow { QString label; };
    struct MultiIkCandidateRow { QStringList columns; };
    void setMultiIkPoints(const QVector<MultiIkPointRow>& points, const QString& summary, bool complete);
    void setMultiIkCandidates(const QVector<MultiIkCandidateRow>& rows, int selected);
    void setMultiIkBusy(bool busy, const QString& message);
    void setMultiIkPointLabel(int point, const QString& label);
    void setLayeredGraphResults(const QVector<QStringList>& rows, const QString& summary);
    void setLayeredGraphPath(const QVector<QStringList>& rows);
    void setLayeredGraphBusy(bool busy, const QString& message);
    void showConfigurationSelection(const QVector<QVector<int>>& sequences, int initialRank);
    void showCdfPage();

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
    void setSprayMeasurementText(const QString& text);
    void setSprayRecordingState(bool hasSamples, bool active, bool exportPending);
    void showSprayMeasurementPlot(const QVector<double>& distancesMm,
        const QVector<double>& anglesDegrees, const QString& title);

signals:
    void planRequested(
        const QString& startJoints,
        const QString& goalJoints,
        const QString& jointNames,
        double duration,
        int sampleCount);
    void importTrajectoryRequested();
    void multiIkRequested(bool useTool, const QVector<double>& lowerDegrees,
        const QVector<double>& upperDegrees, int seeds);
    void multiIkCancelRequested();
    void multiIkPointChanged(int point);
    void multiIkApplyRequested(int point, int candidate);
    void multiIkSelectRequested(int point, int candidate, bool continueFollowing);
    void multiIkPlaybackRequested(double duration);
    void multiIkTargetChanged();
    void layeredGraphRequested(int maxPaths, const QVector<double>& weights);
    void layeredGraphCancelRequested();
    void layeredGraphSettingsChanged();
    void layeredGraphSelectionChanged(int row);
    void configurationSelectionRequested(int row);
    void useLayeredGraphResultRequested(int row);
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
    void exportJointTrajectoryRequested();
    void trajectoryPointsVisibilityChanged(bool visible);
    void sprayRangeVisibilityChanged(bool visible);
    void endEffectorTraceVisibilityChanged(bool visible);
    void sprayMeasurementEnabledChanged(bool enabled);
    void exportSprayMeasurementsRequested();
    void plotSprayMeasurementsRequested();
    void trajectorySelectionChanged(const QString& trajectoryId);

private:
    void updateTrajectoryActions();
    void updateCdfActions();
    void showControlPointContextMenu(const QPoint& pos);

    QTabWidget* m_tabs = nullptr;
    QSpinBox* m_graphMaxPaths = nullptr;
    QLineEdit* m_graphWeights = nullptr;
    QPushButton* m_graphFilter = nullptr;
    QPushButton* m_graphUse = nullptr;
    QPushButton* m_graphView = nullptr;
    QPointer<QDialog> m_configurationDialog;
    QLabel* m_graphStatus = nullptr;
    QTableWidget* m_graphResults = nullptr;
    QTableWidget* m_graphPath = nullptr;
    bool m_graphBusy = false;
    QPushButton* m_allIkButton = nullptr;
    QLabel* m_multiIkStatus = nullptr;
    QComboBox* m_multiIkPoint = nullptr;
    QTableWidget* m_multiIkTable = nullptr;
    QPushButton* m_multiIkApply = nullptr;
    QPushButton* m_multiIkSelect = nullptr;
    QPushButton* m_multiIkContinue = nullptr;
    QPushButton* m_multiIkPlay = nullptr;
    QDoubleSpinBox* m_multiIkDuration = nullptr;
    bool m_multiIkBusy = false;
    bool m_multiIkComplete = false;
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
    QPushButton* m_exportJointTrajectoryButton = nullptr;
    QCheckBox* m_trajectoryPointsVisible = nullptr;
    QPushButton* m_applyJointPointButton = nullptr;
    QDoubleSpinBox* m_playbackDuration = nullptr;
    QPushButton* m_playbackButton = nullptr;
    QCheckBox* m_sprayRangeVisible = nullptr;
    QCheckBox* m_endEffectorTraceVisible = nullptr;
    QCheckBox* m_sprayMeasurementEnabled = nullptr;
    QLabel* m_sprayMeasurement = nullptr;
    QPushButton* m_exportSprayMeasurements = nullptr;
    QPushButton* m_plotSprayMeasurements = nullptr;
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
