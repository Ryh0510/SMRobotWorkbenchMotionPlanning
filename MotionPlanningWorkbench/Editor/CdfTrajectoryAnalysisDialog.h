#pragma once

#include <QDialog>
#include <QVector>
#include <QStringList>
#include <QPointF>

// A view projection only; domain metrics and FK are calculated in ProjectMotionPlanning.
struct CdfStageViewData
{
    QString name;
    bool timingValid = false;
    QStringList metrics;
    QVector<double> times;
    QVector<QVector<double>> angles, accelerations;
    QVector<QVector<QPointF>> segmentVelocities;
    QVector<QVector<double>> tcpPositions, referenceTcpPositions;
    QVector<double> peakVelocities, peakAccelerations, velocityLimits, accelerationLimits;
};

QStringList cdfQualityMetricLabels();

class CdfTrajectoryAnalysisDialog final : public QDialog
{
    Q_OBJECT
public:
    CdfTrajectoryAnalysisDialog(const QVector<CdfStageViewData>& stages,
        const QStringList& jointNames, const QString& diagnostics, QWidget* parent = nullptr);
signals:
    void exportQualityRequested();
};
