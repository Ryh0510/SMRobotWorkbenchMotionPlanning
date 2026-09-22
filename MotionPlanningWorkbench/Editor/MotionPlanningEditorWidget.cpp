#include "MotionPlanningEditorWidget.h"

#include "RobotQtWidgetUtils.h"

#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStyle>
#include <QStringList>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTabWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    constexpr int kOriginalPointIndexRole = Qt::UserRole + 101;
    constexpr int kLargeTrajectoryTablePreviewLimit = 1000;

    class SprayCurveWidget : public QWidget
    {
    public:
        SprayCurveWidget(const QVector<double>& values, const QString& title,
            const QColor& color, QWidget* parent)
            : QWidget(parent), m_values(values), m_title(title), m_color(color)
        {
            setMinimumSize(420, 210);
        }

    protected:
        void paintEvent(QPaintEvent*) override
        {
            QPainter painter(this);
            painter.setRenderHint(QPainter::Antialiasing);
            painter.fillRect(rect(), palette().base());
            const QRectF plot(76, 32, width() - 100, height() - 78);
            double low = std::numeric_limits<double>::max();
            double high = std::numeric_limits<double>::lowest();
            for(double value : m_values) {
                if(std::isfinite(value)) {
                    low = std::min(low, value);
                    high = std::max(high, value);
                }
            }
            const bool hasValid = low <= high;
            if(!hasValid) { low = 0.0; high = 1.0; }
            const double padding = std::max((high - low) * 0.08, 0.1);
            low -= padding;
            high += padding;
            const QColor textColor = palette().text().color();
            painter.setPen(textColor);
            painter.drawText(QRectF(76, 5, plot.width(), 24), Qt::AlignLeft, m_title);
            for(int tick = 0; tick <= 4; ++tick) {
                const double fraction = tick / 4.0;
                const double y = plot.bottom() - fraction * plot.height();
                painter.setPen(palette().mid().color());
                painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
                painter.setPen(textColor);
                painter.drawText(QRectF(0, y - 10, 68, 20), Qt::AlignRight | Qt::AlignVCenter,
                    QString::number(low + fraction * (high - low), 'g', 5));
            }
            const int lastIndex = std::max(0, m_values.size() - 1);
            const int xTicks = std::min(4, lastIndex);
            for(int tick = 0; tick <= xTicks; ++tick) {
                const int index = xTicks == 0 ? 0 : qRound(lastIndex * tick / double(xTicks));
                const double x = plot.left() + plot.width() * index / std::max(1, lastIndex);
                painter.setPen(palette().mid().color());
                painter.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()));
                painter.setPen(textColor);
                painter.drawText(QRectF(x - 30, plot.bottom() + 3, 60, 20), Qt::AlignCenter,
                    QString::number(index + 1));
            }
            painter.setPen(textColor);
            painter.drawRect(plot);
            painter.drawText(QRectF(plot.left(), height() - 24, plot.width(), 22), Qt::AlignCenter,
                QStringLiteral("Trajectory point"));
            if(!hasValid) {
                painter.drawText(plot, Qt::AlignCenter, QStringLiteral("No valid surface intersections"));
                return;
            }
            painter.save();
            painter.setClipRect(plot.adjusted(-3, -3, 3, 3));
            painter.setPen(QPen(m_color, 1.6));
            QPainterPath path;
            bool connected = false;
            for(int index = 0; index < m_values.size(); ++index) {
                if(!std::isfinite(m_values[index])) { connected = false; continue; }
                const QPointF point(plot.left() + plot.width() * index / std::max(1, lastIndex),
                    plot.bottom() - (m_values[index] - low) / (high - low) * plot.height());
                if(connected) { path.lineTo(point); } else { path.moveTo(point); }
                if(m_values.size() <= 200 || !connected || index == lastIndex ||
                    !std::isfinite(m_values[index + 1])) {
                    painter.drawEllipse(point, 2.0, 2.0);
                }
                connected = true;
            }
            painter.drawPath(path);
            painter.restore();
        }

    private:
        QVector<double> m_values;
        QString m_title;
        QColor m_color;
    };

    void configureTrajectoryTable(
        QTableWidget* table,
        const QStringList& headers,
        int minimumHeight)
    {
        table->setColumnCount(headers.size());
        table->setHorizontalHeaderLabels(headers);
        table->verticalHeader()->setVisible(false);
        table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
        table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed);
        for(int column = 2; column < headers.size(); ++column) {
            table->horizontalHeader()->setSectionResizeMode(column, QHeaderView::Stretch);
        }
        table->setColumnWidth(0, 42);
        table->setColumnWidth(1, 56);
        table->setSelectionBehavior(QAbstractItemView::SelectRows);
        table->setSelectionMode(QAbstractItemView::SingleSelection);
        table->setEditTriggers(QAbstractItemView::NoEditTriggers);
        table->setMinimumHeight(minimumHeight);
    }

    void configureCdfTable(
        QTableWidget* table,
        int jointCount,
        int minimumHeight)
    {
        QStringList headers;
        headers << QStringLiteral("#") << QStringLiteral("time_s");
        for(int index = 0; index < jointCount; ++index) {
            headers << QStringLiteral("J%1").arg(index + 1);
        }
        configureTrajectoryTable(table, headers, minimumHeight);
        table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
        for(int column = 3; column < headers.size(); ++column) {
            table->horizontalHeader()->setSectionResizeMode(column, QHeaderView::Stretch);
        }
    }

    QTableWidgetItem* makeReadOnlyItem(const QString& text)
    {
        auto* item = new QTableWidgetItem(text);
        item->setFlags(item->flags() & ~Qt::ItemIsEditable);
        return item;
    }

    QTableWidgetItem* makeNoticeItem(const QString& text)
    {
        QTableWidgetItem* item = makeReadOnlyItem(text);
        item->setFlags((item->flags() & ~Qt::ItemIsSelectable) | Qt::ItemIsEnabled);
        return item;
    }

    void setOriginalPointIndex(QTableWidgetItem* item, int pointIndex)
    {
        if(item != nullptr) {
            item->setData(kOriginalPointIndexRole, pointIndex);
        }
    }

    int originalPointIndexAt(const QTableWidget* table, int row)
    {
        if(table == nullptr || row < 0 || row >= table->rowCount()) {
            return -1;
        }
        const QTableWidgetItem* item = table->item(row, 0);
        if(item == nullptr) {
            return -1;
        }
        bool ok = false;
        const int pointIndex = item->data(kOriginalPointIndexRole).toInt(&ok);
        return ok ? pointIndex : -1;
    }

    int selectedOriginalPointIndex(const QTableWidget* table)
    {
        return table == nullptr ? -1 : originalPointIndexAt(table, table->currentRow());
    }

    bool hasAnyOriginalPointRow(const QTableWidget* table)
    {
        if(table == nullptr) {
            return false;
        }
        for(int row = 0; row < table->rowCount(); ++row) {
            if(originalPointIndexAt(table, row) >= 0) {
                return true;
            }
        }
        return false;
    }

    QVector<int> sampledSourceRows(int pointCount)
    {
        QVector<int> rows;
        if(pointCount <= 0) {
            return rows;
        }
        if(pointCount <= kLargeTrajectoryTablePreviewLimit) {
            rows.reserve(pointCount);
            for(int index = 0; index < pointCount; ++index) {
                rows.push_back(index);
            }
            return rows;
        }

        rows.reserve(kLargeTrajectoryTablePreviewLimit);
        const int previewCount = kLargeTrajectoryTablePreviewLimit;
        for(int displayRow = 0; displayRow < previewCount; ++displayRow) {
            const long long numerator =
                static_cast<long long>(displayRow) * static_cast<long long>(pointCount - 1);
            const int sourceRow = static_cast<int>(
                (numerator + (previewCount - 1) / 2) / static_cast<long long>(previewCount - 1));
            if(rows.empty() || rows.back() != sourceRow) {
                rows.push_back(sourceRow);
            }
        }
        if(!rows.empty()) {
            rows.back() = pointCount - 1;
        }
        return rows;
    }

    QDoubleSpinBox* makePoseSpinBox(
        QWidget* parent,
        double minimum,
        double maximum,
        double value,
        double step,
        const QString& suffix = QString())
    {
        auto* spinBox = new QDoubleSpinBox(parent);
        spinBox->setRange(minimum, maximum);
        spinBox->setDecimals(6);
        spinBox->setSingleStep(step);
        spinBox->setValue(value);
        spinBox->setSuffix(suffix);
        return spinBox;
    }

    void populateTrajectoryTable(
        QTableWidget* table,
        const QVector<MotionPlanningEditorWidget::TrajectoryPointRow>& points,
        const QString& emptyText)
    {
        if(table == nullptr) {
            return;
        }

        const QVector<int> sourceRows = sampledSourceRows(points.size());
        const bool isPreview = sourceRows.size() < points.size();
        const int noticeRowCount = isPreview ? 1 : 0;
        const QSignalBlocker blocker(table);
        table->setUpdatesEnabled(false);
        table->clearContents();
        table->clearSpans();
        const int columnCount = table->columnCount();
        table->setRowCount(sourceRows.size() + noticeRowCount);
        for(int row = 0; row < sourceRows.size(); ++row) {
            const int sourceRow = sourceRows[row];
            const MotionPlanningEditorWidget::TrajectoryPointRow& point = points[sourceRow];
            QTableWidgetItem* indexItem = makeReadOnlyItem(QString::number(point.index));
            setOriginalPointIndex(indexItem, sourceRow);
            table->setItem(row, 0, indexItem);
            table->setItem(row, 1, makeReadOnlyItem(point.timeText));
            table->setItem(row, 2, makeReadOnlyItem(point.valueText));
            if(columnCount > 3) {
                table->setItem(row, 3, makeReadOnlyItem(point.orientationText));
            }
        }

        if(isPreview) {
            const int noticeRow = sourceRows.size();
            table->setSpan(noticeRow, 0, 1, columnCount);
            table->setItem(
                noticeRow,
                0,
                makeNoticeItem(QStringLiteral("Showing %1 sampled rows from %2 total points. Playback and CDF/QP use all points.")
                    .arg(sourceRows.size())
                    .arg(points.size())));
        }

        if(points.empty()) {
            table->setRowCount(1);
            table->setSpan(0, 0, 1, columnCount);
            table->setItem(0, 0, makeReadOnlyItem(emptyText));
        }
        table->resizeColumnToContents(0);
        table->resizeColumnToContents(1);
        if(columnCount > 3) {
            table->resizeColumnToContents(3);
        }
        table->setUpdatesEnabled(true);
    }
}

MotionPlanningEditorWidget::MotionPlanningEditorWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(8, 8, 8, 8);
    rootLayout->setSpacing(8);

    auto* title = new QLabel(QStringLiteral("Motion Planning"), this);
    title->setProperty("panelTitle", true);
    rootLayout->addWidget(title);

    auto* tabs = new QTabWidget(this);
    rootLayout->addWidget(tabs);

    auto* basicPage = new QWidget(tabs);
    auto* layout = new QVBoxLayout(basicPage);
    layout->setContentsMargins(4, 8, 4, 4);
    layout->setSpacing(8);
    tabs->addTab(basicPage, QStringLiteral("Basic Planning"));

    auto* form = new QFormLayout();
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    m_robotValue = new QLabel(QStringLiteral("No robot selected"), this);
    robot_qt_viewer::makeHorizontallyCompressible(m_robotValue);
    form->addRow(QStringLiteral("Robot"), m_robotValue);

    m_startJoints = new QLineEdit(this);
    m_startJoints->setPlaceholderText(QStringLiteral("0, 0, 0"));
    form->addRow(QStringLiteral("Start joints"), m_startJoints);

    m_jointNames = new QLineEdit(this);
    m_jointNames->setPlaceholderText(QStringLiteral("joint_1, joint_2, joint_3"));
    form->addRow(QStringLiteral("Joint names"), m_jointNames);

    m_goalJoints = new QLineEdit(this);
    m_goalJoints->setPlaceholderText(QStringLiteral("0.5, -0.2, 0.8"));
    form->addRow(QStringLiteral("Goal joints"), m_goalJoints);

    m_duration = new QDoubleSpinBox(this);
    m_duration->setRange(0.01, 3600.0);
    m_duration->setValue(5.0);
    m_duration->setSuffix(QStringLiteral(" s"));
    form->addRow(QStringLiteral("Duration"), m_duration);

    m_sampleCount = new QSpinBox(this);
    m_sampleCount->setRange(2, 10000);
    m_sampleCount->setValue(50);
    form->addRow(QStringLiteral("Samples"), m_sampleCount);

    layout->addLayout(form);

    m_planButton = new QPushButton(QStringLiteral("Plan trajectory"), this);
    layout->addWidget(m_planButton);

    auto* controlPointTitle = new QLabel(QStringLiteral("Trajectory Control Points"), this);
    controlPointTitle->setProperty("panelTitle", true);
    layout->addWidget(controlPointTitle);

    m_importButton = new QPushButton(QStringLiteral("Import trajectory..."), this);
    layout->addWidget(m_importButton);

    auto* ikForm = new QFormLayout();
    ikForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    m_ikToolMode = new QComboBox(this);
    m_ikToolMode->addItem(QStringLiteral("With tool TCP"), true);
    m_ikToolMode->addItem(QStringLiteral("Robot flange"), false);
    ikForm->addRow(QStringLiteral("IK target"), m_ikToolMode);
    layout->addLayout(ikForm);

    m_solveIkButton = new QPushButton(QStringLiteral("Solve IK and apply"), this);
    layout->addWidget(m_solveIkButton);

    m_trajectoryCombo = new QComboBox(this);
    robot_qt_viewer::makeHorizontallyCompressible(m_trajectoryCombo);
    layout->addWidget(m_trajectoryCombo);

    auto* poseTitle = new QLabel(QStringLiteral("Control Point Poses"), this);
    poseTitle->setProperty("panelTitle", true);
    layout->addWidget(poseTitle);

    m_poseTable = new QTableWidget(this);
    configureTrajectoryTable(m_poseTable, {
        QStringLiteral("#"),
        QStringLiteral("t"),
        QStringLiteral("Position"),
        QStringLiteral("Euler (deg)")
    }, 145);
    m_poseTable->setContextMenuPolicy(Qt::CustomContextMenu);
    layout->addWidget(m_poseTable);

    auto* jointTitle = new QLabel(QStringLiteral("IK Joint Values"), this);
    jointTitle->setProperty("panelTitle", true);
    layout->addWidget(jointTitle);

    m_jointTable = new QTableWidget(this);
    configureTrajectoryTable(m_jointTable, {
        QStringLiteral("#"),
        QStringLiteral("t"),
        QStringLiteral("Joint values")
    }, 145);
    layout->addWidget(m_jointTable);

    m_applyJointPointButton = new QPushButton(QStringLiteral("Apply selected joint point"), this);
    layout->addWidget(m_applyJointPointButton);

    auto* playbackLayout = new QHBoxLayout();
    playbackLayout->setSpacing(6);
    m_playbackDuration = new QDoubleSpinBox(this);
    m_playbackDuration->setRange(0.1, 3600.0);
    m_playbackDuration->setValue(5.0);
    m_playbackDuration->setSuffix(QStringLiteral(" s"));
    m_playbackButton = new QPushButton(QStringLiteral("Play IK result"), this);
    playbackLayout->addWidget(m_playbackDuration);
    playbackLayout->addWidget(m_playbackButton);
    layout->addLayout(playbackLayout);

    m_sprayRangeVisible = new QCheckBox(QStringLiteral("Show spray range"), basicPage);
    layout->addWidget(m_sprayRangeVisible);

    m_endEffectorTraceVisible = new QCheckBox(
        QStringLiteral("\u663e\u793a\u672b\u7aef\u8f68\u8ff9"), basicPage);
    m_endEffectorTraceVisible->setObjectName(QStringLiteral("endEffectorTraceVisible"));
    m_endEffectorTraceVisible->setToolTip(QStringLiteral(
        "Trace the spray cone tip during playback. Restarting playback or unchecking clears the trace."));
    layout->addWidget(m_endEffectorTraceVisible);

    m_sprayMeasurementEnabled = new QCheckBox(
        QStringLiteral("Calculate spray distance and angle"), basicPage);
    m_sprayMeasurementEnabled->setObjectName(QStringLiteral("sprayMeasurementEnabled"));
    m_sprayMeasurementEnabled->setChecked(true);
    layout->addWidget(m_sprayMeasurementEnabled);

    m_sprayMeasurement = new QLabel(QStringLiteral("Spray distance: -- mm\nSpray angle: -- deg"), basicPage);
    m_sprayMeasurement->setObjectName(QStringLiteral("sprayMeasurementResult"));
    m_sprayMeasurement->setWordWrap(true);
    layout->addWidget(m_sprayMeasurement);
    m_exportSprayMeasurements = new QPushButton(
        QStringLiteral("\u5bfc\u51fa\u55b7\u6d82\u8ddd\u79bb\u548c\u55b7\u6d82\u89d2\u5ea6"), basicPage);
    m_exportSprayMeasurements->setObjectName(QStringLiteral("exportSprayMeasurements"));
    m_plotSprayMeasurements = new QPushButton(
        QStringLiteral("\u7ed8\u5236\u55b7\u6d82\u8ddd\u79bb\u548c\u55b7\u6d82\u89d2\u5ea6\u56fe"), basicPage);
    m_plotSprayMeasurements->setObjectName(QStringLiteral("plotSprayMeasurements"));
    m_plotSprayMeasurements->setEnabled(false);
    layout->addWidget(m_exportSprayMeasurements);
    layout->addWidget(m_plotSprayMeasurements);

    m_result = new QLabel(QStringLiteral("Select a robot and enter joint vectors."), this);
    m_result->setWordWrap(true);
    layout->addWidget(m_result);
    layout->addStretch(1);

    auto* cdfPage = new QWidget(tabs);
    auto* cdfLayout = new QVBoxLayout(cdfPage);
    cdfLayout->setContentsMargins(4, 8, 4, 4);
    cdfLayout->setSpacing(8);
    tabs->addTab(cdfPage, QStringLiteral("CDF"));

    auto* cdfTitle = new QLabel(QStringLiteral("CDF Joint Angles"), cdfPage);
    cdfTitle->setProperty("panelTitle", true);
    cdfLayout->addWidget(cdfTitle);

    m_importCdfButton = new QPushButton(QStringLiteral("Import CDF joint angles..."), cdfPage);
    cdfLayout->addWidget(m_importCdfButton);

    m_cdfJointTable = new QTableWidget(cdfPage);
    configureCdfTable(m_cdfJointTable, 6, 320);
    cdfLayout->addWidget(m_cdfJointTable);

    m_applyCdfJointButton = new QPushButton(QStringLiteral("Apply selected CDF joint angles"), cdfPage);
    cdfLayout->addWidget(m_applyCdfJointButton);

    auto* cdfRepairTitle = new QLabel(QStringLiteral("CDF/QP Collision Repair"), cdfPage);
    cdfRepairTitle->setProperty("panelTitle", true);
    cdfLayout->addWidget(cdfRepairTitle);

    auto* cdfRepairForm = new QFormLayout();
    cdfRepairForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    m_cdfSafetyMargin = makePoseSpinBox(cdfPage, 0.0, 1.0, 0.01, 0.001, QStringLiteral(" m"));
    cdfRepairForm->addRow(QStringLiteral("Safety margin"), m_cdfSafetyMargin);

    m_cdfTargetClearance = makePoseSpinBox(cdfPage, 0.0, 1.0, 0.0, 0.001, QStringLiteral(" m"));
    cdfRepairForm->addRow(QStringLiteral("Target clearance"), m_cdfTargetClearance);

    m_cdfFiniteDifferenceStep = makePoseSpinBox(cdfPage, 0.000001, 0.1, 0.0005, 0.0001, QStringLiteral(" rad"));
    cdfRepairForm->addRow(QStringLiteral("Finite diff step"), m_cdfFiniteDifferenceStep);

    m_cdfDistanceThreshold = makePoseSpinBox(cdfPage, 0.001, 100.0, 5.0, 0.01, QStringLiteral(" m"));
    cdfRepairForm->addRow(QStringLiteral("Distance horizon"), m_cdfDistanceThreshold);

    m_cdfTrustRegion = makePoseSpinBox(cdfPage, 0.001, 1.0, 0.02, 0.005, QStringLiteral(" rad"));
    cdfRepairForm->addRow(QStringLiteral("Trust region"), m_cdfTrustRegion);

    m_cdfSeedCorridor = makePoseSpinBox(cdfPage, 0.001, 2.0, 0.10, 0.01, QStringLiteral(" rad"));
    cdfRepairForm->addRow(QStringLiteral("Seed corridor"), m_cdfSeedCorridor);

    m_cdfSeedTrackingWeight = makePoseSpinBox(cdfPage, 0.0, 2.0, 0.40, 0.05);
    cdfRepairForm->addRow(QStringLiteral("Seed tracking"), m_cdfSeedTrackingWeight);

    m_cdfSegmentIntermediateSamples = new QSpinBox(cdfPage);
    m_cdfSegmentIntermediateSamples->setRange(0, 20);
    m_cdfSegmentIntermediateSamples->setValue(1);
    cdfRepairForm->addRow(QStringLiteral("Intermediate samples / segment"), m_cdfSegmentIntermediateSamples);

    m_cdfMaxIterations = new QSpinBox(cdfPage);
    m_cdfMaxIterations->setRange(1, 1000);
    m_cdfMaxIterations->setValue(5);
    cdfRepairForm->addRow(QStringLiteral("Max iterations"), m_cdfMaxIterations);

    m_cdfKeepEndpoints = new QCheckBox(QStringLiteral("Lock endpoints"), cdfPage);
    m_cdfKeepEndpoints->setChecked(true);
    cdfRepairForm->addRow(QString(), m_cdfKeepEndpoints);

    cdfLayout->addLayout(cdfRepairForm);

    m_repairCdfTrajectoryButton = new QPushButton(QStringLiteral("Repair imported trajectory with CDF/QP"), cdfPage);
    cdfLayout->addWidget(m_repairCdfTrajectoryButton);

    m_exportCdfTrajectoryButton = new QPushButton(
        QStringLiteral("Export OMPL + CDF/QP trajectory..."),
        cdfPage);
    cdfLayout->addWidget(m_exportCdfTrajectoryButton);

    m_cdfResult = new QLabel(QStringLiteral("Import a CDF joint angle file."), cdfPage);
    m_cdfResult->setWordWrap(true);
    cdfLayout->addWidget(m_cdfResult);
    cdfLayout->addStretch(1);

    connect(m_planButton, &QPushButton::clicked, this, [this]() {
        emit planRequested(
            m_startJoints->text(),
            m_goalJoints->text(),
            m_jointNames->text(),
            m_duration->value(),
            m_sampleCount->value());
    });
    connect(m_importButton, &QPushButton::clicked,
        this, &MotionPlanningEditorWidget::importTrajectoryRequested);
    connect(m_importCdfButton, &QPushButton::clicked,
        this, &MotionPlanningEditorWidget::importCdfJointAnglesRequested);
    connect(m_solveIkButton, &QPushButton::clicked, this, [this]() {
        const bool useToolTransform = m_ikToolMode != nullptr &&
            m_ikToolMode->currentData().toBool();
        emit inverseKinematicsRequested(useToolTransform);
    });
    connect(m_applyJointPointButton, &QPushButton::clicked, this, [this]() {
        emit applySelectedJointPointRequested(selectedOriginalPointIndex(m_jointTable));
    });
    connect(m_playbackButton, &QPushButton::clicked, this, [this]() {
        if(m_playbackActive) {
            emit playbackStopRequested();
        } else {
            emit playbackRequested(m_playbackDuration != nullptr ? m_playbackDuration->value() : 5.0);
        }
    });
    connect(m_sprayRangeVisible, &QCheckBox::toggled,
        this, &MotionPlanningEditorWidget::sprayRangeVisibilityChanged);
    connect(m_endEffectorTraceVisible, &QCheckBox::toggled,
        this, &MotionPlanningEditorWidget::endEffectorTraceVisibilityChanged);
    connect(m_sprayMeasurementEnabled, &QCheckBox::toggled,
        this, &MotionPlanningEditorWidget::sprayMeasurementEnabledChanged);
    connect(m_exportSprayMeasurements, &QPushButton::clicked,
        this, &MotionPlanningEditorWidget::exportSprayMeasurementsRequested);
    connect(m_plotSprayMeasurements, &QPushButton::clicked,
        this, &MotionPlanningEditorWidget::plotSprayMeasurementsRequested);
    connect(m_applyCdfJointButton, &QPushButton::clicked, this, [this]() {
        emit applySelectedCdfJointAnglesRequested(selectedOriginalPointIndex(m_cdfJointTable));
    });
    connect(m_repairCdfTrajectoryButton, &QPushButton::clicked, this, [this]() {
        emit repairImportedCdfTrajectoryRequested();
    });
    connect(m_exportCdfTrajectoryButton, &QPushButton::clicked, this, [this]() {
        emit exportCdfTrajectoryRequested();
    });
    connect(m_poseTable, &QTableWidget::customContextMenuRequested,
        this, &MotionPlanningEditorWidget::showControlPointContextMenu);
    connect(m_trajectoryCombo, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
        this, [this]() {
            if(m_trajectoryCombo != nullptr) {
                emit trajectorySelectionChanged(m_trajectoryCombo->currentData().toString());
            }
            updateTrajectoryActions();
        });
    connect(m_jointTable, &QTableWidget::currentCellChanged, this, [this]() {
        updateTrajectoryActions();
    });
    connect(m_cdfJointTable, &QTableWidget::currentCellChanged, this, [this]() {
        updateCdfActions();
    });
    setCdfJointAngleView(QString(), {}, {}, QStringLiteral("No CDF joint angles imported."));
    updateTrajectoryActions();
    updateCdfActions();
}

bool MotionPlanningEditorWidget::editControlPointPose(
    ControlPointPoseEditorData& data,
    const QString& title)
{
    QDialog dialog(this);
    dialog.setWindowTitle(title);

    auto* layout = new QFormLayout(&dialog);
    layout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    auto* time = makePoseSpinBox(&dialog, 0.0, 3600.0, data.time, 0.1, QStringLiteral(" s"));
    auto* x = makePoseSpinBox(&dialog, -10000.0, 10000.0, data.x, 0.001, QStringLiteral(" m"));
    auto* y = makePoseSpinBox(&dialog, -10000.0, 10000.0, data.y, 0.001, QStringLiteral(" m"));
    auto* z = makePoseSpinBox(&dialog, -10000.0, 10000.0, data.z, 0.001, QStringLiteral(" m"));
    auto* roll = makePoseSpinBox(&dialog, -360.0, 360.0, data.rollDeg, 1.0, QStringLiteral(" deg"));
    auto* pitch = makePoseSpinBox(&dialog, -360.0, 360.0, data.pitchDeg, 1.0, QStringLiteral(" deg"));
    auto* yaw = makePoseSpinBox(&dialog, -360.0, 360.0, data.yawDeg, 1.0, QStringLiteral(" deg"));

    layout->addRow(QStringLiteral("Time"), time);
    layout->addRow(QStringLiteral("X"), x);
    layout->addRow(QStringLiteral("Y"), y);
    layout->addRow(QStringLiteral("Z"), z);
    layout->addRow(QStringLiteral("Roll"), roll);
    layout->addRow(QStringLiteral("Pitch"), pitch);
    layout->addRow(QStringLiteral("Yaw"), yaw);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
        &dialog);
    layout->addRow(buttons);

    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    if(dialog.exec() != QDialog::Accepted) {
        return false;
    }

    data.time = time->value();
    data.x = x->value();
    data.y = y->value();
    data.z = z->value();
    data.rollDeg = roll->value();
    data.pitchDeg = pitch->value();
    data.yawDeg = yaw->value();
    return true;
}

void MotionPlanningEditorWidget::setPlaybackActive(bool active)
{
    m_playbackActive = active;
    if(m_playbackButton != nullptr) {
        m_playbackButton->setText(active
            ? QStringLiteral("Stop playback")
            : QStringLiteral("Play IK result"));
    }
    updateTrajectoryActions();
}

void MotionPlanningEditorWidget::setSprayMeasurementText(const QString& text)
{
    m_sprayMeasurement->setText(text);
}

void MotionPlanningEditorWidget::setSprayRecordingState(bool hasSamples, bool active, bool exportPending)
{
    m_exportSprayMeasurements->setEnabled(!exportPending);
    m_exportSprayMeasurements->setText(exportPending
        ? QStringLiteral("\u7b49\u5f85\u64ad\u653e\u7ed3\u675f\u540e\u5bfc\u51fa...")
        : QStringLiteral("\u5bfc\u51fa\u55b7\u6d82\u8ddd\u79bb\u548c\u55b7\u6d82\u89d2\u5ea6"));
    m_plotSprayMeasurements->setEnabled(hasSamples && !active);
}

void MotionPlanningEditorWidget::showSprayMeasurementPlot(
    const QVector<double>& distancesMm, const QVector<double>& anglesDegrees, const QString& title)
{
    auto* dialog = new QDialog(this, Qt::Window);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->setWindowTitle(title);
    auto* layout = new QVBoxLayout(dialog);
    layout->addWidget(new SprayCurveWidget(distancesMm, QStringLiteral("Spray distance (mm)"),
        QColor(0, 145, 160), dialog), 1);
    layout->addWidget(new SprayCurveWidget(anglesDegrees, QStringLiteral("Spray angle (deg)"),
        QColor(205, 80, 85), dialog), 1);
    dialog->resize(760, 600);
    dialog->show();
}

void MotionPlanningEditorWidget::setJointDefaults(
    const QString& jointNames,
    const QString& startJoints)
{
    if(m_jointNames->text().trimmed().isEmpty()) {
        m_jointNames->setText(jointNames);
    }
    if(m_startJoints->text().trimmed().isEmpty()) {
        m_startJoints->setText(startJoints);
    }
}

void MotionPlanningEditorWidget::setRobotId(const QString& robotId)
{
    m_robotValue->setText(robotId.isEmpty() ? QStringLiteral("No robot selected") : robotId);
    m_planButton->setEnabled(!robotId.isEmpty());
    if(m_importButton != nullptr) {
        m_importButton->setEnabled(!robotId.isEmpty());
    }
    updateTrajectoryActions();
    updateCdfActions();
}

MotionPlanningEditorWidget::CdfQpRepairSettings MotionPlanningEditorWidget::cdfQpRepairSettings() const
{
    CdfQpRepairSettings settings;
    settings.safetyMargin = m_cdfSafetyMargin != nullptr ? m_cdfSafetyMargin->value() : settings.safetyMargin;
    settings.targetClearance = m_cdfTargetClearance != nullptr ? m_cdfTargetClearance->value() : settings.targetClearance;
    settings.finiteDifferenceStep = m_cdfFiniteDifferenceStep != nullptr
        ? m_cdfFiniteDifferenceStep->value()
        : settings.finiteDifferenceStep;
    settings.distanceThreshold = m_cdfDistanceThreshold != nullptr
        ? m_cdfDistanceThreshold->value()
        : settings.distanceThreshold;
    settings.trustRegion = m_cdfTrustRegion != nullptr ? m_cdfTrustRegion->value() : settings.trustRegion;
    settings.seedCorridor = m_cdfSeedCorridor != nullptr ? m_cdfSeedCorridor->value() : settings.seedCorridor;
    settings.seedTrackingWeight = m_cdfSeedTrackingWeight != nullptr
        ? m_cdfSeedTrackingWeight->value()
        : settings.seedTrackingWeight;
    settings.segmentIntermediateSamples = m_cdfSegmentIntermediateSamples != nullptr
        ? m_cdfSegmentIntermediateSamples->value()
        : settings.segmentIntermediateSamples;
    settings.maxIterations = m_cdfMaxIterations != nullptr ? m_cdfMaxIterations->value() : settings.maxIterations;
    settings.keepEndpoints = m_cdfKeepEndpoints != nullptr ? m_cdfKeepEndpoints->isChecked() : settings.keepEndpoints;
    return settings;
}

void MotionPlanningEditorWidget::setResult(const QString& summary, bool success)
{
    m_result->setText(summary);
    m_result->setProperty("error", !success);
    m_result->style()->unpolish(m_result);
    m_result->style()->polish(m_result);
}

void MotionPlanningEditorWidget::setTrajectoryView(
    const QVector<TrajectoryListItem>& trajectories,
    const QString& selectedTrajectoryId,
    const QVector<TrajectoryPointRow>& posePoints,
    const QVector<TrajectoryPointRow>& jointPoints,
    const QString& emptyPoseText,
    const QString& emptyJointText)
{
    if(m_trajectoryCombo == nullptr || m_poseTable == nullptr || m_jointTable == nullptr) {
        return;
    }

    QSignalBlocker comboBlocker(m_trajectoryCombo);
    m_trajectoryCombo->clear();
    int selectedIndex = -1;
    for(const TrajectoryListItem& trajectory : trajectories) {
        QString label = trajectory.label.isEmpty() ? trajectory.id : trajectory.label;
        if(!trajectory.kind.isEmpty()) {
            label = QStringLiteral("%1 (%2, %3 pts)")
                .arg(label)
                .arg(trajectory.kind)
                .arg(trajectory.pointCount);
        }
        m_trajectoryCombo->addItem(label, trajectory.id);
        m_trajectoryCombo->setItemData(
            m_trajectoryCombo->count() - 1,
            trajectory.kind,
            Qt::UserRole + 1);
        if(trajectory.id == selectedTrajectoryId) {
            selectedIndex = m_trajectoryCombo->count() - 1;
        }
    }
    if(selectedIndex < 0 && m_trajectoryCombo->count() > 0) {
        selectedIndex = 0;
    }
    if(selectedIndex >= 0) {
        m_trajectoryCombo->setCurrentIndex(selectedIndex);
    }
    m_trajectoryCombo->setEnabled(m_trajectoryCombo->count() > 0);

    populateTrajectoryTable(m_poseTable, posePoints, emptyPoseText);
    populateTrajectoryTable(m_jointTable, jointPoints, emptyJointText);
    updateTrajectoryActions();
}

void MotionPlanningEditorWidget::setCdfJointAngleView(
    const QString& sourceName,
    const QVector<QString>& jointNames,
    const QVector<CdfJointAngleRow>& jointRows,
    const QString& emptyText)
{
    if(m_cdfJointTable == nullptr) {
        return;
    }

    const QString displayEmptyText = sourceName.isEmpty()
        ? emptyText
        : QStringLiteral("%1: %2").arg(sourceName, emptyText);

    QStringList headers;
    headers << QStringLiteral("#") << QStringLiteral("time_s");
    for(int index = 0; index < jointNames.size(); ++index) {
        headers << QStringLiteral("%1 (deg)").arg(jointNames[index]);
    }
    if(headers.size() <= 2) {
        for(int index = 0; index < 6; ++index) {
            headers << QStringLiteral("J%1 (deg)").arg(index + 1);
        }
    }
    configureTrajectoryTable(m_cdfJointTable, headers, 320);

    const QVector<int> sourceRows = sampledSourceRows(jointRows.size());
    const bool isPreview = sourceRows.size() < jointRows.size();
    const int noticeRowCount = isPreview ? 1 : 0;
    const QSignalBlocker blocker(m_cdfJointTable);
    m_cdfJointTable->setUpdatesEnabled(false);
    m_cdfJointTable->clearContents();
    m_cdfJointTable->clearSpans();
    m_cdfJointTable->setRowCount(sourceRows.size() + noticeRowCount);
    for(int row = 0; row < sourceRows.size(); ++row) {
        const int sourceRow = sourceRows[row];
        const CdfJointAngleRow& point = jointRows[sourceRow];
        QTableWidgetItem* indexItem = makeReadOnlyItem(QString::number(point.index));
        setOriginalPointIndex(indexItem, sourceRow);
        m_cdfJointTable->setItem(row, 0, indexItem);
        m_cdfJointTable->setItem(row, 1, makeReadOnlyItem(point.timeText));
        for(int jointIndex = 0; jointIndex < point.jointAngleTexts.size(); ++jointIndex) {
            m_cdfJointTable->setItem(row, jointIndex + 2, makeReadOnlyItem(point.jointAngleTexts[jointIndex]));
        }
    }

    if(isPreview) {
        const int noticeRow = sourceRows.size();
        m_cdfJointTable->setSpan(noticeRow, 0, 1, m_cdfJointTable->columnCount());
        m_cdfJointTable->setItem(
            noticeRow,
            0,
            makeNoticeItem(QStringLiteral("Showing %1 sampled rows from %2 total points. Apply selected row maps to the original point.")
                .arg(sourceRows.size())
                .arg(jointRows.size())));
    }

    if(jointRows.empty()) {
        m_cdfJointTable->setRowCount(1);
        m_cdfJointTable->setSpan(0, 0, 1, m_cdfJointTable->columnCount());
        m_cdfJointTable->setItem(0, 0, makeReadOnlyItem(displayEmptyText));
    }
    m_cdfJointTable->resizeColumnToContents(0);
    m_cdfJointTable->resizeColumnToContents(1);
    m_cdfJointTable->setUpdatesEnabled(true);
    updateCdfActions();
}

void MotionPlanningEditorWidget::setCdfExportAvailable(bool available)
{
    m_cdfExportAvailable = available;
    updateCdfActions();
}

void MotionPlanningEditorWidget::setCdfResult(const QString& summary, bool success)
{
    if(m_cdfResult == nullptr) {
        return;
    }
    m_cdfResult->setText(summary);
    m_cdfResult->setProperty("error", !success);
    m_cdfResult->style()->unpolish(m_cdfResult);
    m_cdfResult->style()->polish(m_cdfResult);
}

void MotionPlanningEditorWidget::updateTrajectoryActions()
{
    const bool hasRobot = m_robotValue != nullptr &&
        m_robotValue->text() != QStringLiteral("No robot selected");
    QString kind;
    if(m_trajectoryCombo != nullptr && m_trajectoryCombo->currentIndex() >= 0) {
        kind = m_trajectoryCombo->currentData(Qt::UserRole + 1).toString();
    }
    const bool hasCartesian = kind.contains(QStringLiteral("cartesian"));
    const bool hasJoint = kind.contains(QStringLiteral("joint"));
    const bool hasValidJointRow = selectedOriginalPointIndex(m_jointTable) >= 0;
    const bool hasAnyJointRow = hasAnyOriginalPointRow(m_jointTable);

    if(m_ikToolMode != nullptr) {
        m_ikToolMode->setEnabled(hasRobot && hasCartesian);
    }
    if(m_solveIkButton != nullptr) {
        m_solveIkButton->setEnabled(hasRobot && hasCartesian);
    }
    if(m_applyJointPointButton != nullptr) {
        m_applyJointPointButton->setEnabled(hasRobot && hasJoint && hasValidJointRow);
    }
    if(m_playbackDuration != nullptr) {
        m_playbackDuration->setEnabled(!m_playbackActive && hasRobot && hasJoint && hasAnyJointRow);
    }
    if(m_playbackButton != nullptr) {
        m_playbackButton->setEnabled(m_playbackActive || (hasRobot && hasJoint && hasAnyJointRow));
    }
}

void MotionPlanningEditorWidget::updateCdfActions()
{
    const bool hasRobot = m_robotValue != nullptr &&
        m_robotValue->text() != QStringLiteral("No robot selected");
    const bool hasValidRow = selectedOriginalPointIndex(m_cdfJointTable) >= 0;
    const bool hasImportedRows = hasAnyOriginalPointRow(m_cdfJointTable);

    if(m_importCdfButton != nullptr) {
        m_importCdfButton->setEnabled(hasRobot);
    }
    if(m_applyCdfJointButton != nullptr) {
        m_applyCdfJointButton->setEnabled(hasRobot && hasValidRow);
    }
    if(m_cdfSafetyMargin != nullptr) {
        m_cdfSafetyMargin->setEnabled(hasRobot && hasImportedRows);
    }
    if(m_cdfTargetClearance != nullptr) {
        m_cdfTargetClearance->setEnabled(hasRobot && hasImportedRows);
    }
    if(m_cdfFiniteDifferenceStep != nullptr) {
        m_cdfFiniteDifferenceStep->setEnabled(hasRobot && hasImportedRows);
    }
    if(m_cdfDistanceThreshold != nullptr) {
        m_cdfDistanceThreshold->setEnabled(hasRobot && hasImportedRows);
    }
    if(m_cdfTrustRegion != nullptr) {
        m_cdfTrustRegion->setEnabled(hasRobot && hasImportedRows);
    }
    if(m_cdfSeedCorridor != nullptr) {
        m_cdfSeedCorridor->setEnabled(hasRobot && hasImportedRows);
    }
    if(m_cdfSegmentIntermediateSamples != nullptr) {
        m_cdfSegmentIntermediateSamples->setEnabled(hasRobot && hasImportedRows);
    }
    if(m_cdfMaxIterations != nullptr) {
        m_cdfMaxIterations->setEnabled(hasRobot && hasImportedRows);
    }
    if(m_cdfKeepEndpoints != nullptr) {
        m_cdfKeepEndpoints->setEnabled(hasRobot && hasImportedRows);
    }
    if(m_repairCdfTrajectoryButton != nullptr) {
        m_repairCdfTrajectoryButton->setEnabled(hasRobot && hasImportedRows);
    }
    if(m_exportCdfTrajectoryButton != nullptr) {
        m_exportCdfTrajectoryButton->setEnabled(hasRobot && m_cdfExportAvailable);
    }
}

void MotionPlanningEditorWidget::showControlPointContextMenu(const QPoint& pos)
{
    if(m_poseTable == nullptr) {
        return;
    }

    QTableWidgetItem* item = m_poseTable->itemAt(pos);
    if(item == nullptr || m_poseTable->columnSpan(item->row(), 0) > 1) {
        return;
    }

    const int row = item->row();
    const int pointIndex = originalPointIndexAt(m_poseTable, row);
    if(pointIndex < 0) {
        return;
    }
    m_poseTable->setCurrentCell(row, item->column());

    QMenu menu(this);
    QAction* insertBefore = menu.addAction(QStringLiteral("Insert Before"));
    QAction* insertAfter = menu.addAction(QStringLiteral("Insert After"));
    menu.addSeparator();
    QAction* edit = menu.addAction(QStringLiteral("Edit"));
    QAction* remove = menu.addAction(QStringLiteral("Delete"));

    QAction* selectedAction = menu.exec(m_poseTable->viewport()->mapToGlobal(pos));
    if(selectedAction == insertBefore) {
        emit insertControlPointBeforeRequested(pointIndex);
    } else if(selectedAction == insertAfter) {
        emit insertControlPointAfterRequested(pointIndex);
    } else if(selectedAction == edit) {
        emit editControlPointRequested(pointIndex);
    } else if(selectedAction == remove) {
        emit deleteControlPointRequested(pointIndex);
    }
}
