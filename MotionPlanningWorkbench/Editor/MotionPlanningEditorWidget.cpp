#include "MotionPlanningEditorWidget.h"
#include "CdfTrajectoryAnalysisDialog.h"
#include "ConfigurationSelectionDialog.h"

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

    void replaceLayeredGraphRows(QTableWidget* table, const QVector<QStringList>& rows)
    {
        const QSignalBlocker blocker(table);
        const bool updatesEnabled = table->updatesEnabled();
        table->setUpdatesEnabled(false);
        // Disabling painting does not suppress model dataChanged/visualRect.
        // With ResizeToContents, each setItem can rescan the whole column. Keep
        // section geometry stable throughout the batch, then size it once.
        auto* header = table->horizontalHeader();
        header->setSectionResizeMode(QHeaderView::Interactive);
        table->clearContents();
        table->setRowCount(rows.size());
        for(int row = 0; row < rows.size(); ++row) {
            for(int column = 0; column < rows[row].size(); ++column) {
                table->setItem(row, column, makeReadOnlyItem(rows[row][column]));
            }
        }
        header->setSectionResizeMode(QHeaderView::ResizeToContents);
        header->resizeSections(QHeaderView::ResizeToContents);
        table->setUpdatesEnabled(updatesEnabled);
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
    m_tabs = tabs;
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

    m_trajectoryPointsVisible = new QCheckBox(
        QStringLiteral("\u663e\u793a\u8f68\u8ff9\u70b9"), basicPage);
    m_trajectoryPointsVisible->setObjectName(QStringLiteral("trajectoryPointsVisible"));
    layout->addWidget(m_trajectoryPointsVisible);

    auto* ikForm = new QFormLayout();
    ikForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    m_ikToolMode = new QComboBox(this);
    m_ikToolMode->addItem(QStringLiteral("With tool TCP"), true);
    m_ikToolMode->addItem(QStringLiteral("Robot flange"), false);
    ikForm->addRow(QStringLiteral("IK target"), m_ikToolMode);
    layout->addLayout(ikForm);

    m_solveIkButton = new QPushButton(QStringLiteral("Solve IK and apply"), this);
    layout->addWidget(m_solveIkButton);

    m_allIkButton = new QPushButton(QStringLiteral("\u5168\u9006\u89e3"), basicPage);
    m_allIkButton->setObjectName(QStringLiteral("solveAllIk"));
    layout->addWidget(m_allIkButton);
    connect(m_allIkButton, &QPushButton::clicked, this, [this]() {
        if(m_multiIkBusy) { emit multiIkCancelRequested(); return; }
        QDialog dialog(this);
        dialog.setWindowTitle(QStringLiteral("\u5168\u9006\u89e3 - \u641c\u7d22\u8bbe\u7f6e"));
        auto* form = new QFormLayout(&dialog);
        auto* note = new QLabel(QStringLiteral(
            "\u591a\u521d\u503c\u6570\u503c\u641c\u7d22\uff0c\u4e0d\u4fdd\u8bc1\u627e\u5168\u6240\u6709\u5206\u652f\u3002\n"
            "\u5404\u8f74\u8303\u56f4\u5355\u4f4d\u4e3a\u5ea6\uff0c\u4e0e\u6a21\u578b\u5df2\u77e5\u9650\u4f4d\u53d6\u4ea4\u96c6\u3002\n"
            "continuous \u8f74\u7684\u641c\u7d22\u7a97\u53e3\u4e0d\u4ee3\u8868\u771f\u5b9e\u673a\u68b0\u9650\u4f4d\u3002\n"
            "\u6269\u5927\u8303\u56f4\u53ef\u679a\u4e3e\u591a\u5708\u89e3\uff1b\u6bcf\u70b9\u6700\u591a\u4fdd\u7559 512 \u7ec4\u3002"), &dialog);
        form->addRow(note);
        QVector<QDoubleSpinBox*> lows, highs;
        for(int j = 0; j < 6; ++j) {
            auto* row = new QHBoxLayout;
            auto* low = new QDoubleSpinBox(&dialog);
            auto* high = new QDoubleSpinBox(&dialog);
            for(auto* box : {low, high}) { box->setRange(-3600, 3600); box->setDecimals(2); row->addWidget(box); }
            low->setValue(-180); high->setValue(180);
            lows.push_back(low); highs.push_back(high);
            form->addRow(QStringLiteral("J%1 min / max (deg)").arg(j + 1), row);
        }
        auto* seeds = new QSpinBox(&dialog);
        seeds->setRange(8, 1024); seeds->setValue(64);
        form->addRow(QStringLiteral("\u6bcf\u70b9\u5206\u6563\u521d\u503c\u6570"), seeds);
        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
        form->addRow(buttons);
        connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
        if(dialog.exec() != QDialog::Accepted) { return; }
        QVector<double> lower, upper;
        for(int j = 0; j < 6; ++j) { lower.push_back(lows[j]->value()); upper.push_back(highs[j]->value()); }
        emit multiIkRequested(m_ikToolMode->currentData().toBool(), lower, upper, seeds->value());
    });
    connect(m_ikToolMode, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, [this](int) { emit multiIkTargetChanged(); });

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

    m_exportJointTrajectoryButton = new QPushButton(
        QStringLiteral("\u5bfc\u51fa\u5173\u8282\u8f68\u8ff9"), basicPage);
    m_exportJointTrajectoryButton->setObjectName(QStringLiteral("exportJointTrajectory"));
    layout->addWidget(m_exportJointTrajectoryButton);

    auto* playbackLayout = new QHBoxLayout();
    playbackLayout->setSpacing(6);
    m_playbackDuration = new QDoubleSpinBox(this);
    m_playbackDuration->setRange(0.1, 3600.0);
    m_playbackDuration->setValue(5.0);
    m_playbackDuration->setSuffix(QStringLiteral(" s"));
    m_playbackDuration->setToolTip(QStringLiteral("Preview duration. CDF results use uniform joint-path speed; original trajectory times and exports are unchanged. Playback waits for each displayed frame. Slow rendering or sampling extends the preview; hiding the viewport pauses progress."));
    m_playbackButton = new QPushButton(QStringLiteral("Play IK result"), this);
    playbackLayout->addWidget(m_playbackDuration);
    playbackLayout->addWidget(m_playbackButton);
    layout->addLayout(playbackLayout);

    auto* multiTitle = new QLabel(QStringLiteral("\u591a\u89e3\u5173\u8282\u8f68\u8ff9"), basicPage);
    multiTitle->setProperty("panelTitle", true);
    layout->addWidget(multiTitle);
    m_multiIkStatus = new QLabel(QStringLiteral("\u70b9\u51fb\u5168\u9006\u89e3\u751f\u6210\u5019\u9009\u3002"), basicPage);
    m_multiIkStatus->setObjectName(QStringLiteral("multiIkStatus"));
    m_multiIkStatus->setWordWrap(true);
    layout->addWidget(m_multiIkStatus);
    m_multiIkPoint = new QComboBox(basicPage);
    m_multiIkPoint->setObjectName(QStringLiteral("multiIkPoint"));
    robot_qt_viewer::makeHorizontallyCompressible(m_multiIkPoint);
    layout->addWidget(m_multiIkPoint);
    m_multiIkTable = new QTableWidget(basicPage);
    m_multiIkTable->setObjectName(QStringLiteral("multiIkCandidates"));
    configureTrajectoryTable(m_multiIkTable, {QStringLiteral("\u56fa\u5b9a\u6784\u578b / \u5019\u9009 / \u64ad\u653e"),
        QStringLiteral("J1 deg"), QStringLiteral("J2 deg"), QStringLiteral("J3 deg"),
        QStringLiteral("J4 deg"), QStringLiteral("J5 deg"), QStringLiteral("J6 deg"),
        QStringLiteral("turn J1..J6"), QStringLiteral("error mm"), QStringLiteral("error deg")}, 150);
    m_multiIkTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    layout->addWidget(m_multiIkTable);
    m_multiIkApply = new QPushButton(QStringLiteral("\u5355\u70b9\u5e94\u7528\u6240\u9009\u89e3"), basicPage);
    m_multiIkApply->setObjectName(QStringLiteral("multiIkApply"));
    m_multiIkSelect = new QPushButton(QStringLiteral("\u8bbe\u4e3a\u8be5\u70b9\u64ad\u653e\u89e3"), basicPage);
    m_multiIkContinue = new QPushButton(QStringLiteral("\u4ece\u6240\u9009\u89e3\u5c31\u8fd1\u9009\u53d6\u540e\u7eed\u89e3"), basicPage);
    layout->addWidget(m_multiIkApply);
    layout->addWidget(m_multiIkSelect);
    layout->addWidget(m_multiIkContinue);
    auto* multiPlayback = new QHBoxLayout;
    m_multiIkDuration = new QDoubleSpinBox(basicPage);
    m_multiIkDuration->setRange(0.1, 3600); m_multiIkDuration->setValue(5); m_multiIkDuration->setSuffix(QStringLiteral(" s"));
    m_multiIkPlay = new QPushButton(QStringLiteral("\u52a8\u6001\u8fd0\u884c\u591a\u89e3\u9009\u5b9a\u8f68\u8ff9"), basicPage);
    m_multiIkPlay->setObjectName(QStringLiteral("multiIkPlay"));
    m_multiIkPlay->setToolTip(QStringLiteral("\u6bcf\u70b9\u4f7f\u7528\u6807\u8bb0\u7684\u4e00\u7ec4\u89e3\u3002\u4ec5\u7528\u4e8e\u9010\u70b9\u8c03\u8bd5\uff0c\u672a\u8fdb\u884c\u907f\u969c\u89c4\u5212\u3002"));
    multiPlayback->addWidget(m_multiIkDuration); multiPlayback->addWidget(m_multiIkPlay);
    layout->addLayout(multiPlayback);
    connect(m_multiIkPoint, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
        [this](int point) { emit multiIkPointChanged(point); });
    connect(m_multiIkTable, &QTableWidget::itemSelectionChanged, this, &MotionPlanningEditorWidget::updateTrajectoryActions);
    connect(m_multiIkApply, &QPushButton::clicked, this, [this]() {
        emit multiIkApplyRequested(m_multiIkPoint->currentIndex(), m_multiIkTable->currentRow());
    });
    connect(m_multiIkSelect, &QPushButton::clicked, this, [this]() {
        emit multiIkSelectRequested(m_multiIkPoint->currentIndex(), m_multiIkTable->currentRow(), false);
    });
    connect(m_multiIkContinue, &QPushButton::clicked, this, [this]() {
        emit multiIkSelectRequested(m_multiIkPoint->currentIndex(), m_multiIkTable->currentRow(), true);
    });
    connect(m_multiIkPlay, &QPushButton::clicked, this, [this]() {
        if(m_playbackActive) { emit playbackStopRequested(); }
        else { emit multiIkPlaybackRequested(m_multiIkDuration->value()); }
    });

    auto* graphTitle = new QLabel(QStringLiteral("\u5206\u5c42\u56fe Top-M \u7b5b\u9009\u7ed3\u679c"), basicPage);
    graphTitle->setProperty("panelTitle", true);
    layout->addWidget(graphTitle);
    auto* graphForm = new QFormLayout;
    m_graphMaxPaths = new QSpinBox(basicPage);
    m_graphMaxPaths->setObjectName(QStringLiteral("layeredGraphMaxPaths"));
    m_graphMaxPaths->setRange(1, 200); m_graphMaxPaths->setValue(30);
    graphForm->addRow(QStringLiteral("\u5019\u9009\u6570 M"), m_graphMaxPaths);
    m_graphPerStartPaths = new QSpinBox(basicPage);
    m_graphPerStartPaths->setObjectName(QStringLiteral("layeredGraphPerStartPaths"));
    m_graphPerStartPaths->setRange(1, 200); m_graphPerStartPaths->setValue(1);
    m_graphPerStartPaths->setToolTip(QStringLiteral("\u6bcf\u4e2a\u8d77\u70b9\u72ec\u7acb\u4fdd\u7559 K \u6761\uff1bK=1 \u5f97\u5230\u6bcf\u4e2a\u8d77\u70b9\u5404\u81ea\u6700\u4f18\u7684\u4e00\u6761\u3002"));
    graphForm->addRow(QStringLiteral("\u6bcf\u8d77\u70b9\u4fdd\u7559 K \u6761"), m_graphPerStartPaths);
    m_graphWeights = new QLineEdit(QStringLiteral("1, 1, 1, 1, 1, 1"), basicPage);
    m_graphWeights->setObjectName(QStringLiteral("layeredGraphWeights"));
    graphForm->addRow(QStringLiteral("J1..J6 \u6743\u91cd"), m_graphWeights);
    layout->addLayout(graphForm);
    m_graphFilter = new QPushButton(QStringLiteral("\u5206\u5c42\u56fe\u7b5b\u9009"), basicPage);
    m_graphFilter->setObjectName(QStringLiteral("layeredGraphFilter"));
    layout->addWidget(m_graphFilter);
    m_graphStatus = new QLabel(QStringLiteral("\u8bf7\u5148\u751f\u6210\u5b8c\u6574\u591a\u9006\u89e3\u3002\u6b64\u9636\u6bb5\u4e0d\u8fdb\u884c\u78b0\u649e\u68c0\u6d4b\u3002"), basicPage);
    m_graphStatus->setObjectName(QStringLiteral("layeredGraphStatus"));
    m_graphStatus->setWordWrap(true);
    layout->addWidget(m_graphStatus);
    m_graphResults = new QTableWidget(basicPage);
    m_graphResults->setObjectName(QStringLiteral("layeredGraphResults"));
    configureTrajectoryTable(m_graphResults, {QStringLiteral("\u6392\u540d"), QStringLiteral("\u603b\u4ee3\u4ef7 (rad^2)"),
        QStringLiteral("\u70b9\u6570"), QStringLiteral("\u9996\u70b9\u89e3"), QStringLiteral("\u672b\u70b9\u89e3"), QStringLiteral("\u8d77\u70b9\u6784\u578b"), QStringLiteral("\u7ec8\u70b9\u6784\u578b")}, 160);
    m_graphResults->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_graphResultTabs = new QTabWidget(basicPage);
    m_graphResultTabs->setObjectName(QStringLiteral("layeredGraphResultTabs"));
    auto* globalPage = new QWidget(m_graphResultTabs);
    auto* globalLayout = new QVBoxLayout(globalPage);
    auto* globalNote = new QLabel(QStringLiteral("\u5168\u5c40\u6700\u4f18\uff1a\u8d77\u70b9\u4e0d\u9650\uff0c\u6240\u6709\u5b8c\u6574\u5e8f\u5217\u7edf\u4e00\u6392\u540d\uff0c\u524d M \u6761\u53ef\u80fd\u5171\u4eab\u540c\u4e00\u8d77\u70b9\u3002"), globalPage);
    globalNote->setWordWrap(true); globalLayout->addWidget(globalNote); globalLayout->addWidget(m_graphResults);
    auto* startPage = new QWidget(m_graphResultTabs);
    auto* startLayout = new QVBoxLayout(startPage);
    auto* startNote = new QLabel(QStringLiteral("\u6309\u8d77\u70b9\u6700\u4f18\uff1a\u56fa\u5b9a\u6bcf\u4e2a\u8d77\u70b9\u9006\u89e3\uff0c\u5206\u522b\u6c42\u524d K \u6761\uff1bK=1 \u5373\u5404\u8d77\u70b9\u6700\u4f18\u8f68\u8ff9\u3002\u5168\u5c40 >M \u8868\u793a\u672a\u8fdb\u5165\u5168\u5c40\u524d M \u6761\uff0c\u7cbe\u786e\u540d\u6b21\u672a\u8ba1\u7b97\u3002\u4e24\u9875\u5747\u672a\u505a\u78b0\u649e\u68c0\u67e5\u3002"), startPage);
    startNote->setWordWrap(true); startLayout->addWidget(startNote);
    m_graphStartResults = new QTableWidget(startPage);
    m_graphStartResults->setObjectName(QStringLiteral("layeredGraphStartResults"));
    configureTrajectoryTable(m_graphStartResults, {QStringLiteral("\u8d77\u70b9\u9006\u89e3"), QStringLiteral("\u7ec4\u5185\u6392\u540d"),
        QStringLiteral("\u5168\u5c40\u6392\u540d"), QStringLiteral("\u603b\u4ee3\u4ef7 (rad^2)"), QStringLiteral("\u70b9\u6570"), QStringLiteral("\u672b\u70b9\u89e3"), QStringLiteral("\u8d77\u70b9\u6784\u578b"), QStringLiteral("\u7ec8\u70b9\u6784\u578b")}, 160);
    m_graphStartResults->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    startLayout->addWidget(m_graphStartResults);
    m_graphResultTabs->addTab(globalPage, QStringLiteral("\u5168\u5c40 Top-M"));
    m_graphResultTabs->addTab(startPage, QStringLiteral("\u6309\u8d77\u70b9 Top-K"));
    layout->addWidget(m_graphResultTabs);
    m_graphView = new QPushButton(QStringLiteral("\u67e5\u770b\u6784\u578b\u9009\u62e9"), basicPage);
    m_graphView->setObjectName(QStringLiteral("viewConfigurationSelection"));
    layout->addWidget(m_graphView);
    connect(m_graphView, &QPushButton::clicked, this,
        [this]() { emit configurationSelectionRequested(activeGraphTable()->currentRow(), m_graphResultTabs->currentIndex() == 1); });
    m_graphPath = new QTableWidget(basicPage);
    m_graphPath->setObjectName(QStringLiteral("layeredGraphPath"));
    configureTrajectoryTable(m_graphPath, {QStringLiteral("\u70b9"), QStringLiteral("t (s)"), QStringLiteral("\u89e3"),
        QStringLiteral("J1..J6 (deg)"), QStringLiteral("turn J1..J6"), QStringLiteral("\u56fa\u5b9a\u6784\u578b\uff08\u80a9/\u8098/\u8155\uff09")}, 150);
    m_graphPath->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    layout->addWidget(m_graphPath);
    m_graphUse = new QPushButton(QStringLiteral("\u4f7f\u7528\u8be5\u7ed3\u679c\u4f5c\u4e3a\u4f18\u5316\u521d\u59cb\u89e3"), basicPage);
    m_graphUse->setObjectName(QStringLiteral("useLayeredGraphResult"));
    layout->addWidget(m_graphUse);
    connect(m_graphFilter, &QPushButton::clicked, this, [this]() {
        if(m_graphBusy) { emit layeredGraphCancelRequested(); return; }
        QVector<double> weights;
        for(const auto& text : m_graphWeights->text().split(',')) {
            bool ok = false; const double value = text.trimmed().toDouble(&ok);
            if(!ok || !std::isfinite(value) || value < 0) {
                m_graphStatus->setText(QStringLiteral("\u6743\u91cd\u5fc5\u987b\u4e3a\u9017\u53f7\u5206\u9694\u7684\u516d\u4e2a\u975e\u8d1f\u6570\u3002")); return;
            }
            weights.push_back(value);
        }
        emit layeredGraphRequested(m_graphMaxPaths->value(), weights, m_graphPerStartPaths->value());
    });
    connect(m_graphMaxPaths, QOverload<int>::of(&QSpinBox::valueChanged), this,
        [this](int) { emit layeredGraphSettingsChanged(); });
    connect(m_graphPerStartPaths, QOverload<int>::of(&QSpinBox::valueChanged), this,
        [this](int) { emit layeredGraphSettingsChanged(); });
    connect(m_graphWeights, &QLineEdit::textChanged, this,
        [this](const QString&) { emit layeredGraphSettingsChanged(); });
    const auto graphSelection = [this]() {
        updateTrajectoryActions();
        emit layeredGraphSelectionChanged(activeGraphTable()->currentRow(), m_graphResultTabs->currentIndex() == 1);
    };
    connect(m_graphResults, &QTableWidget::itemSelectionChanged, this, graphSelection);
    connect(m_graphStartResults, &QTableWidget::itemSelectionChanged, this, graphSelection);
    connect(m_graphResultTabs, &QTabWidget::currentChanged, this, graphSelection);
    connect(m_graphUse, &QPushButton::clicked, this,
        [this]() { emit useLayeredGraphResultRequested(activeGraphTable()->currentRow(), m_graphResultTabs->currentIndex() == 1); });

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

    auto* cdfTitle = new QLabel(QStringLiteral("\u4f18\u5316\u521d\u59cb\u5173\u8282\u8f68\u8ff9\uff08\u4fdd\u7559\u8f93\u5165\uff09"), cdfPage);
    cdfTitle->setProperty("panelTitle", true);
    cdfLayout->addWidget(cdfTitle);

    m_importCdfButton = new QPushButton(QStringLiteral("Import CDF joint angles..."), cdfPage);
    cdfLayout->addWidget(m_importCdfButton);

    m_cdfJointTable = new QTableWidget(cdfPage);
    m_cdfJointTable->setObjectName(QStringLiteral("cdfInitialJointAngles"));
    configureCdfTable(m_cdfJointTable, 6, 320);
    cdfLayout->addWidget(m_cdfJointTable);

    m_applyCdfJointButton = new QPushButton(QStringLiteral("Apply selected CDF joint angles"), cdfPage);
    cdfLayout->addWidget(m_applyCdfJointButton);

    auto* cdfRepairTitle = new QLabel(QStringLiteral("APF + CDF/QP Collision Repair"), cdfPage);
    cdfRepairTitle->setProperty("panelTitle", true);
    cdfLayout->addWidget(cdfRepairTitle);

    auto* cdfRepairForm = new QFormLayout();
    cdfRepairForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    m_cdfTcpDeviation = makePoseSpinBox(cdfPage, 1.0, 1000.0, 100.0, 5.0, QStringLiteral(" mm"));
    m_cdfTcpDeviation->setObjectName(QStringLiteral("cdfApfTcpDeviation"));
    m_cdfTcpDeviation->setToolTip(QStringLiteral("\u4ee5\u539f\u59cb\u672b\u7aef\u63a7\u5236\u70b9\u6298\u7ebf\u4e3a\u53c2\u8003\uff0cAPF\u3001\u5e73\u6ed1\u4e0eQP\u5747\u5fc5\u987b\u9075\u5b88\uff1b\u8303\u56f4\u5185\u65e0\u6cd5\u7ed5\u5f00\u5219\u5931\u8d25\uff0c\u4e0d\u81ea\u52a8\u653e\u5bbd\u3002\u4ec5\u9650\u5236\u4f4d\u7f6e\uff0c\u4e0d\u9501\u5b9a\u59ff\u6001\u3002"));
    cdfRepairForm->addRow(QStringLiteral("APF \u672b\u7aef\u6700\u5927\u504f\u79fb"), m_cdfTcpDeviation);

    m_cdfEquivalentConfigurations = new QCheckBox(QStringLiteral("\u4fdd\u6301\u8d77\u7ec8 TCP \u4f4d\u59ff\uff0c\u5141\u8bb8\u7b49\u4ef7\u9006\u89e3\u6784\u578b"), cdfPage);
    m_cdfEquivalentConfigurations->setObjectName(QStringLiteral("cdfEquivalentConfigurations"));
    m_cdfEquivalentConfigurations->setChecked(true);
    m_cdfEquivalentConfigurations->setToolTip(QStringLiteral("Top-K \u521d\u59cb\u89e3\u6709\u539f\u59cb Cartesian \u76ee\u6807\u65f6\uff0c\u5c1d\u8bd5\u8fde\u7eed\u9006\u89e3\u4ee5\u51cf\u5c11\u5206\u652f\u8df3\u53d8\u548c\u7ed5\u5708\u3002\u53d6\u6d88\u540e\u4fdd\u7559\u539f\u9009\u5173\u8282\u6784\u578b\u3002\u5355\u72ec\u5bfc\u5165\u5173\u8282 TXT \u4e0d\u542f\u7528\u6b64\u9879\u3002"));
    cdfRepairForm->addRow(QString(), m_cdfEquivalentConfigurations);

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
    m_cdfMaxIterations->setValue(1);
    cdfRepairForm->addRow(QStringLiteral("Max iterations"), m_cdfMaxIterations);

    m_cdfKeepEndpoints = new QCheckBox(QStringLiteral("Lock endpoints"), cdfPage);
    m_cdfKeepEndpoints->setChecked(true);
    cdfRepairForm->addRow(QString(), m_cdfKeepEndpoints);

    m_cdfSmoothWeight = makePoseSpinBox(cdfPage, 0.0, 100.0, 2.0, 0.5);
    m_cdfSmoothWeight->setObjectName(QStringLiteral("cdfCurvatureWeight"));
    m_cdfSmoothWeight->setToolTip(QStringLiteral("\u975e\u5747\u5300\u53c2\u8003\u5f27\u957f\u53c2\u6570\u7684\u4e8c\u9636\u5f2f\u66f2\u60e9\u7f5a\uff1b0 \u5173\u95ed QP \u66f2\u7387\u5e73\u6ed1\u3002"));
    cdfRepairForm->addRow(QStringLiteral("\u66f2\u7387\u5e73\u6ed1\u6743\u91cd"), m_cdfSmoothWeight);
    m_cdfSmoothingPasses = new QSpinBox(cdfPage);
    m_cdfSmoothingPasses->setRange(0, 12); m_cdfSmoothingPasses->setValue(6);
    m_cdfSmoothingPasses->setToolTip(QStringLiteral("0: off; 1-3: one multiscale cycle; 4-6: two; 7-12: three. Every cycle checks 128, 64, 32, 16, 8, 4 and 2-node windows against the ordered TCP corridor and collisions."));
    cdfRepairForm->addRow(QStringLiteral("\u78b0\u649e\u7ea6\u675f\u5e73\u6ed1\u5f3a\u5ea6"), m_cdfSmoothingPasses);
    m_cdfRetime = new QCheckBox(QStringLiteral("\u91cd\u65b0\u5b9a\u65f6\u5e76\u68c0\u67e5\u79bb\u6563\u901f\u5ea6 / \u52a0\u901f\u5ea6"), cdfPage);
    m_cdfRetime->setChecked(true); cdfRepairForm->addRow(QString(), m_cdfRetime);
    m_cdfFallbackVelocity = makePoseSpinBox(cdfPage, 0.1, 1000.0, 60.0, 5.0, QStringLiteral(" deg/s"));
    m_cdfFallbackAcceleration = makePoseSpinBox(cdfPage, 0.1, 10000.0, 120.0, 10.0, QStringLiteral(" deg/s^2"));
    m_cdfFallbackVelocity->setToolTip(QStringLiteral("\u4ec5\u7528\u4e8e\u673a\u5668\u4eba\u6a21\u578b\u672a\u63d0\u4f9b\u901f\u5ea6\u9650\u4f4d\u7684\u5173\u8282\u3002"));
    m_cdfFallbackAcceleration->setToolTip(QStringLiteral("\u4ec5\u7528\u4e8e\u673a\u5668\u4eba\u6a21\u578b\u672a\u63d0\u4f9b\u52a0\u901f\u5ea6\u9650\u4f4d\u7684\u5173\u8282\uff1b\u8fd9\u4e0d\u662f\u5382\u5bb6\u8ba4\u8bc1\u53c2\u6570\u3002"));
    cdfRepairForm->addRow(QStringLiteral("\u7f3a\u5931\u901f\u5ea6\u9650\u4f4d\u7684\u7f3a\u7701\u503c"), m_cdfFallbackVelocity);
    cdfRepairForm->addRow(QStringLiteral("\u7f3a\u5931\u52a0\u901f\u5ea6\u9650\u4f4d\u7684\u7f3a\u7701\u503c"), m_cdfFallbackAcceleration);
    cdfLayout->addLayout(cdfRepairForm);

    m_repairCdfTrajectoryButton = new QPushButton(QStringLiteral("Repair imported trajectory with APF + CDF/QP"), cdfPage);
    cdfLayout->addWidget(m_repairCdfTrajectoryButton);

    m_exportCdfTrajectoryButton = new QPushButton(
        QStringLiteral("Export APF + CDF/QP trajectory..."),
        cdfPage);
    cdfLayout->addWidget(m_exportCdfTrajectoryButton);

    m_cdfResult = new QLabel(QStringLiteral("Import a CDF joint angle file."), cdfPage);
    m_cdfResult->setWordWrap(true);
    cdfLayout->addWidget(m_cdfResult);
    auto* stagesTitle = new QLabel(QStringLiteral("\u9636\u6bb5\u7ed3\u679c\u68c0\u67e5\uff1a\u8f93\u5165 \u2192 APF \u2192 CDF/QP"), cdfPage);
    stagesTitle->setProperty("panelTitle", true); cdfLayout->addWidget(stagesTitle);
    m_cdfStageCombo = new QComboBox(cdfPage); m_cdfStageCombo->setObjectName(QStringLiteral("cdfStageSelection"));
    cdfLayout->addWidget(m_cdfStageCombo);
    m_cdfStageSummary = new QLabel(QStringLiteral("\u8fd0\u884c\u4f18\u5316\u540e\u53ef\u5206\u522b\u68c0\u67e5\u5404\u9636\u6bb5\u3002"), cdfPage);
    m_cdfStageSummary->setWordWrap(true); cdfLayout->addWidget(m_cdfStageSummary);
    m_cdfStageTable = new QTableWidget(cdfPage); m_cdfStageTable->setObjectName(QStringLiteral("cdfStageJointAngles"));
    configureCdfTable(m_cdfStageTable, 6, 220); cdfLayout->addWidget(m_cdfStageTable);
    auto* stageActions = new QHBoxLayout();
    m_cdfStageApply = new QPushButton(QStringLiteral("\u5e94\u7528\u9636\u6bb5\u6240\u9009\u70b9"), cdfPage);
    m_cdfStageApply->setObjectName(QStringLiteral("cdfStageApply"));
    m_cdfStagePlay = new QPushButton(QStringLiteral("\u52a8\u6001\u64ad\u653e\u9636\u6bb5\u8f68\u8ff9"), cdfPage);
    m_cdfStagePlay->setObjectName(QStringLiteral("cdfStagePlay"));
    stageActions->addWidget(m_cdfStageApply); stageActions->addWidget(m_cdfStagePlay); cdfLayout->addLayout(stageActions);
    auto* stageTiming = new QHBoxLayout();
    m_cdfStageDuration = makePoseSpinBox(cdfPage, 0.1, 3600.0, 5.0, 1.0, QStringLiteral(" s"));
    m_cdfStageActualTiming = new QCheckBox(QStringLiteral("\u6309\u8bb0\u5f55\u65f6\u95f4 1\u00d7 \u64ad\u653e"), cdfPage);
    m_cdfStageActualTiming->setObjectName(QStringLiteral("cdfStageActualTiming"));
    m_cdfStageActualTiming->setToolTip(QStringLiteral("\u52fe\u9009\u540e\u4f7f\u7528\u9636\u6bb5\u8bb0\u5f55\u65f6\u95f4\uff0c\u4e0d\u91c7\u7528\u9884\u89c8\u65f6\u957f\u3002\u7ed8\u5236\u8fc7\u8f7d\u4f1a\u653e\u6162\u663e\u793a\uff1b\u8d28\u91cf\u7a97\u53e3\u7684\u901f\u5ea6\u59cb\u7ec8\u7531\u8bb0\u5f55\u65f6\u95f4\u8ba1\u7b97\u3002"));
    stageTiming->addWidget(new QLabel(QStringLiteral("\u9884\u89c8\u65f6\u957f"), cdfPage)); stageTiming->addWidget(m_cdfStageDuration);
    stageTiming->addWidget(m_cdfStageActualTiming); cdfLayout->addLayout(stageTiming);
    m_cdfStageExport = new QPushButton(QStringLiteral("\u5bfc\u51fa\u6240\u9009\u9636\u6bb5\u8f68\u8ff9..."), cdfPage);
    m_cdfStageExport->setObjectName(QStringLiteral("cdfStageExport")); cdfLayout->addWidget(m_cdfStageExport);
    m_cdfAnalysisButton = new QPushButton(QStringLiteral("\u67e5\u770b\u9636\u6bb5\u5bf9\u6bd4\u4e0e\u8d28\u91cf\u62a5\u544a"), cdfPage);
    m_cdfAnalysisButton->setObjectName(QStringLiteral("cdfStageAnalysis")); cdfLayout->addWidget(m_cdfAnalysisButton);
    connect(m_cdfStageCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) { emit cdfStageChanged(index); updateCdfActions(); });
    connect(m_cdfStageTable, &QTableWidget::currentCellChanged, this, [this]() { updateCdfActions(); });
    connect(m_cdfStageApply, &QPushButton::clicked, this, [this]() { emit applyCdfStagePointRequested(m_cdfStageCombo->currentIndex(), selectedOriginalPointIndex(m_cdfStageTable)); });
    connect(m_cdfStagePlay, &QPushButton::clicked, this, [this]() {
        if(m_playbackActive) emit playbackStopRequested();
        else {
            if(m_endEffectorTraceVisible) m_endEffectorTraceVisible->setChecked(true);
            emit playCdfStageRequested(m_cdfStageCombo->currentIndex(), m_cdfStageDuration->value(), m_cdfStageActualTiming->isChecked());
        }
    });
    connect(m_cdfStageExport, &QPushButton::clicked, this, [this]() { emit exportCdfStageRequested(m_cdfStageCombo->currentIndex()); });
    connect(m_cdfAnalysisButton, &QPushButton::clicked, this, &MotionPlanningEditorWidget::cdfAnalysisRequested);
    connect(m_cdfStageActualTiming, &QCheckBox::toggled, this, [this]() { updateCdfActions(); });
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
    connect(m_exportJointTrajectoryButton, &QPushButton::clicked,
        this, &MotionPlanningEditorWidget::exportJointTrajectoryRequested);
    connect(m_trajectoryPointsVisible, &QCheckBox::toggled,
        this, &MotionPlanningEditorWidget::trajectoryPointsVisibilityChanged);
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
    updateCdfActions();
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
    settings.allowEquivalentConfigurations = m_cdfEquivalentConfigurations->isChecked();
    settings.apfMaxTcpDeviation = m_cdfTcpDeviation->value() / 1000.0;
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
    settings.smoothWeight = m_cdfSmoothWeight->value();
    settings.smoothingPasses = m_cdfSmoothingPasses->value();
    settings.retimeOutput = m_cdfRetime->isChecked();
    settings.fallbackVelocityDegrees = m_cdfFallbackVelocity->value();
    settings.fallbackAccelerationDegrees = m_cdfFallbackAcceleration->value();
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

void MotionPlanningEditorWidget::setCdfAnalysisStages(const QStringList& names)
{
    if(m_cdfAnalysisDialog) { m_cdfAnalysisDialog->close(); m_cdfAnalysisDialog.clear(); }
    const QSignalBlocker blocker(m_cdfStageCombo);
    m_cdfStageCombo->clear(); m_cdfStageCombo->addItems(names);
    m_cdfStageCombo->setCurrentIndex(names.size() - 1);
    if(names.isEmpty()) {
        m_cdfStageTable->clearContents(); m_cdfStageTable->setRowCount(0);
        m_cdfStageSummary->setText(QStringLiteral("\u8fd0\u884c\u4f18\u5316\u540e\u53ef\u5206\u522b\u68c0\u67e5\u5404\u9636\u6bb5\u3002"));
    }
    updateCdfActions();
}

void MotionPlanningEditorWidget::setCdfStageView(const QVector<QString>& names,
    const QVector<CdfJointAngleRow>& rows, const QString& summary)
{
    QStringList headers{QStringLiteral("#"), QStringLiteral("time_s")};
    for(const auto& name : names) headers << name + QStringLiteral(" (deg)");
    configureTrajectoryTable(m_cdfStageTable, headers, 220);
    const QSignalBlocker blocker(m_cdfStageTable);
    m_cdfStageTable->setUpdatesEnabled(false); m_cdfStageTable->clearContents(); m_cdfStageTable->clearSpans();
    const auto sourceRows = sampledSourceRows(rows.size());
    const bool preview = sourceRows.size() < rows.size();
    m_cdfStageTable->setRowCount(sourceRows.size() + (preview ? 1 : 0));
    for(int i = 0; i < sourceRows.size(); ++i) {
        const auto index = sourceRows[i]; const auto& row = rows[index];
        auto* item = makeReadOnlyItem(QString::number(row.index)); setOriginalPointIndex(item, index);
        m_cdfStageTable->setItem(i, 0, item); m_cdfStageTable->setItem(i, 1, makeReadOnlyItem(row.timeText));
        for(int j = 0; j < row.jointAngleTexts.size(); ++j) m_cdfStageTable->setItem(i, j + 2, makeReadOnlyItem(row.jointAngleTexts[j]));
    }
    if(preview) {
        m_cdfStageTable->setSpan(sourceRows.size(), 0, 1, headers.size());
        m_cdfStageTable->setItem(sourceRows.size(), 0, makeNoticeItem(QStringLiteral("\u663e\u793a %1 / %2 \u884c\uff1b\u5e94\u7528\u3001\u64ad\u653e\u548c\u5bfc\u51fa\u4ecd\u4f7f\u7528\u5168\u90e8\u539f\u59cb\u6570\u636e\u3002").arg(sourceRows.size()).arg(rows.size())));
    }
    if(!sourceRows.empty()) m_cdfStageTable->setCurrentCell(0, 0);
    m_cdfStageTable->setUpdatesEnabled(true); m_cdfStageSummary->setText(summary); updateCdfActions();
}

void MotionPlanningEditorWidget::showCdfAnalysis(const QVector<CdfStageViewData>& stages,
    const QStringList& names, const QString& diagnostics)
{
    if(m_cdfAnalysisDialog) { m_cdfAnalysisDialog->show(); m_cdfAnalysisDialog->raise(); return; }
    auto* dialog = new CdfTrajectoryAnalysisDialog(stages, names, diagnostics, this);
    m_cdfAnalysisDialog = dialog;
    connect(dialog, &CdfTrajectoryAnalysisDialog::exportQualityRequested, this, &MotionPlanningEditorWidget::exportCdfQualityRequested);
    m_cdfAnalysisDialog->show();
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

    if(m_graphFilter) {
        m_graphFilter->setEnabled(m_graphBusy || (!m_multiIkBusy && m_multiIkComplete && !m_playbackActive));
        m_graphView->setEnabled(!m_graphBusy && !m_multiIkBusy && (m_graphResults->rowCount() > 0 || m_graphStartResults->rowCount() > 0));
        m_graphMaxPaths->setEnabled(!m_graphBusy);
        m_graphPerStartPaths->setEnabled(!m_graphBusy);
        m_graphWeights->setEnabled(!m_graphBusy);
        m_graphUse->setEnabled(!m_graphBusy && !m_multiIkBusy && m_multiIkComplete && activeGraphTable()->currentRow() >= 0);
    }
    if(m_allIkButton) {
        m_allIkButton->setEnabled(m_multiIkBusy || (hasRobot && hasCartesian && !m_playbackActive));
        m_ikToolMode->setEnabled(!m_multiIkBusy);
        const bool candidate = !m_multiIkBusy && m_multiIkTable->currentRow() >= 0;
        m_multiIkApply->setEnabled(candidate);
        m_multiIkSelect->setEnabled(candidate && !m_playbackActive);
        m_multiIkContinue->setEnabled(candidate && !m_playbackActive);
        m_multiIkPlay->setEnabled(m_playbackActive || (!m_multiIkBusy && m_multiIkComplete));
        m_multiIkDuration->setEnabled(!m_playbackActive && !m_multiIkBusy);
        m_multiIkPlay->setText(m_playbackActive ? QStringLiteral("Stop playback") :
            QStringLiteral("\u52a8\u6001\u8fd0\u884c\u591a\u89e3\u9009\u5b9a\u8f68\u8ff9"));
    }
    if(m_ikToolMode != nullptr) {
        m_ikToolMode->setEnabled(hasRobot && hasCartesian && !m_multiIkBusy);
    }
    if(m_solveIkButton != nullptr) {
        m_solveIkButton->setEnabled(hasRobot && hasCartesian && !m_multiIkBusy);
    }
    if(m_applyJointPointButton != nullptr) {
        m_applyJointPointButton->setEnabled(hasRobot && hasJoint && hasValidJointRow);
    }
    if(m_exportJointTrajectoryButton != nullptr) {
        m_exportJointTrajectoryButton->setEnabled(hasRobot && hasJoint && hasAnyJointRow);
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
        m_applyCdfJointButton->setEnabled(hasRobot && hasValidRow && !m_playbackActive);
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
        m_repairCdfTrajectoryButton->setEnabled(hasRobot && hasImportedRows && !m_playbackActive);
    }
    if(m_exportCdfTrajectoryButton != nullptr) {
        m_exportCdfTrajectoryButton->setEnabled(hasRobot && m_cdfExportAvailable);
    }
    const bool hasStage = m_cdfStageCombo && m_cdfStageCombo->count() > 0;
    if(m_cdfStageCombo) m_cdfStageCombo->setEnabled(hasStage && !m_playbackActive);
    if(m_cdfStageApply) m_cdfStageApply->setEnabled(hasStage && hasRobot && !m_playbackActive && selectedOriginalPointIndex(m_cdfStageTable) >= 0);
    if(m_cdfStagePlay) {
        m_cdfStagePlay->setEnabled(hasStage && hasRobot);
        m_cdfStagePlay->setText(m_playbackActive ? QStringLiteral("\u505c\u6b62\u64ad\u653e") : QStringLiteral("\u52a8\u6001\u64ad\u653e\u9636\u6bb5\u8f68\u8ff9"));
    }
    if(m_cdfStageExport) m_cdfStageExport->setEnabled(hasStage);
    if(m_cdfAnalysisButton) m_cdfAnalysisButton->setEnabled(hasStage);
    if(m_cdfStageDuration) m_cdfStageDuration->setEnabled(hasStage && !m_playbackActive && !m_cdfStageActualTiming->isChecked());
    if(m_cdfStageActualTiming) m_cdfStageActualTiming->setEnabled(hasStage && !m_playbackActive);
    for(QWidget* setting : QVector<QWidget*>{m_cdfSmoothWeight, m_cdfSmoothingPasses, m_cdfRetime, m_cdfFallbackVelocity, m_cdfFallbackAcceleration})
        if(setting) setting->setEnabled(hasRobot && hasImportedRows && !m_playbackActive);
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

void MotionPlanningEditorWidget::setMultiIkPoints(const QVector<MultiIkPointRow>& points,
    const QString& summary, bool complete)
{
    const QSignalBlocker blocker(m_multiIkPoint);
    m_multiIkPoint->clear();
    for(const auto& point : points) { m_multiIkPoint->addItem(point.label); }
    m_multiIkStatus->setText(summary);
    m_multiIkComplete = complete;
    m_multiIkTable->setRowCount(0);
    updateTrajectoryActions();
}

void MotionPlanningEditorWidget::setMultiIkCandidates(const QVector<MultiIkCandidateRow>& rows, int selected)
{
    m_multiIkTable->setUpdatesEnabled(false);
    m_multiIkTable->setRowCount(rows.size());
    for(int r = 0; r < rows.size(); ++r) {
        for(int c = 0; c < rows[r].columns.size(); ++c) {
            m_multiIkTable->setItem(r, c, new QTableWidgetItem(rows[r].columns[c]));
        }
    }
    if(selected >= 0 && selected < rows.size()) { m_multiIkTable->selectRow(selected); }
    m_multiIkTable->setUpdatesEnabled(true);
    updateTrajectoryActions();
}

void MotionPlanningEditorWidget::setMultiIkBusy(bool busy, const QString& message)
{
    m_multiIkBusy = busy;
    m_multiIkStatus->setText(message);
    m_allIkButton->setText(busy ? QStringLiteral("\u53d6\u6d88\u5168\u9006\u89e3") : QStringLiteral("\u5168\u9006\u89e3"));
    updateTrajectoryActions();
}

void MotionPlanningEditorWidget::setMultiIkPointLabel(int point, const QString& label)
{
    m_multiIkPoint->setItemText(point, label);
}

QTableWidget* MotionPlanningEditorWidget::activeGraphTable() const
{
    return m_graphResultTabs->currentIndex() == 1 ? m_graphStartResults : m_graphResults;
}

void MotionPlanningEditorWidget::showConfigurationSelection(const QVector<QVector<int>>& sequences,
    const QVector<QVector<int>>& startSequences, const QStringList& startLabels, int initialRank, bool byStart, const ConfigurationSelectionCatalog& catalog)
{
    if(sequences.isEmpty() && startSequences.isEmpty()) { return; }
    if(!m_configurationDialog) {
        m_configurationDialog = new ConfigurationSelectionDialog(sequences, startSequences, startLabels, initialRank, byStart, this, catalog);
    }
    static_cast<ConfigurationSelectionDialog*>(m_configurationDialog.data())->selectPage(byStart);
    m_configurationDialog->show();
    m_configurationDialog->raise();
    m_configurationDialog->activateWindow();
}

void MotionPlanningEditorWidget::setLayeredGraphResults(const QVector<QStringList>& rows, const QString& summary,
    const QVector<QStringList>& startRows)
{
    if(m_configurationDialog) { m_configurationDialog->close(); m_configurationDialog.clear(); }
    {
        const QSignalBlocker blocker(m_graphResults), startBlocker(m_graphStartResults);
        replaceLayeredGraphRows(m_graphResults, rows);
        replaceLayeredGraphRows(m_graphStartResults, startRows);
        m_graphPath->setRowCount(0);
        m_graphStatus->setText(summary);
    }
    if(!rows.empty()) { m_graphResults->selectRow(0); }
    if(!startRows.empty()) { m_graphStartResults->selectRow(0); }
    updateTrajectoryActions();
}

void MotionPlanningEditorWidget::setLayeredGraphPath(const QVector<QStringList>& rows)
{
    replaceLayeredGraphRows(m_graphPath, rows);
}

void MotionPlanningEditorWidget::setLayeredGraphBusy(bool busy, const QString& message)
{
    m_graphBusy = busy;
    m_graphStatus->setText(message);
    m_graphFilter->setText(busy ? QStringLiteral("\u53d6\u6d88\u5206\u5c42\u56fe\u7b5b\u9009") : QStringLiteral("\u5206\u5c42\u56fe\u7b5b\u9009"));
    updateTrajectoryActions();
}

void MotionPlanningEditorWidget::showCdfPage()
{
    m_tabs->setCurrentIndex(1);
}
