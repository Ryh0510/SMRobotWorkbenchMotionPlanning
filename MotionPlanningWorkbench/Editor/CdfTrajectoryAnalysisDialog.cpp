#include "CdfTrajectoryAnalysisDialog.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QTableWidget>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <limits>

QStringList cdfQualityMetricLabels()
{
    return {QStringLiteral("\u70b9\u6570\uff08\u7528\u4e8e\u4e00\u81f4\u5bf9\u6bd4\u7684\u52a0\u5bc6\u8282\u70b9\uff09"), QStringLiteral("\u8bb0\u5f55\u7684\u6267\u884c\u65f6\u957f (s)"),
        QStringLiteral("\u5173\u8282\u8def\u5f84\u957f\u5ea6 (rad) \u2193"), QStringLiteral("\u540c\u4e00\u53c2\u8003\u53c2\u6570\u7684\u5f2f\u66f2\u4ee3\u4ef7 \u2193"),
        QStringLiteral("\u6700\u5927\u5173\u8282\u6298\u89d2 (deg) \u2193"), QStringLiteral("\u5cf0\u503c\u5173\u8282\u901f\u5ea6 (deg/s)"),
        QStringLiteral("\u5cf0\u503c\u79bb\u6563\u52a0\u901f\u5ea6 (deg/s^2)"), QStringLiteral("\u975e\u9012\u589e\u65f6\u95f4\u6bb5\u6570"),
        QStringLiteral("\u5173\u8282\u4f4d\u7f6e\u8d8a\u9650\u6570"), QStringLiteral("\u901f\u5ea6\u8d8a\u9650\u6570"), QStringLiteral("\u79bb\u6563\u52a0\u901f\u5ea6\u8d8a\u9650\u6570"),
        QStringLiteral("\u4e0d\u901a\u8fc7\u78b0\u649e\u68c0\u67e5\u7684\u6bb5\u6570"), QStringLiteral("\u8282\u70b9\u6700\u5c0f\u5b89\u5168\u4f59\u91cf Phi (mm) \u2191"),
        QStringLiteral("TCP \u8def\u5f84\u957f\u5ea6 (mm)"), QStringLiteral("TCP \u5cf0\u503c\u6bb5\u901f\u5ea6 (mm/s)"),
        QStringLiteral("\u76f8\u5bf9\u539f\u59cb\u63a7\u5236\u70b9\u6298\u7ebf\u7684\u8282\u70b9\u6700\u5927\u504f\u79fb (mm)"), QStringLiteral("\u76f8\u5bf9\u8f93\u5165\u5173\u8282\u63d2\u503c\u7684\u6700\u5927\u59ff\u6001\u504f\u79fb (deg)"),
        QStringLiteral("\u4f7f\u7528\u7f3a\u7701\u901f\u5ea6/\u52a0\u901f\u5ea6\u9650\u4f4d\u7684\u8f74\u6570"), QStringLiteral("\u9636\u6bb5\u7d2f\u8ba1\u8ba1\u7b97\u8017\u65f6 (s)")};
}

namespace
{
    class ComparisonPlot final : public QWidget
    {
    public:
        struct Curve { QString name; QColor color; QVector<QPointF> points; };
        QVector<Curve> curves;
        QString xLabel, yLabel;
        bool equalAspect = false;
        explicit ComparisonPlot(QWidget* parent) : QWidget(parent) { setMinimumSize(550, 360); }
    protected:
        void paintEvent(QPaintEvent*) override
        {
            QPainter painter(this);
            painter.setRenderHint(QPainter::Antialiasing);
            painter.fillRect(rect(), palette().base());
            QRectF plot(88, 45, std::max(1, width() - 115), std::max(1, height() - 103));
            double minX = std::numeric_limits<double>::max(), maxX = -minX, minY = minX, maxY = -minX;
            for(const auto& curve : curves) for(const auto& p : curve.points) {
                if(!std::isfinite(p.x()) || !std::isfinite(p.y())) continue;
                minX = std::min(minX, p.x()); maxX = std::max(maxX, p.x());
                minY = std::min(minY, p.y()); maxY = std::max(maxY, p.y());
            }
            if(minX > maxX || minY > maxY) {
                painter.setPen(palette().text().color());
                painter.drawText(plot, Qt::AlignCenter, QStringLiteral("\u8be5\u9636\u6bb5\u7684\u6570\u636e\u4e0d\u53ef\u7528\uff1b\u8bf7\u68c0\u67e5\u65f6\u95f4\u6233\u6216\u5b9e\u9645 TCP \u6a21\u578b\u3002"));
                return;
            }
            if(maxX - minX < 1e-9) { minX -= 0.5; maxX += 0.5; }
            if(maxY - minY < 1e-9) { minY -= 0.5; maxY += 0.5; }
            const double margin = 0.05 * (maxY - minY); minY -= margin; maxY += margin;
            if(equalAspect) {
                const double scale = std::max((maxX - minX) / plot.width(), (maxY - minY) / plot.height());
                const double x = 0.5 * (minX + maxX), y = 0.5 * (minY + maxY);
                minX = x - scale * plot.width() / 2; maxX = x + scale * plot.width() / 2;
                minY = y - scale * plot.height() / 2; maxY = y + scale * plot.height() / 2;
            }
            const auto map = [&](const QPointF& p) {
                return QPointF(plot.left() + (p.x() - minX) / (maxX - minX) * plot.width(),
                    plot.bottom() - (p.y() - minY) / (maxY - minY) * plot.height());
            };
            for(int i = 0; i <= 5; ++i) {
                const double fraction = i / 5.0;
                const double x = plot.left() + fraction * plot.width(), y = plot.bottom() - fraction * plot.height();
                painter.setPen(palette().mid().color());
                painter.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()));
                painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
                painter.setPen(palette().text().color());
                painter.drawText(QRectF(x - 40, plot.bottom() + 6, 80, 22), Qt::AlignCenter, QString::number(minX + fraction * (maxX - minX), 'g', 5));
                painter.drawText(QRectF(4, y - 11, 76, 22), Qt::AlignRight | Qt::AlignVCenter, QString::number(minY + fraction * (maxY - minY), 'g', 5));
            }
            painter.drawText(QRectF(plot.left(), height() - 31, plot.width(), 23), Qt::AlignCenter, xLabel);
            painter.drawText(QRectF(8, 8, width() - 16, 25), Qt::AlignLeft, yLabel);
            painter.save(); painter.setClipRect(plot);
            for(const auto& curve : curves) {
                QPainterPath path; bool previous = false;
                for(const auto& p : curve.points) {
                    if(!std::isfinite(p.x()) || !std::isfinite(p.y())) { previous = false; continue; }
                    if(!previous) path.moveTo(map(p)); else path.lineTo(map(p));
                    previous = true;
                }
                painter.setPen(QPen(curve.color, 1.6)); painter.drawPath(path);
            }
            painter.restore();
        }
    };
}

CdfTrajectoryAnalysisDialog::CdfTrajectoryAnalysisDialog(const QVector<CdfStageViewData>& stages,
    const QStringList& jointNames, const QString& diagnostics, QWidget* parent)
    : QDialog(parent)
{
    setObjectName(QStringLiteral("cdfTrajectoryAnalysis"));
    setAttribute(Qt::WA_DeleteOnClose);
    setWindowTitle(QStringLiteral("APF / CDF \u8f68\u8ff9\u4e0e\u8d28\u91cf\u5bf9\u6bd4"));
    resize(1120, 780);
    auto* layout = new QVBoxLayout(this);
    auto* note = new QLabel(QStringLiteral("Top-K \u7684\u5168\u5c40\u6392\u540d\u4ec5\u9488\u5bf9\u5206\u5c42\u56fe\u5173\u8282\u53d8\u5316\u4ee3\u4ef7\u3002APF/CDF \u662f\u5c40\u90e8\u907f\u969c\u4e0e\u5e73\u6ed1\uff0c\u4e0d\u4fdd\u8bc1\u975e\u51f8\u95ee\u9898\u5168\u5c40\u6700\u4f18\u3002\n\u901f\u5ea6\u6309\u8bb0\u5f55\u65f6\u95f4\u8ba1\u7b97\uff0c\u9884\u89c8\u5feb\u6162\u4e0d\u4ee3\u8868\u6267\u884c\u901f\u5ea6\uff1b\u52a0\u901f\u5ea6\u4e3a\u8282\u70b9\u6709\u9650\u5dee\u5206\uff0c\u5206\u6bb5\u7ebf\u6027\u6298\u70b9\u7684\u8fde\u7eed\u7269\u7406\u52a0\u901f\u5ea6\u5c1a\u672a\u8ba4\u8bc1\u3002\n\u4f4d\u7f6e\u504f\u79fb\u6309\u539f\u59cb\u63a7\u5236\u70b9\u6298\u7ebf\u7684\u6709\u5e8f\u5bf9\u5e94\u8ba1\u7b97\uff1b\u6574\u6bb5\u8fd0\u52a8\u9ad8\u5bc6\u5ea6\u68c0\u67e5\u6700\u5927\u504f\u79fb\u9650\u5236\u3002\u59ff\u6001\u504f\u79fb\u4ecd\u76f8\u5bf9\u8f93\u5165\u5173\u8282\u63d2\u503c\uff0c\u4e0d\u7b49\u4e8e\u9501\u5b9a\u59ff\u6001\u3002"), this);
    note->setWordWrap(true); layout->addWidget(note);
    auto* tabs = new QTabWidget(this); tabs->setObjectName(QStringLiteral("cdfAnalysisTabs")); layout->addWidget(tabs, 1);
    auto* summary = new QWidget(tabs); auto* summaryLayout = new QVBoxLayout(summary);
    auto* table = new QTableWidget(summary); table->setObjectName(QStringLiteral("cdfQualityComparison"));
    const auto labels = cdfQualityMetricLabels();
    table->setRowCount(labels.size()); table->setColumnCount(stages.size() + 1);
    QStringList headers{QStringLiteral("\u6307\u6807")}; for(const auto& stage : stages) headers << stage.name;
    table->setHorizontalHeaderLabels(headers); table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->verticalHeader()->setVisible(false);
    for(int row = 0; row < labels.size(); ++row) {
        table->setItem(row, 0, new QTableWidgetItem(labels[row]));
        for(int column = 0; column < stages.size(); ++column)
            table->setItem(row, column + 1, new QTableWidgetItem(stages[column].metrics.value(row)));
    }
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents); summaryLayout->addWidget(table, 1);
    auto* jointTable = new QTableWidget(summary); jointTable->setObjectName(QStringLiteral("cdfJointLimitComparison"));
    jointTable->setColumnCount(6); jointTable->setHorizontalHeaderLabels({QStringLiteral("\u9636\u6bb5"), QStringLiteral("\u5173\u8282"), QStringLiteral("\u5cf0\u503c\u901f\u5ea6 deg/s"), QStringLiteral("\u901f\u5ea6\u9650\u4f4d deg/s"), QStringLiteral("\u5cf0\u503c\u52a0\u901f\u5ea6 deg/s^2"), QStringLiteral("\u52a0\u901f\u5ea6\u9650\u4f4d deg/s^2")});
    jointTable->setRowCount(stages.size() * jointNames.size()); jointTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    for(int i = 0; i < stages.size(); ++i) for(int j = 0; j < jointNames.size(); ++j) {
        const auto& s = stages[i]; const int row = i * jointNames.size() + j;
        const auto number = [](const QVector<double>& v, int index) { return index < v.size() && std::isfinite(v[index]) ? QString::number(v[index], 'g', 7) : QStringLiteral("\u4e0d\u53ef\u7528"); };
        const QStringList values{s.name, jointNames[j], number(s.peakVelocities, j), number(s.velocityLimits, j), number(s.peakAccelerations, j), number(s.accelerationLimits, j)};
        for(int c = 0; c < values.size(); ++c) jointTable->setItem(row, c, new QTableWidgetItem(values[c]));
    }
    jointTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents); jointTable->setMaximumHeight(155); summaryLayout->addWidget(jointTable);
    tabs->addTab(summary, QStringLiteral("\u8d28\u91cf\u4e0e\u9650\u4f4d"));
    const QVector<QColor> colors{QColor(80, 150, 230), QColor(235, 155, 35), QColor(210, 60, 150)};
    for(int type = 0; type < 6; ++type) {
        auto* page = new QWidget(tabs); auto* pageLayout = new QVBoxLayout(page); auto* controls = new QHBoxLayout();
        auto* axis = new QComboBox(page);
        if(type <= 2) axis->addItems(jointNames);
        else if(type == 3) axis->addItems({QStringLiteral("XY"), QStringLiteral("XZ"), QStringLiteral("YZ")});
        else axis->addItems({QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z")});
        controls->addWidget(axis);
        auto* plot = new ComparisonPlot(page); plot->setObjectName(QStringLiteral("cdfComparisonPlot%1").arg(type));
        QVector<QCheckBox*> enabled;
        for(int i = 0; i < stages.size(); ++i) {
            auto* checkbox = new QCheckBox(stages[i].name, page);
            const bool available = (type != 1 && type != 2 && type != 5) || stages[i].timingValid;
            checkbox->setChecked(available); checkbox->setEnabled(available);
            if(!available) checkbox->setToolTip(QStringLiteral("\u65f6\u95f4\u6233\u975e\u9012\u589e\uff0c\u901f\u5ea6\u4e0e\u52a0\u901f\u5ea6\u66f2\u7ebf\u4e0d\u53ef\u7528\u3002"));
            checkbox->setStyleSheet(QStringLiteral("color:%1").arg(colors[i % colors.size()].name()));
            enabled.push_back(checkbox); controls->addWidget(checkbox);
        }
        QCheckBox* showReference = nullptr;
        if((type == 3 || type == 4) && !stages.empty() && !stages.front().referenceTcpPositions.empty()) {
            showReference = new QCheckBox(QStringLiteral("\u539f\u59cb\u63a7\u5236\u70b9\u6298\u7ebf"), page);
            showReference->setObjectName(QStringLiteral("cdfOriginalTcpReference"));
            showReference->setChecked(true); controls->addWidget(showReference);
        }
        controls->addStretch(); pageLayout->addLayout(controls); pageLayout->addWidget(plot, 1);
        const auto rebuild = [=]() {
            plot->curves.clear(); const int joint = axis->currentIndex();
            plot->equalAspect = type == 3;
            plot->xLabel = type == 0 || type == 4 ? QStringLiteral("\u5bf9\u5e94\u8f68\u8ff9\u8fdb\u5ea6 (%)") : type == 3 ? axis->currentText().left(1) + QStringLiteral(" (mm)") : QStringLiteral("\u8bb0\u5f55\u65f6\u95f4 (s)");
            plot->yLabel = type == 0 ? QStringLiteral("\u5173\u8282\u89d2 (deg)") : type == 1 ? QStringLiteral("\u5206\u6bb5\u7ebf\u6027\u5173\u8282\u901f\u5ea6 (deg/s)") : type == 2 ? QStringLiteral("\u8282\u70b9\u6709\u9650\u5dee\u5206\u52a0\u901f\u5ea6 (deg/s^2)") : type == 3 ? axis->currentText().right(1) + QStringLiteral(" (mm)") : type == 4 ? QStringLiteral("TCP \u5750\u6807 (mm)") : QStringLiteral("TCP \u6bb5\u901f\u5ea6 (mm/s)");
            for(int k = 0; k < stages.size(); ++k) {
                if(!enabled[k]->isChecked()) continue;
                const auto& s = stages[k]; ComparisonPlot::Curve curve{s.name, colors[k % colors.size()], {}};
                if(type == 1 && joint < s.segmentVelocities.size()) curve.points = s.segmentVelocities[joint];
                else for(int i = 0; i < s.times.size(); ++i) {
                    double x = s.times[i], y = std::numeric_limits<double>::quiet_NaN();
                    const double progress = s.times.size() <= 1 ? 0.0 : 100.0 * i / (s.times.size() - 1);
                    if(type == 0 && i < s.angles.size() && joint < s.angles[i].size()) { x = progress; y = s.angles[i][joint]; }
                    if(type == 2 && i < s.accelerations.size() && joint < s.accelerations[i].size()) y = s.accelerations[i][joint];
                    if(type == 3 && i < s.tcpPositions.size()) {
                        const int a = joint == 2 ? 1 : 0, b = joint == 0 ? 1 : 2;
                        x = s.tcpPositions[i][a]; y = s.tcpPositions[i][b];
                    }
                    if(type == 4 && i < s.tcpPositions.size()) { x = progress; y = s.tcpPositions[i][joint]; }
                    if(type == 5 && i > 0 && i < s.tcpPositions.size() && s.times[i] > s.times[i - 1]) {
                        double squared = 0.0; for(int j = 0; j < 3; ++j) squared += std::pow(s.tcpPositions[i][j] - s.tcpPositions[i - 1][j], 2);
                        y = std::sqrt(squared) / (s.times[i] - s.times[i - 1]);
                    }
                    curve.points.push_back({x, y});
                }
                plot->curves.push_back(std::move(curve));
            }
            if(showReference && showReference->isChecked()) {
                ComparisonPlot::Curve curve{showReference->text(), QColor(35, 175, 100), {}};
                const auto& positions = stages.front().referenceTcpPositions;
                for(int i = 0; i < positions.size(); ++i) {
                    if(type == 3) {
                        const int a = joint == 2 ? 1 : 0, b = joint == 0 ? 1 : 2;
                        curve.points.push_back({positions[i][a], positions[i][b]});
                    } else curve.points.push_back({positions.size() <= 1 ? 0.0 : 100.0*i/(positions.size()-1), positions[i][joint]});
                }
                plot->curves.push_back(std::move(curve));
            }
            plot->update();
        };
        connect(axis, QOverload<int>::of(&QComboBox::currentIndexChanged), plot, [=](int) { rebuild(); });
        for(auto* checkbox : enabled) connect(checkbox, &QCheckBox::toggled, plot, [=](bool) { rebuild(); });
        if(showReference) connect(showReference, &QCheckBox::toggled, plot, [=](bool) { rebuild(); });
        rebuild();
        tabs->addTab(page, QStringList{QStringLiteral("\u5173\u8282\u5f62\u72b6"), QStringLiteral("\u5173\u8282\u901f\u5ea6"), QStringLiteral("\u5173\u8282\u52a0\u901f\u5ea6"), QStringLiteral("TCP \u6295\u5f71"), QStringLiteral("TCP \u5750\u6807"), QStringLiteral("TCP \u901f\u5ea6")}[type]);
    }
    auto* log = new QPlainTextEdit(diagnostics, tabs); log->setReadOnly(true); tabs->addTab(log, QStringLiteral("\u6c42\u89e3\u4e0e\u9a8c\u6536\u65e5\u5fd7"));
    auto* exportButton = new QPushButton(QStringLiteral("\u5bfc\u51fa\u8d28\u91cf\u62a5\u544a CSV..."), this);
    exportButton->setObjectName(QStringLiteral("cdfQualityExport"));
    connect(exportButton, &QPushButton::clicked, this, &CdfTrajectoryAnalysisDialog::exportQualityRequested);
    layout->addWidget(exportButton);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, this); connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::close); layout->addWidget(buttons);
}
