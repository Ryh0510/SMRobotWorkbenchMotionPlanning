#include "ConfigurationSelectionDialog.h"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QIcon>
#include <QPixmap>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QTabWidget>
#include <QSet>
#include <QMouseEvent>
#include <QToolTip>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace
{
    QColor rankColor(int rank)
    {
        return QColor::fromHsv((rank * 137 + 210) % 360, 190, 185);
    }

    Qt::PenStyle rankStyle(int rank)
    {
        const Qt::PenStyle styles[] = {Qt::SolidLine, Qt::DashLine, Qt::DotLine, Qt::DashDotLine};
        return styles[(rank / 10) % 4];
    }
}

class ConfigurationSelectionPlot : public QWidget
{
public:
    ConfigurationSelectionPlot(const QVector<QVector<int>>& sequences, const QStringList& labels, QWidget* parent,
        const ConfigurationSelectionCatalog& catalog)
        : QWidget(parent), m_sequences(sequences), m_labels(labels), m_catalog(catalog)
    {
        setObjectName(QStringLiteral("configurationSelectionPlot"));
        setMinimumWidth(580);
        setMouseTracking(true);
        for(const auto& sequence : m_sequences) {
            for(int value : sequence) { m_maxCandidate = std::max(m_maxCandidate, value); }
        }
    }

    void setView(const QVector<int>& ranks, int first, int last, bool separate, bool byBranch)
    {
        m_ranks = ranks; m_first = first; m_last = last; m_separate = separate;
        m_byBranch = byBranch;
        setMinimumHeight(separate ? std::max(1, ranks.size()) * (byBranch ? 320 : 180) : (byBranch ? 390 : 360));
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.fillRect(rect(), palette().base());
        painter.setPen(palette().text().color());
        if(m_ranks.isEmpty()) {
            painter.drawText(rect(), Qt::AlignCenter, QStringLiteral("\u8bf7\u52fe\u9009\u8981\u5bf9\u6bd4\u7684\u7ed3\u679c\u5e8f\u5217\u53f7"));
            return;
        }
        const int panels = m_separate ? m_ranks.size() : 1;
        const double panelHeight = double(height()) / panels;
        for(int panel = 0; panel < panels; ++panel) {
            const double left = plotLeft();
            const QRectF plot(left, panel * panelHeight + 29, width() - left - 24, panelHeight - 79);
            const int maximum = m_byBranch ? m_catalog.categoryLabels.size() : m_maxCandidate;
            const auto x = [&](int point) {
                return m_first == m_last ? plot.center().x() :
                    plot.left() + (point - m_first) * plot.width() / (m_last - m_first);
            };
            const auto y = [&](int candidate) {
                return plot.bottom() - (candidate - 1) * plot.height() / std::max(1, maximum - 1);
            };
            const int yStep = std::max(1, int(std::ceil((maximum - 1) / 8.0)));
            for(int candidate = 1; candidate <= maximum; candidate += yStep) {
                painter.setPen(palette().mid().color());
                painter.drawLine(QPointF(plot.left(), y(candidate)), QPointF(plot.right(), y(candidate)));
                painter.setPen(palette().text().color());
                painter.drawText(QRectF(30, y(candidate) - 10, left - 37, 20), Qt::AlignRight | Qt::AlignVCenter,
                    m_byBranch ? m_catalog.categoryLabels.value(candidate - 1) : QString::number(candidate));
            }
            const int ticks = std::min(6, m_last - m_first);
            for(int tick = 0; tick <= ticks; ++tick) {
                const int point = ticks == 0 ? m_first : m_first + int(std::lround(double(tick) * (m_last - m_first) / ticks));
                painter.setPen(palette().mid().color());
                painter.drawLine(QPointF(x(point), plot.top()), QPointF(x(point), plot.bottom()));
                painter.setPen(palette().text().color());
                painter.drawText(QRectF(x(point) - 30, plot.bottom() + 5, 60, 20), Qt::AlignCenter,
                    QString::number(point + 1));
            }
            painter.setPen(palette().text().color());
            painter.drawRect(plot);
            painter.drawText(QRectF(plot.left(), plot.bottom() + 26, plot.width(), 20), Qt::AlignCenter,
                QStringLiteral("\u63a7\u5236\u70b9\u5e8f\u53f7"));
            painter.save();
            painter.translate(18, plot.center().y()); painter.rotate(-90);
            painter.drawText(QRectF(-plot.height()/2, -10, plot.height(), 20), Qt::AlignCenter,
                m_byBranch ? QStringLiteral("\u56fa\u5b9a\u6784\u578b\u5206\u652f") : QStringLiteral("\u672c\u70b9\u5019\u9009\u5e8f\u53f7"));
            painter.restore();
            const QVector<int> ranks = m_separate ? QVector<int>{m_ranks[panel]} : m_ranks;
            const QString title = m_separate ? m_labels.value(ranks.front(), QStringLiteral("#%1").arg(ranks.front() + 1)) :
                QStringLiteral("\u6784\u578b\u9009\u62e9\u5bf9\u6bd4\uff08%1 \u6761\uff09").arg(ranks.size());
            painter.drawText(QRectF(plot.left(), plot.top() - 25, plot.width(), 20), Qt::AlignLeft,
                painter.fontMetrics().elidedText(title, Qt::ElideRight, int(plot.width())));
            painter.save();
            painter.setClipRect(plot.adjusted(-4, -4, 4, 4));
            for(int rank : ranks) {
                const auto& sequence = m_sequences[rank];
                const int last = std::min(m_last, sequence.size() - 1);
                QPainterPath line;
                for(int point = m_first; point <= last; ++point) {
                    const QPointF position(x(point), y(category(point, sequence[point])));
                    if(point == m_first) { line.moveTo(position); }
                    else if(m_byBranch) {
                        line.lineTo(QPointF(position.x(), line.currentPosition().y()));
                        line.lineTo(position);
                    } else { line.lineTo(position); }
                }
                painter.setPen(QPen(rankColor(rank), 1.7, rankStyle(rank)));
                painter.setBrush(Qt::NoBrush);
                painter.drawPath(line);
                // Draw every sample, including isolated differences; no downsampling.
                painter.setBrush(rankColor(rank));
                for(int point = m_first; point <= last; ++point) {
                    const double radius = last - m_first < 60 ? 2.8 : 1.5;
                    painter.drawEllipse(QPointF(x(point), y(category(point, sequence[point]))), radius, radius);
                }
            }
            painter.restore();
        }
    }

    void mouseMoveEvent(QMouseEvent* event) override
    {
        const double left = plotLeft();
        if(m_ranks.isEmpty() || event->pos().x() < left || event->pos().x() > width() - 24) {
            QToolTip::hideText(); return;
        }
        const int point = std::clamp(m_first + int(std::lround((event->pos().x() - left) *
            (m_last - m_first) / std::max(1.0, width() - left - 24))), m_first, m_last);
        const int panel = std::clamp(event->pos().y() * m_ranks.size() / std::max(1, height()), 0, m_ranks.size() - 1);
        const auto ranks = m_separate ? QVector<int>{m_ranks[panel]} : m_ranks;
        QStringList lines{QStringLiteral("\u63a7\u5236\u70b9 %1").arg(point + 1)};
        for(int rank : ranks) {
            if(point >= m_sequences[rank].size()) { continue; }
            const int candidate = m_sequences[rank][point];
            const auto details = point < m_catalog.candidateDetails.size() ?
                m_catalog.candidateDetails[point].value(candidate - 1) : QString::number(candidate);
            lines << m_labels.value(rank) + QStringLiteral(": ") + details;
            if(lines.size() >= 13) { lines << QStringLiteral("..."); break; }
        }
        QToolTip::showText(event->globalPos(), lines.join(QStringLiteral("\n")), this);
    }

private:
    double plotLeft() const
    {
        if(!m_byBranch) { return 68; }
        int labelWidth = 0;
        for(const auto& label : m_catalog.categoryLabels) {
            labelWidth = std::max(labelWidth, fontMetrics().horizontalAdvance(label));
        }
        return labelWidth + 45;
    }

    int category(int point, int candidate) const
    {
        return m_byBranch && point < m_catalog.categories.size() ?
            m_catalog.categories[point].value(candidate - 1, 9) : candidate;
    }
    QVector<QVector<int>> m_sequences;
    QStringList m_labels;
    QVector<int> m_ranks;
    int m_first = 0;
    int m_last = 0;
    int m_maxCandidate = 2;
    bool m_separate = false;
    bool m_byBranch = false;
    ConfigurationSelectionCatalog m_catalog;
};

ConfigurationSelectionPage::ConfigurationSelectionPage(const QVector<QVector<int>>& sequences,
    const QStringList& labels, const QString& explanation, int initialRank, bool selectStartBest, QWidget* parent,
    const ConfigurationSelectionCatalog& catalog)
    : QWidget(parent), m_sequences(sequences)
{
    m_branchSequences = m_sequences;
    for(auto& sequence : m_branchSequences) {
        for(int i = 0; i < sequence.size(); ++i) {
            sequence[i] = i < catalog.categories.size() ? catalog.categories[i].value(sequence[i] - 1, 9) : sequence[i];
        }
    }
    auto* layout = new QVBoxLayout(this);
    auto* note = new QLabel(explanation, this);
    note->setWordWrap(true); layout->addWidget(note);
    auto* controls = new QHBoxLayout;
    int count = 1;
    for(const auto& sequence : m_sequences) { count = std::max(count, sequence.size()); }
    m_first = new QSpinBox(this); m_first->setObjectName(QStringLiteral("configurationFirstPoint"));
    m_last = new QSpinBox(this); m_last->setObjectName(QStringLiteral("configurationLastPoint"));
    m_first->setRange(1, count); m_last->setRange(1, count); m_last->setValue(count);
    controls->addWidget(new QLabel(QStringLiteral("\u63a7\u5236\u70b9\u533a\u95f4"), this));
    controls->addWidget(m_first); controls->addWidget(new QLabel(QStringLiteral("\u81f3"), this)); controls->addWidget(m_last);
    auto* full = new QPushButton(QStringLiteral("\u5168\u7a0b"), this); full->setObjectName(QStringLiteral("configurationFullRange"));
    controls->addWidget(full);
    m_separate = new QCheckBox(QStringLiteral("\u5206\u884c\u5bf9\u6bd4"), this);
    m_separate->setObjectName(QStringLiteral("configurationSeparate")); controls->addWidget(m_separate); controls->addStretch();
    m_byBranch = new QCheckBox(QStringLiteral("\u6309\u80a9/\u8098/\u8155\u663e\u793a"), this);
    m_byBranch->setObjectName(QStringLiteral("configurationByBranch"));
    m_byBranch->setEnabled(!catalog.categories.isEmpty());
    m_byBranch->setChecked(!catalog.categories.isEmpty());
    controls->addWidget(m_byBranch);
    layout->addLayout(controls);
    auto* body = new QHBoxLayout;
    auto* sidebar = new QVBoxLayout;
    sidebar->addWidget(new QLabel(QStringLiteral("\u52fe\u9009\u7ed3\u679c\u5e8f\u5217\u53f7\uff08\u6392\u540d\uff09"), this));
    m_ranks = new QListWidget(this); m_ranks->setObjectName(QStringLiteral("configurationRanks"));
    m_ranks->setMaximumWidth(380); m_ranks->setMinimumWidth(selectStartBest ? 280 : 160);
    m_ranks->setWordWrap(true);
    m_ranks->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    QSet<int> checkedStarts;
    for(int rank = 0; rank < sequences.size(); ++rank) {
        QPixmap swatch(30, 14); swatch.fill(Qt::transparent);
        QPainter painter(&swatch); painter.setPen(QPen(rankColor(rank), 2, rankStyle(rank)));
        painter.drawLine(0, 7, 30, 7);
        painter.end();
        const auto label = labels.value(rank, QStringLiteral("#%1").arg(rank + 1));
        auto displayLabel = label;
        const int separator = displayLabel.contains(QStringLiteral(" | ")) ?
            displayLabel.indexOf(QStringLiteral(" | ")) : displayLabel.lastIndexOf(QStringLiteral(" / "));
        if(selectStartBest && separator >= 0) { displayLabel.replace(separator, 3, QStringLiteral("\n")); }
        auto* item = new QListWidgetItem(QIcon(swatch), displayLabel, m_ranks);
        item->setToolTip(label);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        const int start = sequences[rank].isEmpty() ? -1 : sequences[rank].front();
        const bool checked = selectStartBest ? !checkedStarts.contains(start) || rank == initialRank : rank == 0 || rank == initialRank;
        item->setCheckState(checked ? Qt::Checked : Qt::Unchecked);
        checkedStarts.insert(start);
    }
    sidebar->addWidget(m_ranks);
    auto* selectionButtons = new QHBoxLayout;
    auto* all = new QPushButton(QStringLiteral("\u5168\u9009"), this); all->setObjectName(QStringLiteral("configurationSelectAll"));
    auto* none = new QPushButton(QStringLiteral("\u6e05\u7a7a"), this); none->setObjectName(QStringLiteral("configurationSelectNone"));
    selectionButtons->addWidget(all); selectionButtons->addWidget(none); sidebar->addLayout(selectionButtons);
    body->addLayout(sidebar);
    auto* scroll = new QScrollArea(this); scroll->setWidgetResizable(true);
    m_plot = new ConfigurationSelectionPlot(m_sequences, labels, scroll, catalog); scroll->setWidget(m_plot);
    body->addWidget(scroll, 1); layout->addLayout(body, 1);
    m_summary = new QLabel(this); m_summary->setObjectName(QStringLiteral("configurationSummary"));
    m_summary->setWordWrap(true); layout->addWidget(m_summary);
    connect(m_ranks, &QListWidget::itemChanged, this, [this]() { refreshPlot(); });
    const auto selectAll = [this](Qt::CheckState state) {
        const QSignalBlocker blocker(m_ranks);
        for(int i = 0; i < m_ranks->count(); ++i) { m_ranks->item(i)->setCheckState(state); }
        refreshPlot();
    };
    connect(all, &QPushButton::clicked, this, [selectAll]() { selectAll(Qt::Checked); });
    connect(none, &QPushButton::clicked, this, [selectAll]() { selectAll(Qt::Unchecked); });
    connect(m_first, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value) {
        if(value > m_last->value()) { const QSignalBlocker blocker(m_last); m_last->setValue(value); }
        refreshPlot();
    });
    connect(m_last, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value) {
        if(value < m_first->value()) { const QSignalBlocker blocker(m_first); m_first->setValue(value); }
        refreshPlot();
    });
    connect(full, &QPushButton::clicked, this, [this, count]() {
        const QSignalBlocker first(m_first), last(m_last);
        m_first->setValue(1); m_last->setValue(count); refreshPlot();
    });
    connect(m_separate, &QCheckBox::toggled, this, [this]() { refreshPlot(); });
    connect(m_byBranch, &QCheckBox::toggled, this, [this]() { refreshPlot(); });
    refreshPlot();
}

QVector<int> ConfigurationSelectionPage::selectedRanks() const
{
    QVector<int> ranks;
    for(int i = 0; i < m_ranks->count(); ++i) {
        if(m_ranks->item(i)->checkState() == Qt::Checked) { ranks.push_back(i); }
    }
    return ranks;
}

void ConfigurationSelectionPage::refreshPlot()
{
    const auto ranks = selectedRanks();
    const int first = m_first->value() - 1, last = m_last->value() - 1;
    int different = 0, branchDifferent = 0;
    for(int point = first; point <= last && !ranks.isEmpty(); ++point) {
        const auto& reference = m_sequences[ranks.front()];
        for(int rank : ranks) {
            const auto& sequence = m_sequences[rank];
            if(point < reference.size() && point < sequence.size() && reference[point] != sequence[point]) {
                ++different; break;
            }
        }
    }
    m_summary->setText(QStringLiteral("\u5df2\u9009 %1 / %2 \u6761\uff1b\u63a7\u5236\u70b9 %3\u2013%4\uff1b\u5176\u4e2d %5 \u4e2a\u70b9\u7684\u9006\u89e3\u9009\u62e9\u5b58\u5728\u5dee\u5f02\u3002")
        .arg(ranks.size()).arg(m_sequences.size()).arg(first + 1).arg(last + 1).arg(different));
    if(m_byBranch->isEnabled()) {
        for(int point = first; point <= last && !ranks.isEmpty(); ++point) {
            for(int rank : ranks) {
                if(point < m_branchSequences[rank].size() && point < m_branchSequences[ranks.front()].size() &&
                    m_branchSequences[rank][point] != m_branchSequences[ranks.front()][point]) { ++branchDifferent; break; }
            }
        }
        m_summary->setText(m_summary->text() + QStringLiteral(" \u6784\u578b\u5206\u652f\u4e0d\u540c: %1 \u70b9\u3002\u60ac\u505c\u67e5\u770b\u6784\u578b\u4e0e turn\uff1b\u53d6\u6d88\u6309\u5206\u652f\u663e\u793a\u53ef\u5bf9\u6bd4\u540c\u5206\u652f\u4e0d\u540c\u5019\u9009\u3002").arg(branchDifferent));
    }
    m_plot->setView(ranks, first, last, m_separate->isChecked(), m_byBranch->isChecked());
}

ConfigurationSelectionDialog::ConfigurationSelectionDialog(const QVector<QVector<int>>& sequences,
    int initialRank, QWidget* parent)
    : ConfigurationSelectionDialog(sequences, {}, {}, initialRank, false, parent)
{
}

ConfigurationSelectionDialog::ConfigurationSelectionDialog(const QVector<QVector<int>>& globalSequences,
    const QVector<QVector<int>>& startSequences, const QStringList& startLabels,
    int initialRank, bool byStart, QWidget* parent, const ConfigurationSelectionCatalog& catalog)
    : QDialog(parent, Qt::Window)
{
    setObjectName(QStringLiteral("configurationSelectionDialog"));
    setWindowTitle(QStringLiteral("\u67e5\u770b\u6784\u578b\u9009\u62e9"));
    setAttribute(Qt::WA_DeleteOnClose);
    resize(1180, 760);
    auto* layout = new QVBoxLayout(this);
    m_tabs = new QTabWidget(this); m_tabs->setObjectName(QStringLiteral("configurationSelectionTabs"));
    QStringList globalLabels;
    for(int i = 0; i < globalSequences.size(); ++i) {
        QString label = QStringLiteral("\u5168\u5c40 #%1").arg(i + 1);
        if(!globalSequences[i].isEmpty() && !catalog.categories.isEmpty()) {
            const int category = catalog.categories.front().value(globalSequences[i].front() - 1, 9);
            label += QStringLiteral(" | \u8d77\u70b9 ") + catalog.categoryLabels.value(category - 1);
        }
        globalLabels << label;
    }
    const auto note = catalog.categories.isEmpty() ? QStringLiteral("\u5019\u9009\u5e8f\u53f7\u4ec5\u5728\u5f53\u524d\u70b9\u6709\u6548\u3002") : QStringLiteral("B1\uff5eB8 \u4e3a\u56fa\u5b9a\u80a9/\u8098/\u8155\u5206\u652f\uff08+ \u5728\u524d\uff0c- \u5728\u540e\uff09\uff0c\u4e0d\u662f\u5173\u8282\u89d2\u6b63\u8d1f\u6216 ABB confdata\u3002\u7f3a\u89e3\u4e0d\u91cd\u7f16\u53f7\uff0cturn \u5355\u72ec\u4fdd\u7559\u3002");
    m_pages[0] = new ConfigurationSelectionPage(globalSequences, globalLabels,
        QStringLiteral("\u5168\u5c40 Top-M\uff1a\u4e0d\u9650\u5236\u8d77\u70b9\uff0c\u5728\u6240\u6709\u5b8c\u6574\u5e8f\u5217\u4e2d\u6309\u603b\u4ee3\u4ef7\u6392\u540d\uff0c\u53ef\u80fd\u6765\u81ea\u540c\u4e00\u8d77\u70b9\u3002") + note,
        byStart ? 0 : initialRank, false, m_tabs, catalog);
    m_pages[1] = new ConfigurationSelectionPage(startSequences, startLabels,
        QStringLiteral("\u6309\u8d77\u70b9 Top-K\uff1a\u56fa\u5b9a\u6bcf\u4e00\u4e2a\u8d77\u70b9\u9006\u89e3\uff0c\u72ec\u7acb\u6c42\u8be5\u8d77\u70b9\u7684\u524d K \u6761\u5b8c\u6574\u8f68\u8ff9\uff1b\u9ed8\u8ba4\u52fe\u9009\u5404\u7ec4\u6700\u4f18\u3002\u5168\u5c40 >M \u8868\u793a\u672a\u8fdb\u5165\u7b2c\u4e00\u9875\u524d M \u6761\uff0c\u5177\u4f53\u540d\u6b21\u672a\u8ba1\u7b97\u3002") + note,
        byStart ? initialRank : -1, true, m_tabs, catalog);
    m_pages[0]->setObjectName(QStringLiteral("configurationGlobalPage"));
    m_pages[1]->setObjectName(QStringLiteral("configurationStartPage"));
    m_tabs->addTab(m_pages[0], QStringLiteral("\u5168\u5c40 Top-M"));
    m_tabs->addTab(m_pages[1], QStringLiteral("\u6309\u8d77\u70b9 Top-K"));
    selectPage(byStart);
    layout->addWidget(m_tabs);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::close); layout->addWidget(buttons);
}

const QVector<QVector<int>>& ConfigurationSelectionDialog::sequences(int page) const
{
    return m_pages[page == 1 ? 1 : 0]->sequences();
}

QVector<int> ConfigurationSelectionDialog::selectedRanks(int page) const
{
    return m_pages[page == 1 ? 1 : 0]->selectedRanks();
}

void ConfigurationSelectionDialog::selectPage(bool byStart)
{
    m_tabs->setCurrentIndex(byStart ? 1 : 0);
}
