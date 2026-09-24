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
    ConfigurationSelectionPlot(const QVector<QVector<int>>& sequences, QWidget* parent)
        : QWidget(parent), m_sequences(sequences)
    {
        setObjectName(QStringLiteral("configurationSelectionPlot"));
        setMinimumWidth(460);
        for(const auto& sequence : m_sequences) {
            for(int value : sequence) { m_maxCandidate = std::max(m_maxCandidate, value); }
        }
    }

    void setView(const QVector<int>& ranks, int first, int last, bool separate)
    {
        m_ranks = ranks; m_first = first; m_last = last; m_separate = separate;
        setMinimumHeight(separate ? std::max(1, ranks.size()) * 180 : 360);
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
            const QRectF plot(68, panel * panelHeight + 29, width() - 92, panelHeight - 79);
            const auto x = [&](int point) {
                return m_first == m_last ? plot.center().x() :
                    plot.left() + (point - m_first) * plot.width() / (m_last - m_first);
            };
            const auto y = [&](int candidate) {
                return plot.bottom() - (candidate - 1) * plot.height() / (m_maxCandidate - 1);
            };
            const int yStep = std::max(1, int(std::ceil((m_maxCandidate - 1) / 8.0)));
            for(int candidate = 1; candidate <= m_maxCandidate; candidate += yStep) {
                painter.setPen(palette().mid().color());
                painter.drawLine(QPointF(plot.left(), y(candidate)), QPointF(plot.right(), y(candidate)));
                painter.setPen(palette().text().color());
                painter.drawText(QRectF(30, y(candidate) - 10, 31, 20), Qt::AlignRight | Qt::AlignVCenter,
                    QString::number(candidate));
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
                QStringLiteral("\u672c\u70b9\u9006\u89e3\u7f16\u53f7"));
            painter.restore();
            const QVector<int> ranks = m_separate ? QVector<int>{m_ranks[panel]} : m_ranks;
            const QString title = m_separate ? QStringLiteral("\u7ed3\u679c #%1").arg(ranks.front() + 1) :
                QStringLiteral("\u6784\u578b\u9009\u62e9\u5bf9\u6bd4\uff08%1 \u6761\uff09").arg(ranks.size());
            painter.drawText(QRectF(plot.left(), plot.top() - 25, plot.width(), 20), Qt::AlignLeft, title);
            painter.save();
            painter.setClipRect(plot.adjusted(-4, -4, 4, 4));
            for(int rank : ranks) {
                const auto& sequence = m_sequences[rank];
                const int last = std::min(m_last, sequence.size() - 1);
                QPainterPath line;
                for(int point = m_first; point <= last; ++point) {
                    const QPointF position(x(point), y(sequence[point]));
                    if(point == m_first) { line.moveTo(position); }
                    else { line.lineTo(position); }
                }
                painter.setPen(QPen(rankColor(rank), 1.7, rankStyle(rank)));
                painter.setBrush(Qt::NoBrush);
                painter.drawPath(line);
                // Draw every sample, including isolated differences; no downsampling.
                painter.setBrush(rankColor(rank));
                for(int point = m_first; point <= last; ++point) {
                    const double radius = last - m_first < 60 ? 2.8 : 1.5;
                    painter.drawEllipse(QPointF(x(point), y(sequence[point])), radius, radius);
                }
            }
            painter.restore();
        }
    }

private:
    QVector<QVector<int>> m_sequences;
    QVector<int> m_ranks;
    int m_first = 0;
    int m_last = 0;
    int m_maxCandidate = 2;
    bool m_separate = false;
};

ConfigurationSelectionDialog::ConfigurationSelectionDialog(const QVector<QVector<int>>& sequences,
    int initialRank, QWidget* parent)
    : QDialog(parent, Qt::Window), m_sequences(sequences)
{
    setObjectName(QStringLiteral("configurationSelectionDialog"));
    setWindowTitle(QStringLiteral("\u67e5\u770b\u6784\u578b\u9009\u62e9"));
    setAttribute(Qt::WA_DeleteOnClose);
    resize(980, 590);
    auto* layout = new QVBoxLayout(this);
    auto* note = new QLabel(QStringLiteral("\u663e\u793a\u5f53\u524d\u5206\u5c42\u56fe Top-M \u5019\u9009\u3002\u9006\u89e3\u7f16\u53f7\u4ec5\u5728\u5f53\u524d\u63a7\u5236\u70b9\u5185\u6709\u6548\uff0c\u4e0d\u662f\u56fa\u5b9a\u7684\u80a9/\u8098/\u8155\u5206\u652f\u6807\u7b7e\uff1b\u76f8\u540c\u9009\u62e9\u5904\u66f2\u7ebf\u91cd\u5408\uff0c\u53ef\u5207\u6362\u5206\u884c\u5bf9\u6bd4\u3002"), this);
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
    layout->addLayout(controls);
    auto* body = new QHBoxLayout;
    auto* sidebar = new QVBoxLayout;
    sidebar->addWidget(new QLabel(QStringLiteral("\u52fe\u9009\u7ed3\u679c\u5e8f\u5217\u53f7\uff08\u6392\u540d\uff09"), this));
    m_ranks = new QListWidget(this); m_ranks->setObjectName(QStringLiteral("configurationRanks"));
    m_ranks->setMaximumWidth(230); m_ranks->setMinimumWidth(160);
    for(int rank = 0; rank < sequences.size(); ++rank) {
        QPixmap swatch(30, 14); swatch.fill(Qt::transparent);
        QPainter painter(&swatch); painter.setPen(QPen(rankColor(rank), 2, rankStyle(rank)));
        painter.drawLine(0, 7, 30, 7);
        auto* item = new QListWidgetItem(QIcon(swatch), QStringLiteral("\u7ed3\u679c #%1").arg(rank + 1), m_ranks);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(rank == 0 || rank == initialRank ? Qt::Checked : Qt::Unchecked);
    }
    sidebar->addWidget(m_ranks);
    auto* selectionButtons = new QHBoxLayout;
    auto* all = new QPushButton(QStringLiteral("\u5168\u9009"), this); all->setObjectName(QStringLiteral("configurationSelectAll"));
    auto* none = new QPushButton(QStringLiteral("\u6e05\u7a7a"), this); none->setObjectName(QStringLiteral("configurationSelectNone"));
    selectionButtons->addWidget(all); selectionButtons->addWidget(none); sidebar->addLayout(selectionButtons);
    body->addLayout(sidebar);
    auto* scroll = new QScrollArea(this); scroll->setWidgetResizable(true);
    m_plot = new ConfigurationSelectionPlot(m_sequences, scroll); scroll->setWidget(m_plot);
    body->addWidget(scroll, 1); layout->addLayout(body, 1);
    m_summary = new QLabel(this); m_summary->setObjectName(QStringLiteral("configurationSummary"));
    m_summary->setWordWrap(true); layout->addWidget(m_summary);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::close); layout->addWidget(buttons);
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
    refreshPlot();
}

QVector<int> ConfigurationSelectionDialog::selectedRanks() const
{
    QVector<int> ranks;
    for(int i = 0; i < m_ranks->count(); ++i) {
        if(m_ranks->item(i)->checkState() == Qt::Checked) { ranks.push_back(i); }
    }
    return ranks;
}

void ConfigurationSelectionDialog::refreshPlot()
{
    const auto ranks = selectedRanks();
    const int first = m_first->value() - 1, last = m_last->value() - 1;
    int different = 0;
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
    m_plot->setView(ranks, first, last, m_separate->isChecked());
}
