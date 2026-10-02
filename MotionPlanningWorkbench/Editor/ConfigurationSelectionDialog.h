#pragma once

#include <QDialog>
#include <QVector>
#include <QStringList>

class QListWidget;
class QSpinBox;
class QCheckBox;
class QLabel;
class ConfigurationSelectionPlot;
class QTabWidget;

// Shared projection of domain branches; rows are control points, columns candidates.
// 1..8 are stable shoulder/elbow/wrist categories, 9 is boundary/unclassified.
struct ConfigurationSelectionCatalog
{
    QVector<QVector<int>> categories;
    QVector<QStringList> candidateDetails;
    QStringList categoryLabels;
};

// Preserve one-based candidate identities for playback and turn comparisons.
class ConfigurationSelectionPage : public QWidget
{
public:
    ConfigurationSelectionPage(const QVector<QVector<int>>& sequences, const QStringList& labels,
        const QString& explanation, int initialRank, bool selectStartBest, QWidget* parent,
        const ConfigurationSelectionCatalog& catalog = {});
    const QVector<QVector<int>>& sequences() const { return m_sequences; }
    QVector<int> selectedRanks() const;

private:
    void refreshPlot();
    QVector<QVector<int>> m_sequences;
    QVector<QVector<int>> m_branchSequences;
    QListWidget* m_ranks = nullptr;
    QSpinBox* m_first = nullptr;
    QSpinBox* m_last = nullptr;
    QCheckBox* m_separate = nullptr;
    QCheckBox* m_byBranch = nullptr;
    QLabel* m_summary = nullptr;
    ConfigurationSelectionPlot* m_plot = nullptr;
};

class ConfigurationSelectionDialog : public QDialog
{
    Q_OBJECT
public:
    explicit ConfigurationSelectionDialog(const QVector<QVector<int>>& sequences,
        int initialRank, QWidget* parent = nullptr);
    ConfigurationSelectionDialog(const QVector<QVector<int>>& globalSequences,
        const QVector<QVector<int>>& startSequences, const QStringList& startLabels,
        int initialRank, bool byStart, QWidget* parent = nullptr,
        const ConfigurationSelectionCatalog& catalog = {});
    const QVector<QVector<int>>& sequences(int page = 0) const;
    QVector<int> selectedRanks(int page = 0) const;
    void selectPage(bool byStart);

private:
    QTabWidget* m_tabs = nullptr;
    ConfigurationSelectionPage* m_pages[2] = {};
};
