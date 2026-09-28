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

// Read-only, one-based candidate numbers projected from the ranked domain paths.
class ConfigurationSelectionPage : public QWidget
{
public:
    ConfigurationSelectionPage(const QVector<QVector<int>>& sequences, const QStringList& labels,
        const QString& explanation, int initialRank, bool selectStartBest, QWidget* parent);
    const QVector<QVector<int>>& sequences() const { return m_sequences; }
    QVector<int> selectedRanks() const;

private:
    void refreshPlot();
    QVector<QVector<int>> m_sequences;
    QListWidget* m_ranks = nullptr;
    QSpinBox* m_first = nullptr;
    QSpinBox* m_last = nullptr;
    QCheckBox* m_separate = nullptr;
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
        int initialRank, bool byStart, QWidget* parent = nullptr);
    const QVector<QVector<int>>& sequences(int page = 0) const;
    QVector<int> selectedRanks(int page = 0) const;
    void selectPage(bool byStart);

private:
    QTabWidget* m_tabs = nullptr;
    ConfigurationSelectionPage* m_pages[2] = {};
};
