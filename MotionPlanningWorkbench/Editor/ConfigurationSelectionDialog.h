#pragma once

#include <QDialog>
#include <QVector>

class QListWidget;
class QSpinBox;
class QCheckBox;
class QLabel;
class ConfigurationSelectionPlot;

// Read-only, one-based candidate numbers projected from the ranked domain paths.
class ConfigurationSelectionDialog : public QDialog
{
    Q_OBJECT
public:
    explicit ConfigurationSelectionDialog(const QVector<QVector<int>>& sequences,
        int initialRank, QWidget* parent = nullptr);
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
