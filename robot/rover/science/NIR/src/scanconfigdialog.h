/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**  ALL RIGHTS RESERVED
**
******************************************************************************/

#ifndef SCANCONFIGDIALOG_H
#define SCANCONFIGDIALOG_H

#include <QDialog>
#include "dlpspec_scan.h"
#include "scanconfiglist.h"
#include "evm.h"
#include "QTableWidgetItem"
#include "QTableWidget"

namespace Ui {
class ScanConfigDialog;
}

class ScanConfigDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ScanConfigDialog(QWidget *parent = 0);
    ~ScanConfigDialog();

public slots:
    void Init(ScanConfigList* list);

private slots:
    void on_buttonBox_accepted();

    void on_buttonBox_rejected();

    void on_listWidget_local_scan_cfg_currentRowChanged(int currentRow);

    void on_pushButton_edit_local_clicked();

    void on_pushButton_new_local_clicked();

    void on_pushButton_new_target_clicked();

    void on_pushButton_scan_config_save_clicked();

    void on_listWidget_target_scan_cfg_currentRowChanged(int currentRow);

    void on_pushButton_edit_target_clicked();

    void on_pushButton_delete_local_clicked();

    void on_pushButton_delete_target_clicked();

    void on_pushButton_copy_right_clicked();

    void on_pushButton_move_right_clicked();

    void on_pushButton_copy_left_clicked();

    void on_pushButton_move_left_clicked();

    bool IsDuplicateName(QString name, QList<uScanConfig> list_scan_cfg, int edit_index = -1);

    void PopulateScanSectionGrid(int numsections);

    void on_listWidget_target_scan_cfg_clicked(const QModelIndex &index);

    void on_listWidget_local_scan_cfg_clicked(const QModelIndex &index);

    void on_pushButton_set_active_clicked();

    bool eventFilter(QObject *target, QEvent *event);

    void on_spinBox_numSections_editingFinished();

private:
    void showError(const char *str);
    void clear_editBoxes();
    void enable_editBoxes(bool enable);
    int getMaxPatterns(int startnm, int endnm, double width, int scan_type);
    void PopulateListWidgets(void);
    void DisplayScanConfigItem(uScanConfig uCfg);
    Ui::ScanConfigDialog *ui;

    ScanConfigList* scancfglist;
    /* Temporary lists. Will be applied to ScanConfigList only when user hits OK button to close the dialog */
    QList<uScanConfig> temp_local_list;
    QList<uScanConfig> temp_target_list;
    bool save_to_target;
    bool edit_existing;
    int edit_index;
    calibCoeffs m_calibCoeffs;
    bool target_connected;
    int m_activeConigIndex;

    int GetWidthIndex(double width);
};

#endif // SCANCONFIGDIALOG_H
