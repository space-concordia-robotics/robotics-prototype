#ifndef VERSIONDIALOG_H
#define VERSIONDIALOG_H

#include <QDialog>

namespace Ui {
class versiondialog;
}

class versiondialog : public QDialog
{
    Q_OBJECT

public:
    explicit versiondialog(QWidget *parent = 0);
    ~versiondialog();
    void setTivaVersion(QString str);
    void setDlpcVersion(QString str);
    void setSpeclibVersion(QString str);
    void setEEPROMcalVersion(QString str);
    void setRefcalVersion(QString str);
    void setEEPROMcfgVersion(QString str);
    void setTivaVersionRed();
    void setDLPCVersionRed();
    void setSpeclibVersionRed();
    void setEEPROMcalVersionRed();
    void setRefcalVersionRed();
    void setEEPROMcfgVersionRed();
    void showEEPROMVersions(bool show);

private slots:
    void on_Close_clicked();

private:
    Ui::versiondialog *ui;
    bool m_showEEPROMVersion;
};

#endif // VERSIONDIALOG_H
