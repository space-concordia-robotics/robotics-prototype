#ifndef FILESETTINGS_WINDOW_H
#define FILESETTINGS_WINDOW_H

#include <QDialog>
//#include "mainwindow.h"
namespace Ui {
class Filesettings_window;
}

class Filesettings_window : public QDialog
{
    Q_OBJECT

public:
    explicit Filesettings_window(QWidget *parent = 0);
    ~Filesettings_window();
private slots:
    void on_buttonBox_accepted();
    void on_buttonBox_rejected();

    void on_combinedcsvfiles_clicked(bool checked);

    void on_seperatedcsvfiles_clicked(bool checked);

    void on_absorbance_csv_clicked(bool checked);

    void on_intensity_csv_clicked(bool checked);

    void on_reflectance_csv_clicked(bool checked);

    void on_save_JCAMP_clicked(bool checked);

    void on_absorbance_JCAMP_clicked(bool checked);

    void on_intensity_JCAMP_clicked(bool checked);

    void on_reflectance_JCAMP_clicked(bool checked);

    void on_Filesettings_window_finished(void);

    void on_savebinary_clicked(bool checked);

private:
    Ui::Filesettings_window *ui;
};

#endif // FILESETTINGS_WINDOW_H
