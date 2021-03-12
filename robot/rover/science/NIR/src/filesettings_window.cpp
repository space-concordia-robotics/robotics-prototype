#include "filesettings_window.h"
#include "spectrum.h"
#include "mainwindow.h"
#include "ui_filesettings_window.h"
MainWindow *w;
Filesettings_window::Filesettings_window(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::Filesettings_window)
{
	ui->setupUi(this);
	w = (MainWindow *)parent;
	ui->save_JCAMP->setChecked(w->ifsave_JCAMP());


	ui->combinedcsvfiles->setChecked(w->ifsaveone_csv());
	ui->seperatedcsvfiles->setChecked(w->ifsavesep_csv());

    ui->savebinary->setChecked(w->ifsavebinary());

	ui->absorbance_csv->setDisabled(!w->ifsavesep_csv());
	ui->intensity_csv->setDisabled(!w->ifsavesep_csv());
	ui->reflectance_csv->setDisabled(!w->ifsavesep_csv());

	ui->absorbance_csv->setChecked(w->ifabsorbance_csv());
	ui->intensity_csv->setChecked(w->ifintensity_csv());
	ui->reflectance_csv->setChecked(w->ifreflectance_csv());

	ui->absorbance_JCAMP->setDisabled(!w->ifsave_JCAMP());
	ui->intensity_JCAMP->setDisabled(!w->ifsave_JCAMP());
	ui->reflectance_JCAMP->setDisabled(!w->ifsave_JCAMP());

	ui->absorbance_JCAMP->setChecked(w->ifabsorbance_JCAMP());
	ui->intensity_JCAMP->setChecked(w->ifintensity_JCAMP());
	ui->reflectance_JCAMP->setChecked(w->ifreflectance_JCAMP());


}

Filesettings_window::~Filesettings_window()
{
	on_buttonBox_rejected();
	delete ui;
}


void Filesettings_window::on_buttonBox_accepted()
{
	w->putsavesep_csv( w->saveseperate_csv);
	w->putsaveone_csv(w->saveone_csv );
	w->putabsorbance_csv(w->absorbance_csv);
	w->putintensity_csv(w->intensity_csv);
	w->putreflectance_csv(w->reflectance_csv);
	w->putsave_JCAMP(w->save_JCAMP);
	w->putabsorbance_JCAMP(w->absorbance_JCAMP);
	w->putintensity_JCAMP(w->intensity_JCAMP);
	w->putreflectance_JCAMP(w->reflectance_JCAMP);
    w->putsavebinary(w->savebinary);
	this->destroy(true,true);

}

void Filesettings_window::on_buttonBox_rejected()
{
	w->saveseperate_csv = w->ifsavesep_csv();
	w->saveone_csv =w->ifsaveone_csv();
	w->save_JCAMP = w->ifsave_JCAMP();
	w->absorbance_csv= w->ifabsorbance_csv();
	w->absorbance_JCAMP=w->ifabsorbance_JCAMP();
	w->intensity_csv=w->ifintensity_csv();
	w->intensity_JCAMP=w->ifintensity_JCAMP();
	w->reflectance_csv=w->ifreflectance_csv();
	w->reflectance_JCAMP=w->ifreflectance_JCAMP();
    w->savebinary = w->ifsavebinary();
	this->destroy(true,true);
}



void Filesettings_window::on_combinedcsvfiles_clicked(bool checked)
{
	w->saveone_csv = checked;
}

void Filesettings_window::on_seperatedcsvfiles_clicked(bool checked)
{
	w->saveseperate_csv = checked;
	if(!checked)
	{
		ui->absorbance_csv->setDisabled(true);
		ui->intensity_csv->setDisabled(true);
		ui->reflectance_csv->setDisabled(true);
	}
	else
	{
		ui->absorbance_csv->setDisabled(false);
		ui->intensity_csv->setDisabled(false);
		ui->reflectance_csv->setDisabled(false);
		ui->absorbance_csv->setChecked(w->ifabsorbance_csv());
		ui->intensity_csv->setChecked(w->ifintensity_csv());
		ui->reflectance_csv->setChecked(w->ifreflectance_csv());
	}
}

void Filesettings_window::on_absorbance_csv_clicked(bool checked)
{
	w->absorbance_csv= checked;
}

void Filesettings_window::on_intensity_csv_clicked(bool checked)
{
	w->intensity_csv= checked;
}

void Filesettings_window::on_reflectance_csv_clicked(bool checked)
{
	w->reflectance_csv= checked;
}

void Filesettings_window::on_save_JCAMP_clicked(bool checked)
{
	w->save_JCAMP=checked;
	if(!checked)
	{
		ui->absorbance_JCAMP->setDisabled(true);
		ui->intensity_JCAMP->setDisabled(true);
		ui->reflectance_JCAMP->setDisabled(true);
	}
	else
	{
		ui->absorbance_JCAMP->setDisabled(false);
		ui->intensity_JCAMP->setDisabled(false);
		ui->reflectance_JCAMP->setDisabled(false);
		ui->absorbance_JCAMP->setChecked(w->ifabsorbance_JCAMP());
		ui->intensity_JCAMP->setChecked(w->ifintensity_JCAMP());
		ui->reflectance_JCAMP->setChecked(w->ifreflectance_JCAMP());
	}
}

void Filesettings_window::on_absorbance_JCAMP_clicked(bool checked)
{
	w->absorbance_JCAMP=checked;
}

void Filesettings_window::on_intensity_JCAMP_clicked(bool checked)
{
	w->intensity_JCAMP= checked;
}

void Filesettings_window::on_reflectance_JCAMP_clicked(bool checked)
{
	w->reflectance_JCAMP=checked;
}

void Filesettings_window::on_Filesettings_window_finished(void)
{
	on_buttonBox_rejected();
}



void Filesettings_window::on_savebinary_clicked(bool checked)
{
    w->savebinary=checked;
}
