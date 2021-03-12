/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**  ALL RIGHTS RESERVED
**
******************************************************************************/

#include "scanconfigdialog.h"
#include "ui_scanconfigdialog.h"
#include "mainwindow.h"
#include <QMessageBox>
#include "dlpspec_scan_had.h"
#include "dlpspec_scan_col.h"
#include "scanconfigdialog.h"
#include <stdlib.h>
#include "API.h"
#include "spectrum.h"
#include "dlpspec_calib.h"
#include "dlpspec_util.h"
#include <QMessageBox>
#include "usb.h"
#include <QComboBox>


extern EVM evm;

ScanConfigDialog::ScanConfigDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ScanConfigDialog)
{
    ui->setupUi(this);
    clear_editBoxes();
    enable_editBoxes(false);

    ui->listWidget_target_scan_cfg->clear();

}

ScanConfigDialog::~ScanConfigDialog()
{
    delete ui;
}

void ScanConfigDialog::Init(ScanConfigList* list)
/**
 * Initializes the Local and Target list entries
 * Gets the Calibration Coefficients from EVM
 * Initializes the entries in width combobox based on the Pixel width
 *
 */
{
    scancfglist = list;
    temp_target_list = scancfglist->GetTargetList();
    temp_local_list = scancfglist->GetLocalList();
    m_activeConigIndex = scancfglist->Get_ActiveConfigIndex();
    if(NNO_GetCalibStruct(&m_calibCoeffs) == FAIL)
    {
        if(USB_IsConnected()) //CR18595 KV show the error message only when board is connected
        {
            QString title("Error Message");
            QString text("The Unit is not Calibrated. Please Calibrate and then proceed");
            QMessageBox msgBox(QMessageBox::Information, title, text, QMessageBox::NoButton, this);
            msgBox.exec();
        }
        else
        {
            target_connected = false;
        }
    }
    else
    {
        target_connected = true;
    }

    PopulateListWidgets();

    //setting up the scan Section Table


    ui->SlewItemsWidget->horizontalHeader()->stretchLastSection();

 #if 0
   // ui->SlewItemsWidget->resizeColumnsToContents();
    QComboBox *cb_type= new QComboBox();
    cb_type->addItem("Column");
    cb_type->addItem("Hadamard");
    ui->SlewItemsWidget->setCellWidget(0,0,cb_type);


    QLineEdit *le_startnm = new QLineEdit();

    ui->SlewItemsWidget->setCellWidget(0,1,le_startnm);


    QLineEdit *le_endnm = new QLineEdit();
    ui->SlewItemsWidget->setCellWidget(0,2,le_endnm);

    QComboBox *cb_width = new QComboBox();
    for(int j = 2; nmItem < NUM_WIDTH_ITEMS; j++)
    {
        nmItem = PIXEL_WIDTH * j;

        QString item;
        item.sprintf("%0.02f",nmItem);
        cb_width->addItem(item);
    }

    ui->SlewItemsWidget->setCellWidget(0,3,cb_width);

    QSpinBox *sp_numpoints = new QSpinBox();
    sp_numpoints->setMinimum(2);
    sp_numpoints->setMaximum(MAX_PATTERNS_PER_SCAN);

    ui->SlewItemsWidget->setCellWidget(0,4,sp_numpoints);


    QComboBox *cb_exp = new QComboBox();
    cb_exp->addItem("0.635");
    cb_exp->addItem("1.270");
    cb_exp->addItem("2.540");
    cb_exp->addItem("5.080");
    cb_exp->addItem("15.240");
    cb_exp->addItem("30.480");
    cb_exp->addItem("60.960");
    ui->SlewItemsWidget->setCellWidget(0,5,cb_exp);

#endif
    QHeaderView *headerView = ui->SlewItemsWidget->horizontalHeader();
    headerView->setSectionResizeMode(1,QHeaderView::Stretch);
    ui->SlewItemsWidget->horizontalHeader()->stretchLastSection();
    //ui->SlewItemsWidget->resizeColumnsToContents();
    ui->label_wavelength_error->setText("");
    ui->label_totalpatterns->setText("");


    if((m_activeConigIndex >= 0) && (m_activeConigIndex < temp_target_list.count()))
    {
            QListWidgetItem* lwi = ui->listWidget_target_scan_cfg->item(m_activeConigIndex);

            QFont font = lwi->font();
            font.setBold(true);
            lwi->setFont(font);
     }
    enable_editBoxes(false);

}

void ScanConfigDialog::PopulateListWidgets(void)
/**
  * Populates the Loal and Target Scan Config list boxes in the UI
  */
{
    ui->listWidget_target_scan_cfg->clear();
    ui->listWidget_local_scan_cfg->clear();

    for(int i=0; i<temp_target_list.count(); i++)
    {

        ui->listWidget_target_scan_cfg->addItem(temp_target_list.at(i).scanCfg.config_name);
        QListWidgetItem* lwi = ui->listWidget_target_scan_cfg->item(i);
        QFont font = lwi->font();

        if(i == m_activeConigIndex)
            font.setBold(true);
        else
            font.setBold(false);

        lwi->setFont(font);
    }







    for(int i=0; i<temp_local_list.count(); i++)
    {
        ui->listWidget_local_scan_cfg->addItem(temp_local_list.at(i).scanCfg.config_name);
    }

    if(temp_local_list.count() > 0)

    {
        QModelIndex modelIndex = ui->listWidget_local_scan_cfg->rootIndex(); // u have to find the model index of the first item here
        ui->listWidget_local_scan_cfg->setCurrentIndex(modelIndex);
        ui->listWidget_local_scan_cfg->item(0)->setSelected(true);
    }


}

int ScanConfigDialog::GetWidthIndex(double width)
/**
 * Gets the position/index of the current selected width in the
 * width combo box
 *
 */
{

    float nmItem = PIXEL_WIDTH; //min pixel width

    for(int j = MIN_PIXEL_INDEX; nmItem < NUM_WIDTH_ITEMS ; j++)
    {
        if(rint(width*100) <= rint((PIXEL_WIDTH * j)*100) )
        {
            return j - MIN_PIXEL_INDEX;
        }
    }

    return FAIL;
}

int ScanConfigDialog::getMaxPatterns(int startnm, int endnm, double width, int scan_type)
/**
 * Calculates the Maximum number of patterns possible
 * based on the start & end wavelengths ,calibration coefficients, scan type and width
 * calculation done seperately for column and hadamard types
 * @param startnm - I - the start wavelength for that section
 * @param endnm - I - the end wavelength for that section
 * @param width - I - the width for that section
 * @param scan_type - I - Column(0) or Hadamard(1) type
 * @return the Maximum allowed patterns for the section with given inputs
 */
{

    double startCol, endCol;
    int maxPatterns = 0;
    int maxShift = 0; 			// maximum shift across all rows of the DMD
    int blockShift = 0;			// maximum shift in a 4-row block
    int overlap = 0, maxOverlap = 0;			// amount of overlay between consecutive patterns
	int8_t* shiftVector = NULL;
    DLPSPEC_ERR_CODE ret_val = DLPSPEC_PASS;

    if((startnm < MIN_WAVELENGTH) || (endnm > MAX_WAVELENGTH) || (startnm > endnm))
        return 0;

    dlpspec_util_nmToColumn(startnm, m_calibCoeffs.PixelToWavelengthCoeffs, &endCol);
    dlpspec_util_nmToColumn(endnm, m_calibCoeffs.PixelToWavelengthCoeffs, &startCol);

    // Find max shift amount in a 4 row section. The CCP block uses 8x4 blocks 
    shiftVector = (int8_t*)(malloc(sizeof(uint8_t) * DMD_HEIGHT));
    dlpspec_calib_genShiftVector(m_calibCoeffs.ShiftVectorCoeffs, 
			DMD_HEIGHT, shiftVector);
			
    for (int k = 0; k < DMD_HEIGHT; k+= 4)
    {
        for (int line = 1; line < 4; line++ )
    	{
            if ( abs(shiftVector[line+k-1] - shiftVector[line+k]) > blockShift )
                blockShift = abs(shiftVector[line+k-1] - shiftVector[line+k]);
    	}
    	if ( blockShift > maxShift )
    		maxShift = blockShift;
    	blockShift = 0;
    }
    if(shiftVector != NULL)
    	free(shiftVector);
    	


    if(scan_type == COLUMN_TYPE)
    {
        patDefCol patDefC;
        bool pack8 = false;
        scanConfig new_config;
        strcpy(new_config.config_name, ui->lineEdit_scan_config_name->text().toStdString().c_str());
        new_config.wavelength_start_nm = startnm;
        new_config.wavelength_end_nm = endnm;
        //the GUI shows pixel width starting from 2 pixels , with 2 pixel width at index 0
        new_config.width_px = GetWidthIndex(width) + MIN_PIXEL_INDEX;
        new_config.num_repeats = ui->spinBox_num_scans_avg->value();
        new_config.scan_type =  COLUMN_TYPE;


        maxOverlap = 0;
        for(int i=8; i<=(MAX_PATTERNS_PER_SCAN); i++)
        {
            new_config.num_patterns = i;
            dlpspec_scan_col_genPatDef(&new_config,&m_calibCoeffs,&patDefC);
             for(int j=0; j<(patDefC.numPatterns - 8); j++)		// test pattern set with 8 members
            {
                overlap = patDefC.colWidth - abs(patDefC.colMidPix[j + 1] - patDefC.colMidPix[j]);
                if ( overlap > maxOverlap )
                       maxOverlap = overlap;
                if ( abs(patDefC.colMidPix[j + 7] - patDefC.colMidPix[j]) > (7 + maxShift + maxOverlap + 1) )
                    pack8 = false;
                else
                {
                    pack8 = true;       // More than 8 colors between pattern midpoints
                    break;
                }
            }
            if( (i <= abs(patDefC.colMidPix[patDefC.numPatterns - 1] - patDefC.colMidPix[0])) && !pack8)
            {
                if((pack8 && (patDefC.colWidth <= 16)) || !pack8)
                    maxPatterns = i;
                else
                    break;
            }
            else
                break;
        }
    }
    else if(scan_type == HADAMARD_TYPE)
    {
        patDefHad patDefH;

        scanConfig new_config;
        strcpy(new_config.config_name, ui->lineEdit_scan_config_name->text().toStdString().c_str());
        new_config.wavelength_start_nm = startnm;
        new_config.wavelength_end_nm = endnm;
        //the GUI shows pixel width starting from 2 pixels , with 2 pixel width at index 0
        new_config.width_px = GetWidthIndex(width) + MIN_PIXEL_INDEX;
        new_config.num_repeats = ui->spinBox_num_scans_avg->value();
        new_config.scan_type =  HADAMARD_TYPE;


        for(int i=1; i<MAX_PATTERNS_PER_SCAN; i++)
        {
            new_config.num_patterns = i;
            ret_val = dlpspec_scan_had_genPatDef(&new_config,&m_calibCoeffs,&patDefH);
            // valid pattern generation && unique non-repeated pattern && patterns do not exceed memory && patterns do not exceed ADC buffer
            if((ret_val == DLPSPEC_PASS) && (i <= (endCol - startCol)) && (patDefH.numPatterns < MAX_PATTERNS_PER_SCAN) && (patDefH.numPatterns < (ADC_DATA_LEN - MAX_PATTERNS_PER_SCAN/24)))
                maxPatterns = new_config.num_patterns;
        }
    }
    return maxPatterns;
}

void ScanConfigDialog::on_buttonBox_accepted()
{
    int ret;
    char err_str[50];

    scancfglist->SetLocalList(temp_local_list);
    scancfglist->SetTargetList(temp_target_list);
    scancfglist->Set_ActiveConfigIndex(m_activeConigIndex);
    ret = scancfglist->ExportTargetList();
    if(ret != PASS)
    {
        if(ret != FAIL)
        {
            sprintf(err_str, "Error saving config #%d to EVM", 0-ret);
            showError(err_str);
        }
        else
            showError("Error saving scan configs to EVM");
    }

    clear_editBoxes();
    enable_editBoxes(false);

    temp_target_list.clear();
    ui->listWidget_target_scan_cfg->clear();
}

void ScanConfigDialog::on_buttonBox_rejected()
{
    clear_editBoxes();
    enable_editBoxes(false);

    temp_target_list.clear();
    ui->listWidget_target_scan_cfg->clear();
}

void ScanConfigDialog::clear_editBoxes()
{
    ui->spinBox_num_scans_avg->setValue(1);
    ui->spinBox_num_scans_avg->clear();
    ui->lineEdit_scan_config_name->clear();
    ui->spinBox_numSections->setValue(1);
    ui->spinBox_numSections->clear();
    ui->SlewItemsWidget->setRowCount(0);
}

void ScanConfigDialog::enable_editBoxes(bool enable)
/**
  * Enables or Disables the edit boxes and the Sections widget
  * @param enable - boolean if true enables widgets
  *                            false disables widgets
  */
{

    ui->spinBox_num_scans_avg->setEnabled(enable);
    ui->lineEdit_scan_config_name->setEnabled(enable);
    ui->SlewItemsWidget->setEnabled(enable);

    ui->SlewItemsWidget->setVisible(true);

    ui->label_NumSections->setEnabled(enable);
    ui->spinBox_numSections->setEnabled(enable);
    ui->pushButton_scan_config_save->setEnabled(enable);
}

void ScanConfigDialog::on_listWidget_local_scan_cfg_currentRowChanged(int currentRow)
{
    if( (currentRow >= 0) && currentRow < temp_local_list.count())
        DisplayScanConfigItem(temp_local_list.at(currentRow));

    if(ui->pushButton_edit_local->isChecked() || ui->pushButton_edit_target->isChecked())
    {
        enable_editBoxes(false);
        ui->pushButton_edit_local->setChecked(false);
    }
}

void ScanConfigDialog::on_pushButton_edit_local_clicked()
{
    if(ui->listWidget_local_scan_cfg->currentRow() < 0)
    {
        showError("No items selected");
        return;

    }
    enable_editBoxes(true);

    //emit on_listWidget_local_scan_cfg_currentRowChanged(ui->listWidget_local_scan_cfg->currentRow());

    int currentRow = ui->listWidget_local_scan_cfg->currentRow();
    if( (currentRow >= 0) && currentRow < temp_local_list.count())
        DisplayScanConfigItem(temp_local_list.at(currentRow));

    save_to_target = false;
    ui->pushButton_scan_config_save->setText("Save to Local");
    edit_existing = true;
    edit_index = ui->listWidget_local_scan_cfg->currentRow();
}


void ScanConfigDialog::on_pushButton_edit_target_clicked()
{
    if(ui->listWidget_target_scan_cfg->currentRow() < 0)
    {
        showError("No items selected");
        return;
    }

    enable_editBoxes(true);

    //emit on_listWidget_target_scan_cfg_currentRowChanged(ui->listWidget_target_scan_cfg->currentRow());

    int currentRow = ui->listWidget_target_scan_cfg->currentRow();
    if( (currentRow >= 0) && currentRow < temp_target_list.count())
        DisplayScanConfigItem(temp_target_list.at(currentRow));

    save_to_target = true;
    ui->pushButton_scan_config_save->setText("Save to NIRscan Nano");
    edit_existing = true;
    edit_index = ui->listWidget_target_scan_cfg->currentRow();
}

void ScanConfigDialog::on_pushButton_new_local_clicked()
{
    clear_editBoxes();
    enable_editBoxes(true);

    ui->pushButton_scan_config_save->setText("Save to Local");
    save_to_target = false;
    edit_existing = false;
}

void ScanConfigDialog::on_pushButton_new_target_clicked()
{

    clear_editBoxes();
    enable_editBoxes(true);

    ui->pushButton_scan_config_save->setText("Save to NIRscan Nano");
    save_to_target = true;
    edit_existing = false;
}

void ScanConfigDialog::on_pushButton_scan_config_save_clicked()
/**
  * Saves the current entries in the edit boxes and the Slew sections
  * to the repsective local or target scan configuration
  */
{
    char user_message[100];
    uScanConfig new_u_config;    
    scanConfig temp_cfg;
    int total_num_patterns = 0;
    bool is_duplicate = false;
    patDefHad patDef;

    if(!QString::compare(ui->lineEdit_scan_config_name->text().toStdString().c_str(),""))
    {
        showError("EmptyScan Config Name");
        return;
    }

    int maxPatterns = 100;
  //  if(ui->spinBox_numSections->value() > 1)
    {
        strcpy(new_u_config.slewScanCfg.head.config_name, ui->lineEdit_scan_config_name->text().toStdString().c_str());
        new_u_config.slewScanCfg.head.num_repeats = ui->spinBox_num_scans_avg->value();
        new_u_config.slewScanCfg.head.scan_type = SLEW_TYPE;
        new_u_config.slewScanCfg.head.num_sections = ui->spinBox_numSections->value();
        for(int i = 0; i < ui->spinBox_numSections->value() ; i++)
        {
            QComboBox* cb_type =  (QComboBox*)ui->SlewItemsWidget->cellWidget(i,0);
            if(cb_type)
            {
                new_u_config.slewScanCfg.section[i].section_scan_type =  cb_type->currentIndex();
            }
            QLineEdit* le_startnm = (QLineEdit*)ui->SlewItemsWidget->cellWidget(i,1);
            if(le_startnm)
            {
                new_u_config.slewScanCfg.section[i].wavelength_start_nm = le_startnm->text().toFloat();
            }
            QLineEdit* le_endnm = (QLineEdit*)ui->SlewItemsWidget->cellWidget(i,2);
            if(le_endnm)
            {

                new_u_config.slewScanCfg.section[i].wavelength_end_nm = le_endnm->text().toFloat();
            }

            QComboBox* cb_width =  (QComboBox*)ui->SlewItemsWidget->cellWidget(i,3);
            if(cb_width)
            {
                new_u_config.slewScanCfg.section[i].width_px = GetWidthIndex(cb_width->currentText().toFloat()) + MIN_PIXEL_INDEX;
            }

            if((le_startnm->text().toFloat() < MIN_WAVELENGTH) || (le_endnm->text().toFloat() > MAX_WAVELENGTH) || (le_endnm->text().toFloat() < le_startnm->text().toFloat()))
            {
                QString text;
                QString title("Error Message");
                text.sprintf("Please enter values in the range of %d to %d for section %d", MIN_WAVELENGTH , MAX_WAVELENGTH, i+1);
                QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton, this);
                msgBox.exec();
                return;
            }

            QSpinBox* sp_numPats = (QSpinBox*)ui->SlewItemsWidget->cellWidget(i,4);
            if(sp_numPats)
            {
                maxPatterns = getMaxPatterns(le_startnm->text().toFloat(),le_endnm->text().toFloat(),cb_width->currentText().toFloat(),cb_type->currentIndex());
                if(maxPatterns < 2)
                {
                    QString text;
                    QString title("Error Message");
                    text.sprintf("The Start,End Wavelength and Width of section %d\n is not a valid combination for a minimum of 2 patterns", i+1);
                    QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton, this);
                    msgBox.exec();
                    return;
                }
                if(sp_numPats->value() > maxPatterns)
                {
                    QString text;
                    QString title("Error Message");
                    text.sprintf("The maximum Patterns possible for the section %d is %d", i+1,maxPatterns);
                    QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton, this);
                    msgBox.exec();
                    return;

                }
                else
                {
                    new_u_config.slewScanCfg.section[i].num_patterns = sp_numPats->value();
                    if ((new_u_config.slewScanCfg.section[i].section_scan_type == HADAMARD_TYPE) &&
                            (target_connected == true))
                    {
                        temp_cfg.wavelength_start_nm = new_u_config.slewScanCfg.section[i].wavelength_start_nm;
                        temp_cfg.wavelength_end_nm = new_u_config.slewScanCfg.section[i].wavelength_end_nm;
                        temp_cfg.width_px = new_u_config.slewScanCfg.section[i].width_px;
                        temp_cfg.num_patterns = new_u_config.slewScanCfg.section[i].num_patterns;
                        temp_cfg.num_repeats = new_u_config.slewScanCfg.head.num_repeats;
                        temp_cfg.scan_type = new_u_config.slewScanCfg.section[i].section_scan_type;

                        dlpspec_scan_had_genPatDef(&temp_cfg, &m_calibCoeffs, &patDef);
                        total_num_patterns += patDef.numPatterns;
                    }
                    else
                        total_num_patterns = total_num_patterns + sp_numPats->value();
                }
            }

            QComboBox* cb_exp =  (QComboBox*)ui->SlewItemsWidget->cellWidget(i,5);

            if(cb_exp)
            {
                new_u_config.slewScanCfg.section[i].exposure_time =  cb_exp->currentIndex();
            }

            if(total_num_patterns > MAX_PATTERNS_PER_SCAN) //max patterns allowed
            {
                QString text;
                QString title("Error Message");
                text.sprintf("Total number of patterns %d exceeds 624", total_num_patterns);
                QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton, this);
                msgBox.exec();
                return;
            }
        }
        if(save_to_target)
        {
            is_duplicate = IsDuplicateName(new_u_config.slewScanCfg.head.config_name,temp_target_list, edit_index);

            if(!is_duplicate)
            {
                if(edit_existing)
                {
                    temp_target_list.replace(edit_index, new_u_config);
                    ui->listWidget_target_scan_cfg->currentItem()->setText(new_u_config.slewScanCfg.head.config_name);
                }
                else
                {
                    if(ui->listWidget_target_scan_cfg->count() >= EEPROM_MAX_SCAN_CFG_STORAGE)
                    {
                        sprintf(user_message, "Number of scan configs in EVM cannot exceed %d", EEPROM_MAX_SCAN_CFG_STORAGE);
                        showError(user_message);
                        return;
                    }
                    temp_target_list.append(new_u_config);
                    ui->listWidget_target_scan_cfg->addItem(new_u_config.slewScanCfg.head.config_name);
                }
            }
            else
            {
                showError("Duplicate Scan Config Name");
                return;
            }

        }
        else
        {
            is_duplicate = IsDuplicateName(new_u_config.slewScanCfg.head.config_name,temp_local_list, edit_index);

            if(!is_duplicate)
            {
                if(edit_existing)
                {
                    temp_local_list.replace(edit_index, new_u_config);
                    ui->listWidget_local_scan_cfg->currentItem()->setText(new_u_config.slewScanCfg.head.config_name);
                }
                else
                {
                    temp_local_list.append(new_u_config);
                    ui->listWidget_local_scan_cfg->addItem(new_u_config.slewScanCfg.head.config_name);
                }
            }
            else
            {
                showError("Duplicate Scan Config Name");
            }
        }
    }

   enable_editBoxes(false);//show the table
}

bool ScanConfigDialog::IsDuplicateName(QString name, QList<uScanConfig> list_scan_cfg, int edit_index)
/**
 * Checks if the entered name a;ready exists in the list and warns tha user of duplicate name
 * @param name - I - the current entered name to be checked for duplicates
 * @param list_scan_cfg - I - the list where the duplicate if any has to be searched for
 * @param edit_index - I - in case of editing an entry, skip this index since it is the current entry
 */
{
    bool is_duplicate = false;
    for(int i = 0; i < list_scan_cfg.size(); i++)
    {

        if(!QString::compare(list_scan_cfg[i].scanCfg.config_name , name) && (i != edit_index))
            is_duplicate = true;
    }
    return is_duplicate;
}

void ScanConfigDialog::DisplayScanConfigItem(uScanConfig uCfg)
/**
 *  Calls the function to Populate the SlewSectionsGrid
 *  Displays the current selected Scan Configuration
 *  fields in the repsective Table items
 *  @param uCfg - I - the current selected Scan Configuration
 */
{
    ui->SlewItemsWidget->setVisible(true);
    if(uCfg.scanCfg.scan_type == SLEW_TYPE)
    {
        PopulateScanSectionGrid(uCfg.slewScanCfg.head.num_sections);

        for(int i = 0; i < uCfg.slewScanCfg.head.num_sections; i++)
        {

            QComboBox* cb_type =  (QComboBox*)ui->SlewItemsWidget->cellWidget(i,0);
            if(cb_type)
                cb_type->setCurrentIndex(uCfg.slewScanCfg.section[i].section_scan_type);

            QLineEdit* le_startnm = (QLineEdit*)ui->SlewItemsWidget->cellWidget(i,1);
            if(le_startnm)
                le_startnm->setText(QString::number(uCfg.slewScanCfg.section[i].wavelength_start_nm));

            QLineEdit* le_endnm = (QLineEdit*)ui->SlewItemsWidget->cellWidget(i,2);
            if(le_endnm)
                le_endnm->setText(QString::number(uCfg.slewScanCfg.section[i].wavelength_end_nm));

            QComboBox* cb_width =  (QComboBox*)ui->SlewItemsWidget->cellWidget(i,3);
            if(cb_width)
            {
                int width_index = uCfg.slewScanCfg.section[i].width_px - MIN_PIXEL_INDEX;
                if(width_index >= 0)
                    cb_width->setCurrentIndex(width_index);
            }
            QSpinBox* sb = (QSpinBox*)ui->SlewItemsWidget->cellWidget(i,4);

            if(sb)
                sb->setValue(uCfg.slewScanCfg.section[i].num_patterns);

            QComboBox* cb_exp =  (QComboBox*)ui->SlewItemsWidget->cellWidget(i,5);
            if(cb_exp)
            {

                cb_exp->setCurrentIndex(uCfg.slewScanCfg.section[i].exposure_time);
            }


        }
        ui->lineEdit_scan_config_name->setText(uCfg.slewScanCfg.head.config_name);
        ui->spinBox_num_scans_avg->setValue(uCfg.slewScanCfg.head.num_repeats);
        ui->spinBox_numSections->setValue(uCfg.slewScanCfg.head.num_sections);
    }
    else
    {

        PopulateScanSectionGrid(1);
        ui->spinBox_numSections->setValue(1);
        QComboBox* cb_type =  (QComboBox*)ui->SlewItemsWidget->cellWidget(0,0);
        if(cb_type)
            cb_type->setCurrentIndex(uCfg.scanCfg.scan_type);

        QLineEdit* le_startnm = (QLineEdit*)ui->SlewItemsWidget->cellWidget(0,1);
        if(le_startnm)
            le_startnm->setText(QString::number(uCfg.scanCfg.wavelength_start_nm));

        QLineEdit* le_endnm = (QLineEdit*)ui->SlewItemsWidget->cellWidget(0,2);
        if(le_endnm)
            le_endnm->setText(QString::number(uCfg.scanCfg.wavelength_end_nm));

        QComboBox* cb_width =  (QComboBox*)ui->SlewItemsWidget->cellWidget(0,3);
        if(cb_width)
        {
            int width_index = uCfg.scanCfg.width_px - MIN_PIXEL_INDEX;
            if(width_index >= 0)
                cb_width->setCurrentIndex(width_index);
        }
        QSpinBox* sb = (QSpinBox*)ui->SlewItemsWidget->cellWidget(0,4);

        if(sb)
            sb->setValue(uCfg.scanCfg.num_patterns);

        QComboBox* cb_exp =  (QComboBox*)ui->SlewItemsWidget->cellWidget(0,5);
        if(cb_exp)
        {
            cb_exp->setCurrentIndex(0);
           // cb_exp->setMaxCount(1);
        }

        ui->lineEdit_scan_config_name->setText(uCfg.scanCfg.config_name);
        ui->spinBox_num_scans_avg->setValue(uCfg.scanCfg.num_repeats);
        ui->lineEdit_scan_config_name->setText(uCfg.scanCfg.config_name);

    }
}

void ScanConfigDialog::on_listWidget_target_scan_cfg_currentRowChanged(int currentRow)
{
    if( (currentRow >= 0) && currentRow < temp_target_list.count())
    {
        DisplayScanConfigItem(temp_target_list.at(currentRow));
    }
    if(ui->pushButton_edit_target->isChecked() || ui->pushButton_edit_local->isChecked())
    {
        enable_editBoxes(false);
        ui->pushButton_edit_target->setChecked(false);
    }
}


void ScanConfigDialog::on_pushButton_delete_local_clicked()
{
    if(ui->listWidget_local_scan_cfg->currentRow() < 0)
    {
        showError("No items selected");
        return;
    }

    temp_local_list.removeAt(ui->listWidget_local_scan_cfg->currentRow());
    ui->listWidget_local_scan_cfg->clear();
    for(int i=0; i<temp_local_list.count(); i++)
    {
        ui->listWidget_local_scan_cfg->addItem(temp_local_list.at(i).scanCfg.config_name);
    }

    clear_editBoxes();
}

void ScanConfigDialog::on_pushButton_delete_target_clicked()
{


    if(ui->listWidget_target_scan_cfg->currentRow() < 0)
    {
        showError("No items selected");
        return;
    }
    if(ui->listWidget_target_scan_cfg->currentRow() == 0)
    {
        showError("The factory default config cannot be deleted");
        return;
    }

    if(ui->listWidget_target_scan_cfg->currentRow() == m_activeConigIndex)
    {
        m_activeConigIndex = 0;
        showError("The active config is being deleted.\nSetting the first config as active");
    }
    //reduce the index if a config before is deleted
    else if(m_activeConigIndex > ui->listWidget_target_scan_cfg->currentRow())
        m_activeConigIndex--;

    temp_target_list.removeAt(ui->listWidget_target_scan_cfg->currentRow());
    ui->listWidget_target_scan_cfg->clear();
    for(int i=0; i<temp_target_list.count(); i++)
    {
        ui->listWidget_target_scan_cfg->addItem(temp_target_list.at(i).scanCfg.config_name);
        QListWidgetItem* lwi = ui->listWidget_target_scan_cfg->item(i);
        QFont font = lwi->font();
        if(i == m_activeConigIndex)
        {
            font.setBold(true);
        }
        else
            font.setBold(false);

        lwi->setFont(font);


    }




    clear_editBoxes();
}

void ScanConfigDialog::showError(const char *str)
{
    QString title("Error Message");
    QString text(str);
    QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton, this);
    msgBox.exec();
}

void ScanConfigDialog::on_pushButton_copy_right_clicked()
{

    int row = ui->listWidget_local_scan_cfg->currentRow();

    if(row < 0)
    {
        showError("No items selected");
        return;
    }

    if(!IsDuplicateName(temp_local_list.at(row).slewScanCfg.head.config_name, temp_target_list,-1))
    {
        if(temp_target_list.count() < EEPROM_MAX_SCAN_CFG_STORAGE)
            temp_target_list.append(temp_local_list.at(row));
        else
        {
            char user_message[100];
            sprintf(user_message, "Number of scan configs in EVM cannot exceed %d", EEPROM_MAX_SCAN_CFG_STORAGE);
            showError(user_message);
            return;
        }

    }
    else
        showError("Duplicate Scan Config Name");

    PopulateListWidgets();
}




void ScanConfigDialog::on_pushButton_move_right_clicked()
{

    int row = ui->listWidget_local_scan_cfg->currentRow();

    if(row < 0)
    {
        showError("No items selected");
        return;
    }
    if(!IsDuplicateName(temp_local_list.at(row).slewScanCfg.head.config_name, temp_target_list,-1))
    {
        if(temp_target_list.size() < EEPROM_MAX_SCAN_CFG_STORAGE)
            temp_target_list.append(temp_local_list.takeAt(ui->listWidget_local_scan_cfg->currentRow()));
        else
        {
            char user_message[100];
            sprintf(user_message, "Number of scan configs in EVM cannot exceed %d", EEPROM_MAX_SCAN_CFG_STORAGE);
            showError(user_message);
            return;
        }
    }
    else
        showError("Duplicate Scan Config Name");

    PopulateListWidgets();
}
void ScanConfigDialog::on_pushButton_copy_left_clicked()
{

    int row = ui->listWidget_target_scan_cfg->currentRow();
    if(row < 0)
    {
        showError("No items selected");
        return;
    }


    if(!IsDuplicateName(temp_target_list.at(row).slewScanCfg.head.config_name, temp_local_list,-1))
        temp_local_list.append(temp_target_list.at(row));
    else
        showError("Duplicate Scan Config Name");

    PopulateListWidgets();
}

void ScanConfigDialog::on_pushButton_move_left_clicked()
{
    int row = ui->listWidget_target_scan_cfg->currentRow();

    if(row < 0)
    {
        showError("No items selected");
        return;
    }

    if(!IsDuplicateName(temp_target_list.at(row).slewScanCfg.head.config_name, temp_local_list,-1))
        temp_local_list.append(temp_target_list.takeAt(ui->listWidget_target_scan_cfg->currentRow()));
    else
        showError("Duplicate Scan Config Name");

    PopulateListWidgets();
}


void ScanConfigDialog::PopulateScanSectionGrid(int numsections)
/**
  * Populates the Sections Table entries based on the number of sections selected
  * @param numsections - I - number of sections to be populated in the table
  */
{

    int currentRows = ui->SlewItemsWidget->rowCount();

    if(currentRows < 0)
        currentRows = 0;

   ui->SlewItemsWidget->setRowCount(numsections);

    for(int i = currentRows; i < numsections ; i++)
    {
        ui->label_totalpatterns->setText("");
        ui->label_wavelength_error->setText("");

        QComboBox *cb_type= new QComboBox();
        cb_type->addItem("Column");
        cb_type->addItem("Hadamard");
        cb_type->installEventFilter(this);
        ui->SlewItemsWidget->setCellWidget(i,0,cb_type);

        QLineEdit *le_startnm = new QLineEdit();
        le_startnm->installEventFilter(this);
        ui->SlewItemsWidget->setCellWidget(i,1,le_startnm);

        QLineEdit *le_endnm = new QLineEdit();
        le_endnm->installEventFilter(this);
        ui->SlewItemsWidget->setCellWidget(i,2,le_endnm);
        float nmItem = PIXEL_WIDTH; //min pixel width

        QComboBox *cb_width = new QComboBox();
        for(int j = 2; nmItem < NUM_WIDTH_ITEMS ; j++)
        {
            nmItem = PIXEL_WIDTH * j;

            QString item;
            item.sprintf("%0.02f",nmItem);
            cb_width->addItem(item);
        }
        cb_width->setCurrentIndex(DEFAULT_WIDTH_INDEX);
        cb_width->installEventFilter(this);

        ui->SlewItemsWidget->setCellWidget(i,3,cb_width);

        QSpinBox *sp_numpoints = new QSpinBox();
        sp_numpoints->setMinimum(2);
        sp_numpoints->setMaximum(MAX_PATTERNS_PER_SCAN);
        sp_numpoints->installEventFilter(this);
        ui->SlewItemsWidget->setCellWidget(i,4,sp_numpoints);

        QComboBox *cb_exp = new QComboBox();
        cb_exp->addItem("0.635");
       // if(numsections > 1)
        {
            cb_exp->addItem("1.270");
            cb_exp->addItem("2.540");
            cb_exp->addItem("5.080");
            cb_exp->addItem("15.240");
            cb_exp->addItem("30.480");
            cb_exp->addItem("60.960");
        }

        cb_exp->installEventFilter(this);

        ui->SlewItemsWidget->setCellWidget(i,5,cb_exp);

        QHeaderView *headerView = ui->SlewItemsWidget->horizontalHeader();
        headerView->setSectionResizeMode(1,QHeaderView::Stretch);

    }
    ui->SlewItemsWidget->horizontalHeader()->stretchLastSection();
   // ui->SlewItemsWidget->resizeColumnsToContents();
 }



void ScanConfigDialog::on_listWidget_target_scan_cfg_clicked(const QModelIndex & )
{

    int currentRow = ui->listWidget_target_scan_cfg->currentRow();

    if( (currentRow >= 0) && currentRow < temp_target_list.count())
        DisplayScanConfigItem(temp_target_list.at(currentRow));

    if(ui->pushButton_edit_target->isChecked() || ui->pushButton_edit_local->isChecked())
    {
        enable_editBoxes(false);
        ui->pushButton_edit_target->setChecked(false);
    }

}

void ScanConfigDialog::on_listWidget_local_scan_cfg_clicked(const QModelIndex & )
{
    int currentRow = ui->listWidget_local_scan_cfg->currentRow();

    if( (currentRow >= 0) && currentRow < temp_local_list.count())
        DisplayScanConfigItem(temp_local_list.at(currentRow));

    if(ui->pushButton_edit_local->isChecked() || ui->pushButton_edit_target->isChecked())
    {
        enable_editBoxes(false);
        ui->pushButton_edit_local->setChecked(false);
    }

}

void ScanConfigDialog::on_pushButton_set_active_clicked()
{
     int currentRow = ui->listWidget_target_scan_cfg->currentRow();
     if(currentRow > -1)
         m_activeConigIndex = currentRow;


      if((m_activeConigIndex >= 0) && (m_activeConigIndex < temp_target_list.count()))
      {
          for(int i =0 ; i < temp_target_list.count(); i++)
          {
              QListWidgetItem* lwi = ui->listWidget_target_scan_cfg->item(i);
              QFont font = lwi->font();

              if(i == m_activeConigIndex)
                  font.setBold(true);
              else
                  font.setBold(false);

              lwi->setFont(font);

          }
      }


}


bool ScanConfigDialog::eventFilter(QObject *target, QEvent *event)
{
    int currentRow = -1;
    int currentColumn = -1;
    bool found = false;

    if((event->type() == QEvent::FocusOut) || event->type() == QEvent::FocusIn )
    {
        for(int i = 0 ; i < ui->SlewItemsWidget->rowCount(); i++)
        {
            for(int j = 0; j < 6; j++)
            {
                if(ui->SlewItemsWidget->cellWidget(i,j) == target)
                {
                  currentRow = i;
                  currentColumn = j;
                  found = true;
                  break;
                }
                if(found)
                    break;
            }

         }

    }



    if((currentColumn > -1) && (currentColumn < 5) && (currentRow > -1))
    {

        int section_scan_type = 0;
        float wavelength_start_nm = 0;
        float wavelength_end_nm = 0;
        float width = 0;
        QComboBox* cb_type =  (QComboBox*)ui->SlewItemsWidget->cellWidget(currentRow,0);
        if(cb_type)
        {
            section_scan_type =  cb_type->currentIndex();
        }
        QLineEdit* le_startnm = (QLineEdit*)ui->SlewItemsWidget->cellWidget(currentRow,1);
        if(le_startnm)
        {
            if(le_startnm->text() == "")
                return false;
            wavelength_start_nm = le_startnm->text().toFloat();
        }
        QLineEdit* le_endnm = (QLineEdit*)ui->SlewItemsWidget->cellWidget(currentRow,2);
        if(le_endnm)
        {
            if(le_endnm->text() == "")
               return false;
            wavelength_end_nm = le_endnm->text().toFloat();
        }

        QComboBox* cb_width =  (QComboBox*)ui->SlewItemsWidget->cellWidget(currentRow,3);
        if(cb_width)
        {
            width = cb_width->currentText().toFloat();
        }

        QString text;
        int total_patterns = 0;
        int num_patterns = 0;
        int maxPatterns = 0;
        if(event->type() == QEvent::FocusIn)
        {
            maxPatterns = getMaxPatterns(wavelength_start_nm,wavelength_end_nm,width,section_scan_type);
            QSpinBox* sb_patterns = (QSpinBox*)ui->SlewItemsWidget->cellWidget(currentRow,4);
            if(sb_patterns)
            {
                sb_patterns->setMaximum(maxPatterns);
            }

            if(section_scan_type == HADAMARD_TYPE)
                sb_patterns->setMinimum(3);
            else
                sb_patterns->setMinimum(2);
        }

        scanConfig temp_cfg;
        patDefHad patDef;
        QString wavelength_text = "";
        int width_px = MIN_PIXEL_INDEX;
        for(int i = 0; i < ui->spinBox_numSections->value(); i++)
        {


            QComboBox* cb_type =  (QComboBox*)ui->SlewItemsWidget->cellWidget(i,0);
            if(cb_type)
            {
                section_scan_type =  cb_type->currentIndex();
            }
            QLineEdit* le_startnm = (QLineEdit*)ui->SlewItemsWidget->cellWidget(i,1);
            if(le_startnm)
            {
                if(le_startnm->text() == "")
                    continue;
                wavelength_start_nm = le_startnm->text().toFloat();
            }
            QLineEdit* le_endnm = (QLineEdit*)ui->SlewItemsWidget->cellWidget(i,2);
            if(le_endnm)
            {
                if(le_endnm->text() == "")
                    continue;
                wavelength_end_nm = le_endnm->text().toFloat();
            }


            QComboBox* cb_width =  (QComboBox*)ui->SlewItemsWidget->cellWidget(i,3);
            if(cb_width)
            {
                width_px = GetWidthIndex(cb_width->currentText().toFloat()) + MIN_PIXEL_INDEX;
            }
            QSpinBox* sp_numPats = (QSpinBox*)ui->SlewItemsWidget->cellWidget(i,4);
            if(sp_numPats)
            {
                num_patterns = sp_numPats->value();
            }

            int num_repeats = ui->spinBox_num_scans_avg->value();
            temp_cfg.wavelength_start_nm = wavelength_start_nm;
            temp_cfg.wavelength_end_nm = wavelength_end_nm;
            temp_cfg.width_px = width_px;
            temp_cfg.num_patterns = num_patterns;
            temp_cfg.num_repeats = num_repeats;
            temp_cfg.scan_type = section_scan_type;
            if(section_scan_type == HADAMARD_TYPE)
            {
            dlpspec_scan_had_genPatDef(&temp_cfg, &m_calibCoeffs, &patDef);
            num_patterns = patDef.numPatterns;
            }

             total_patterns = maxPatterns;
             //total_patterns = total_patterns + num_patterns;

             if(event->type() == QEvent::FocusIn)
             {
                 if((wavelength_start_nm < MIN_WAVELENGTH) || (wavelength_end_nm > MAX_WAVELENGTH))
                 {
                     wavelength_text.sprintf("<font color='red'>Please enter values in the range of %d to %d for section %d</font>",MIN_WAVELENGTH , MAX_WAVELENGTH,i+1);
                    // ui->label_wavelength_error->setText(text);

                 }
                 else if( wavelength_end_nm < wavelength_start_nm)
                 {

                     wavelength_text.sprintf("<font color='red'>The start wavelength cannot be greater than the end wavelength for section %d</font>", i+1);
                     //ui->label_wavelength_error->setText(text);

                 }
             }
            }

        text.sprintf("<b>Max patterns used: %d/624</b>", total_patterns);
        ui->label_totalpatterns->setText(text);
        ui->label_wavelength_error->setText(wavelength_text);
        QApplication::processEvents();
    }

    return QDialog::eventFilter(target, event);
}

void ScanConfigDialog::on_spinBox_numSections_editingFinished()
{
    int numsections = ui->spinBox_numSections->value();
    PopulateScanSectionGrid(numsections);
}
