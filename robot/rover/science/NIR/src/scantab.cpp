/*****************************************************************************
 *
 * This module provides function handlers and supporting functions for different controls on scan tab.
 *
 * Copyright (c) 2015 Texas Instruments Incorporated.
 * ALL RIGHTS RESERVED
 *
 ******************************************************************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDirIterator>
#include <QTimer>
#include <QTime>
extern EVM evm;
extern FilePath filepath;
//extern bool cont_scan_flag;
//helper functions in the class

void MainWindow::mark_labels(PLOT_TYPE type)
/**
 * This function marks the labels for the scan plot
 *
 * @param type - I - type of scan as per enum PLOT_TYPE
 */
{
    ui->label_yaxis->setVisible(true);
	ui->label_xaxis->setVisible(true);

    if(type == PLOT_INTENSITY )
	{
        ui->label_yaxis->setText("Intensities(AU)");
        ui->label_yaxis->repaint();

	}
    else if(type == PLOT_ABSORBANCE)
	{
        ui->label_yaxis->setText("Absorbance");
        ui->label_yaxis->repaint();
	}
    else if(type == PLOT_REFLECTANCE)
    {
        ui->label_yaxis->setText("Reflectance%");
        ui->label_yaxis->repaint();
    }
    ui->graphicsView->setScene(scene);
    ui->graphicsView->setVisible(true);
	QApplication::processEvents();
}

#if 0 //only for debug

#define NUM_FRAMEBUFFERS 26
int g_frameSyncArr[NUM_FRAMEBUFFERS];

#define MAX_VSYNCS          2496   //NUM_FRAMEBUFFERS * 96 max possible exposure is 96 times 635 us
static int g_patternsPerVsyncarr[MAX_VSYNCS];

#define VSYNC_ACTIVE_PATTERN_PERIOD 15240

static int Scan_SetFrameSyncs(uScanConfig *pCfg)
{
    int numPattterns_inFrame = 0;
    int frameNumber_index = 0;
    int exp_time_us;
    int numPatterns_inSection;
    int i;
    int j;
    int numPatterns_inVsync = 0;
    int tot_exp_time_inVsync = 0;
    int vSyncarr_index = 0;
    int frame_sync_count = 1;

    for(i = 0; i < NUM_FRAMEBUFFERS ; i++)
    {
        g_frameSyncArr[i] = 1;
    }
    for(j=0; j < MAX_VSYNCS; j++)
    {
        g_patternsPerVsyncarr[j] = 25;
    }

    if(pCfg->scanCfg.scan_type == SLEW_TYPE)
    {
        for(i=0; i<pCfg->slewScanCfg.head.num_sections; i++)
        {
            exp_time_us = dlpspec_scan_get_exp_time_us((EXP_TIME)pCfg->slewScanCfg.section[i].exposure_time);
            numPatterns_inSection = pCfg->slewScanCfg.section[i].num_patterns;

            while(numPatterns_inSection > 0 )
            {
                //special case for exposure time > vsync period
                if(exp_time_us > VSYNC_ACTIVE_PATTERN_PERIOD)
                {
                    //close out the previous vysnc and start at next
                    if(numPatterns_inVsync != 0)
                    {
                        if(numPattterns_inFrame == 24)
                        {
                            //dark time for the 25th frame. There is always room for the black pattern exposure
                            numPatterns_inVsync++;
                            g_frameSyncArr[frameNumber_index++] = frame_sync_count ;
                            frame_sync_count = 1;
                            numPattterns_inFrame = 0;
                        }
                        g_patternsPerVsyncarr[vSyncarr_index++] = numPatterns_inVsync;
                    }
                    numPattterns_inFrame++;
                    if(exp_time_us > 2*VSYNC_ACTIVE_PATTERN_PERIOD) //4x
                    {
                        g_patternsPerVsyncarr[vSyncarr_index++] = 0;
                        g_patternsPerVsyncarr[vSyncarr_index++] = 0;
                        g_patternsPerVsyncarr[vSyncarr_index++] = 0;
                        if(numPattterns_inFrame == 1)
                            frame_sync_count+=3;
                        else
                            frame_sync_count+=4;
                    }
                    else                                            //2x
                    {
                        g_patternsPerVsyncarr[vSyncarr_index++] = 0;
                        if(numPattterns_inFrame == 1)
                            frame_sync_count++;
                        else
                            frame_sync_count+=2;
                    }
                    numPatterns_inVsync = 1;    //carry over this pattern to next vsync
                    tot_exp_time_inVsync = exp_time_us;
                }
                else if((tot_exp_time_inVsync + exp_time_us) > VSYNC_ACTIVE_PATTERN_PERIOD )
                {
                    if(numPattterns_inFrame == 24)
                    {
                        //dark time for the 25th frame. There is always room for the black pattern exposure
                        numPatterns_inVsync++;
                        g_frameSyncArr[frameNumber_index++] = frame_sync_count ;
                        frame_sync_count = 1;
                        numPattterns_inFrame = 1;
                    }
                    else
                    {
                        frame_sync_count++;
                        numPattterns_inFrame++;
                    }
                    g_patternsPerVsyncarr[vSyncarr_index++] = numPatterns_inVsync;
                    numPatterns_inVsync = 1;    //carry over this pattern to next vsync
                    tot_exp_time_inVsync = exp_time_us;
                }
                else
                {
                    if(numPattterns_inFrame == 24)
                    {
                        //dark time for the 25th frame. There is always room for the black pattern exposure
                        g_patternsPerVsyncarr[vSyncarr_index++] = numPatterns_inVsync + 1;
                        g_frameSyncArr[frameNumber_index++] = frame_sync_count ;
                        frame_sync_count = 1;
                        numPattterns_inFrame = 1;
                        numPatterns_inVsync = 1;
                        tot_exp_time_inVsync = exp_time_us;
                    }
                    else
                    {
                        numPattterns_inFrame++;
                        numPatterns_inVsync++;
                        tot_exp_time_inVsync += exp_time_us;
                    }
                }
                numPatterns_inSection--;
            }
        }
        if(numPatterns_inVsync != 0)
        {
            g_patternsPerVsyncarr[vSyncarr_index++] = numPatterns_inVsync;
            g_frameSyncArr[frameNumber_index++] = frame_sync_count;
        }
        return frameNumber_index; //starting with zero
    }
    else
    {
        return NUM_FRAMEBUFFERS;
    }
}
#endif

void MainWindow::apply_scan_config_to_ui(uScanConfig *pConfig)
/**
 *This function applies the current selected scan configuration to the repsective GUI controls on Scan Tab
 *@param pConfig - I - the current slected scan config
 *
 */
{
    uint16 end_nm;
    uint16 num_patterns;
    SCAN_TYPES type;

    if(pConfig->scanCfg.scan_type != SLEW_TYPE)
    {
        ui->label_start_nm_val->setText(QString::number(pConfig->scanCfg.wavelength_start_nm));
        ui->label_end_nm_val->setText(QString::number(pConfig->scanCfg.wavelength_end_nm));
        ui->spinBox_numRepeat->setValue(pConfig->scanCfg.num_repeats);
        ui->label_num_points_val->setText(QString::number(pConfig->scanCfg.num_patterns));
        if(pConfig->scanCfg.scan_type == COLUMN_TYPE)
            ui->label_scan_method_val->setText("Column");
        else if(pConfig->scanCfg.scan_type == HADAMARD_TYPE)
            ui->label_scan_method_val->setText("Hadamard");

    }
    else
    {
        end_nm = dlpspec_scan_slew_get_end_nm(&pConfig->slewScanCfg);
        num_patterns = dlpspec_scan_slew_get_num_patterns(&pConfig->slewScanCfg);
        ui->label_start_nm_val->setText(QString::number(pConfig->slewScanCfg.section[0].wavelength_start_nm));
        ui->label_end_nm_val->setText(QString::number(end_nm));
        ui->spinBox_numRepeat->setValue(pConfig->slewScanCfg.head.num_repeats);
        ui->label_num_points_val->setText(QString::number(num_patterns));
        double scanTime =  (float)NNO_GetEstimatedScanTime()/1000.0;
        ui->label_scan_time_val->setText(QString::number(scanTime));
        type = dlpspec_scan_slew_get_cfg_type(&pConfig->slewScanCfg);
        if(type == COLUMN_TYPE)
            ui->label_scan_method_val->setText("Column");
        else if(type == HADAMARD_TYPE)
            ui->label_scan_method_val->setText("Hadamard");
        else if(type == SLEW_TYPE)
            ui->label_scan_method_val->setText("Slew");
#if 0 //added only for debug
        Scan_SetFrameSyncs(pConfig);
#endif
    }

}

void MainWindow::PopulateScanCfgListComboBox()
/**
 *This function populates the combobox with local and target config lists
 *
 */
{  
    int i;
    uScanConfig config;
    int index;

    ui->comboBox_scanID->clear();
    //temporarily disable signals while we are populating the combobox
    ui->comboBox_scanID->blockSignals(true);

    for(i = 0; i < scancfglist.count(); i++)
    {
        if(scancfglist.GetItemAt(i, &config) != PASS)
            break;
        ui->comboBox_scanID->addItem(config.scanCfg.config_name);
    }

    //This is to ensure that when currentIndex is set correctly below;
    //on_comboBox_scanID_currentIndexChanged event is triggered.
    ui->comboBox_scanID->setCurrentIndex(-1);
    //enable signals again now that the list is populated
    ui->comboBox_scanID->blockSignals(false);
    index = scancfglist.GetLocalList().count();
    if(scancfglist.Get_ActiveConfigIndex() < 0)
        index = 0;
    else
        index += scancfglist.Get_ActiveConfigIndex();
    ui->comboBox_scanID->setCurrentIndex(index);
}

void MainWindow::Populate_ScanList()
    /**
	 * This function displays the saved scans in the PC on the Saved Scans list in the Scan Tab
	 *
	 */
{
    ui->listWidget_savedscans->blockSignals(true);
    ui->listWidget_savedscans->clear();
    QStringList fileArray;
    int i = 0;
    QDirIterator dirIt(filepath.GetdisplayScanSettingsDir());
    while (dirIt.hasNext())
    {
        dirIt.next();
        if (QFileInfo(dirIt.filePath()).isFile())
        {
            if(dirIt.filePath().contains("ref_"))
                continue;
            if (QFileInfo(dirIt.filePath()).suffix() == "dat")
            {
                QString file = filepath.GetdisplayScanSettingsDir().relativeFilePath(dirIt.filePath());
                fileArray.append(file);
            }
        }
    }
    for(i = 0; i < fileArray.size(); i++)
        ui->listWidget_savedscans->addItem(fileArray[i]);

    ui->listWidget_savedscans->blockSignals(false);
}
//handler functions for different widget slots on the Scan Tab

void MainWindow::GetPGASetting(bool *is_fixed, int *fixed_pga_val)
{
    *is_fixed = true;

    switch(ui->comboBox_pga_gain->currentIndex())
    {
    case 0:
        *is_fixed = false;
        return;
    case 1:
        *fixed_pga_val = 64;
        return;
    case 2:
        *fixed_pga_val = 32;
        return;
    case 3:
        *fixed_pga_val = 16;
        return;
    case 4:
        *fixed_pga_val = 8;
        return;
    case 5:
        *fixed_pga_val = 4;
        return;
    case 6:
        *fixed_pga_val = 2;
        return;
    case 7:
        *fixed_pga_val = 1;
        return;
    }
}

void MainWindow::on_pushButton_scan_clicked()
/**
 * This is a handler function for pushButton_scan on Scan Tab clicked() event
 * This function does the following tasks
 * Checks for USB connection
 * gets the selected Scan Configuration parameters - estimates the scan time and displays
 * does the scan by calling the corresponding API functions
 * saves the scan results in .csv and .bat files in user settings directory
 * displays the spectrum - plots the scan data on the GUI
 */
{
    void *pData;
    int scanStatus;
    int fileSize;
    int cont_scans_num = 1;
    int cont_scans_delay = 0;
    QString scanTimeText;
    bool is_fixed;
    int pga_val;

    if(!USB_IsConnected())
    {
        showError("EVM not connected");
        return;
    }
    pData = (scanData *)malloc(SCAN_DATA_BLOB_SIZE);
    if(pData == NULL)
    {
        showError("out of memory\n");
        return;
    }
    //ui->label_wait_plot->setVisible(true);
    QApplication::setOverrideCursor(Qt::WaitCursor);
    ui->pushButton_scan->repaint();
    if(ui->radioButton_Prefix->isChecked())
        spectrum.SetSaveFileNamePrefix(ui->lineEdit_Prefix->text());
    else
        spectrum.SetSaveFileName(ui->lineEdit_FileName->text());

    GetPGASetting(&is_fixed, &pga_val);
    NNO_SetFixedPGAGain(is_fixed, pga_val);


     cont_scans_num = ui->spinBox_cont_scan_num->value();
     cont_scans_delay = ui->spinBox_cont_scan_delay->value();

    	do { 
			pData = (scanData *)malloc(SCAN_DATA_BLOB_SIZE);
			if(pData == NULL)
			{
				showError("out of memory\n");
				return;
			}
			//ui->label_wait_plot->setVisible(true);
			QApplication::setOverrideCursor(Qt::WaitCursor);
			ui->pushButton_scan->repaint();
			if(ui->radioButton_Prefix->isChecked())
				spectrum.SetSaveFileNamePrefix(ui->lineEdit_Prefix->text());
			else
				spectrum.SetSaveFileName(ui->lineEdit_FileName->text());

			SetGUIControls(DISABLE);
			scanStatus = PerformScanReadData(NNO_DONT_STORE_SCAN_IN_SD, ui->spinBox_numRepeat->value(), pData, &fileSize);
			SetGUIControls(ENABLE);

			QApplication::processEvents();
			if(scanStatus != PASS)
			{
				free(pData);
				showError("Scan Failed");
				QApplication::setOverrideCursor(Qt::ArrowCursor);
				return;
			}

			if(m_ref_selection == SCAN_REF_FACTORY)
			{
				if(spectrum.SetData(pData, evm.GetRefCalDataBlob()) != PASS)
				{
					showError("scan or reference data interpret failed");
					free(pData);
					QApplication::setOverrideCursor(Qt::ArrowCursor);
					return;
				}
				PlotSpectrum();
				if(spectrum.SaveToFile() != PASS)
				{
					showError("Save scan to disk failed");
				}
				else
				{
					scanTimeText = ui->labelScanStatus->text();
					QString msg = QString("%1.Results saved in %2").arg(scanTimeText, filepath.GetsaveScanSettingsDir().absolutePath());
					ui->labelScanStatus->setText(msg);
				}
			}
			else if(m_ref_selection == SCAN_REF_NEW)
			{
				if(spectrum.SetData(pData, pData) != PASS)
				{
					showError("scan or reference data interpret failed");
					free(pData);
					QApplication::setOverrideCursor(Qt::ArrowCursor);
					return;
				}
				spectrum.SaveReferenceToFile();
				ui->radioButton_ref_prev->setChecked(true);
				on_radioButton_ref_prev_clicked(true);
			   // ui->label_wait_plot->setVisible(false);
				QApplication::setOverrideCursor(Qt::ArrowCursor);
			}
			else if (m_ref_selection == SCAN_REF_PREV)
			{
				if(spectrum.SetData(pData, prevRefDataBlob) != PASS)
				{
					showError("scan or reference data interpret failed");
					free(pData);
					QApplication::setOverrideCursor(Qt::ArrowCursor);
					return;
				}
				/* Display reference file timestamp */
				ui->label_ref_timestamp->setText(QString("Reference last set on : %1").arg(spectrum.GetReferenceTimeStamp()));
				PlotSpectrum();        
				if(spectrum.SaveToFile() != PASS)
				{
					showError("Save scan to disk failed");
				}
			}
			free(pData);
			
			// Add delay between back-to-back scans minus the lamp warm up time
			QTime waitTime = QTime::currentTime().addMSecs(cont_scans_delay*1000 - 625);
            while ( QTime::currentTime() < waitTime ) {
                QApplication::processEvents();
			}


			Populate_ScanList(); //re populate the scan list from the selected directory
			QApplication::processEvents();  
			
            ui->spinBox_cont_scan_num->setValue(--cont_scans_num);
    	
        } while ( cont_scans_num > 0);

}

void MainWindow::on_pushButton_interpret_clicked()
/**
 * This is a handler function for pushButton_interpret on Scan Tab clicked() event
 * This function does the following tasks
 * Checks for USB connection
 * forces Tiva to interpret the scan by calling the corresponding API functions
 * saves the scan results in .csv and .bat files in user settings directory
 * displays the spectrum - plots the scan data on the GUI
 */
{
    void *pData;
    int scanStatus;
    int fileSize;

    if(!USB_IsConnected())
    {
        showError("EVM not connected");
        return;
    }
    pData = (scanData *)malloc(SCAN_DATA_BLOB_SIZE);
    if(pData == NULL)
    {
        showError("out of memory\n");
        return;
    }
    //ui->label_wait_plot->setVisible(true);
    QApplication::setOverrideCursor(Qt::WaitCursor);
    ui->pushButton_scan->repaint();
    if(ui->radioButton_Prefix->isChecked())
        spectrum.SetSaveFileNamePrefix(ui->lineEdit_Prefix->text());
    else
        spectrum.SetSaveFileName(ui->lineEdit_FileName->text());

    SetGUIControls(DISABLE);
    scanStatus = InterpretScanData( pData, &fileSize);
    SetGUIControls(ENABLE);

    QApplication::processEvents();
    if(scanStatus != PASS)
    {
        free(pData);
        showError("Scan Interpret Failed");
        return;
    }

    if(m_ref_selection == SCAN_REF_FACTORY)
    {
 		if(spectrum.SetInterpretData(pData) != PASS)
        {
            showError("reference data interpret failed");
            free(pData);
            return;
        }   	
        PlotSpectrum();
        if(spectrum.SaveToFile() != PASS)
        {
            showError("Save scan to disk failed");
        }
    }
    free(pData);

    Populate_ScanList(); //re populate the scan list from the selected directory
    QApplication::processEvents();
}

void MainWindow::on_comboBox_scanID_currentIndexChanged(int index)
/**
 * This function is handler for comboBox_scanID on Scan tab currentIndexChanged() event
 * Based on the scan Type that is slecetd, it selectively enables/disables the repsective
 * GUI controls and populates the selected scan config values in the respective fields
 * @param index - I - index of the current selected scan
 *
 */
{
    uScanConfig config;

    if(index < 0)
        return;
    if(!USB_IsConnected())
    {
        showError("EVM not connected");
        return;
    }

    ui->pushButton_scan->setText("Wait");
    ui->pushButton_scan->setEnabled(false);
    ui->pushButton_scan->repaint();

    spectrum.ResetPlot();

    ui->radioButton_ref_factory->setEnabled(true);
    ui->radioButton_ref_new->setEnabled(true);
    ui->radioButton_ref_prev->setEnabled(true);

    if(PASS != scancfglist.GetItemAt(index, &config))
    {
        showError("Invalid selector");
        return;
    }

    if(evm.ApplyScanCfgtoDevice(&config) < 0)
    {
        showError("Error applying the scan config. Double check the scanConfig parameters");
        return;
    }

    apply_scan_config_to_ui(&config);

    /* Clear reference file timestamp */
    ui->label_ref_timestamp->setText(QString(""));

    if(m_ref_selection == SCAN_REF_PREV)
    {
        /* Read the reference again for the new selection */
        on_radioButton_ref_prev_clicked(true);
    }
    ui->pushButton_scan->setEnabled(true);
    ui->pushButton_scan->setText("Scan");
    ui->pushButton_scan->repaint();
}

void MainWindow::on_pushButton_edit_scan_config_clicked()
/**
 *This function is a handler for pushButton_edit_scan_config on Scan Tab and clicked() event
 * Dsiplays the Scan Configuration dialog
 */
{
    scan_cfg_dialog.Init(&scancfglist);
    int result = scan_cfg_dialog.exec();
    //Re-populate scan cfg list combobox to reflect any edits
    if(scan_cfg_dialog.Accepted == result) // only if OK is clicked on Scan Configh Dialog
        PopulateScanCfgListComboBox();
}

void MainWindow::on_pushButton_dir_change_clicked()
/**
 * This function is a handler for pushButton_Settings_Dir on Scan Tab, clicked() event
 * Change the directory from which the scans on PC are displyed in the Saved Scans list
 *
 */
{
    QDir dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                               filepath.GetdisplayScanSettingsDirPath(),
            QFileDialog::ShowDirsOnly
            | QFileDialog::DontResolveSymlinks);
    //Check if dir is empty because of user cancelling the directory select dialog
    if(dir == QDir(""))
        return;

    filepath.displayScanSettingsDir = dir;
    filepath.SetdisplayScanSettingsDirPath(dir.absolutePath());
    ui->label_scan_directory->setText(filepath.GetdisplayScanSettingsDir().absolutePath());
    ui->label_scan_directory->setToolTip(filepath.GetdisplayScanSettingsDir().absolutePath());
    Populate_ScanList(); //re populate the scan list from the selected directory
    QApplication::processEvents();
}

void MainWindow::on_pushButton_customSaveScanDir_clicked()
{
    QDir dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                            filepath.GetsaveScanSettingsDirPath(),
            QFileDialog::ShowDirsOnly
            | QFileDialog::DontResolveSymlinks);
    //Check if dir is empty because of user cancelling the directory select dialog
    if(dir == QDir(""))
        return;

    filepath.saveScanSettingsDir = dir;
    filepath.SetsaveScanSettingsDirPath(dir.absolutePath());
    ui->pushButton_customSaveScanDir->setToolTip(filepath.GetsaveScanSettingsDir().absolutePath());
    ui->lineEdit_scan_save_dir->setText(filepath.GetsaveScanSettingsDir().absolutePath());
    QApplication::processEvents();
}

void MainWindow::on_listWidget_savedscans_currentItemChanged(QListWidgetItem *current, QListWidgetItem *previous)
{
    slewScanConfig scan_cfg;
    uint16 start_nm;
    uint16 end_nm;
    uint16 num_patterns;
    SCAN_TYPES type;

    //check if no item was selected
    if(current == NULL)
      return;

    if(previous)
    {
     //do nothing
    }

    QString scanFileName = current->text();
    QString scanFileNameFull = filepath.GetdisplayScanSettingsDir().absoluteFilePath(scanFileName);

    filepath.m_selectedScanFile = scanFileNameFull;
    if(spectrum.ReadSavedScanFromFile(scanFileNameFull) != PASS)
    {
        showError("Error interpreting saved scan");
        return;
    }

    scan_cfg = spectrum.GetScanConfig();

    start_nm = scan_cfg.section[0].wavelength_start_nm;
    end_nm = dlpspec_scan_slew_get_end_nm(&scan_cfg);
    num_patterns = dlpspec_scan_slew_get_num_patterns(&scan_cfg);

    ui->label_start_nm_val_2->setText((QString::number(start_nm)));
    ui->label_end_nm_val_2->setText((QString::number(end_nm)));
    ui->label_numRepeat->setText(QString::number(scan_cfg.head.num_repeats));
    ui->label_num_points_val_2->setText(QString::number(num_patterns));

    type = dlpspec_scan_slew_get_cfg_type(&scan_cfg);
    if(type == COLUMN_TYPE)
        ui->label_scan_method_val_2->setText("Column");
    else if(type == HADAMARD_TYPE)
        ui->label_scan_method_val_2->setText("Hadamard");
    else if(type == SLEW_TYPE)
        ui->label_scan_method_val_2->setText("Slew");

    PlotSpectrum();
}

void MainWindow::PlotSpectrum()
{
    PLOT_TYPE type;
   // ui->label_wait_plot->setVisible(true);
    QApplication::setOverrideCursor(Qt::WaitCursor);

    if(ui->pushButton_Intensity->isChecked())
        type = PLOT_INTENSITY;
    else if(ui->pushButton_Absorbance->isChecked())
        type = PLOT_ABSORBANCE;
    else if(ui->pushButton_Reflectance->isChecked())
        type = PLOT_REFLECTANCE;
    else
    {
        QApplication::setOverrideCursor(Qt::ArrowCursor);
        return;
    }

    if(ui->checkBox_overlay->isChecked() == false)
        spectrum.ResetPlot();
    mark_labels(type);

    if(!spectrum.Plot(type,scene))
    {
        QString msg = QString("Reference/Sample magnitude is too low to compute absorbance in some areas.");
        ui->labelScanStatus->setText(msg);
    }
    ui->graphicsView->fitInView(scene->sceneRect());
    //ui->label_wait_plot->setVisible(false);
    QApplication::setOverrideCursor(Qt::ArrowCursor);

}


void MainWindow::on_pushButton_sidebar_hide_clicked()
{
    static QLayoutItem* pSideBar;
    static bool IsSideBarHidden = false;

   // QRectF bounds_scene = scene->itemsBoundingRect();
    //QRectF bound_view = ui->graphicsView->rect();
    //QRectF bound_groupbox = ui->groupBox_16->rect();

    if(IsSideBarHidden)
    {
        IsSideBarHidden = false;
        ui->horizontalLayout_77->insertItem(0, pSideBar);
        ui->pushButton_sidebar_hide->setText("<");
        ui->frame_sidebar->show();
    }
    else
    {
        IsSideBarHidden = true;
        pSideBar = ui->horizontalLayout_77->takeAt(0);
        ui->horizontalLayout_77->removeItem(pSideBar);
        ui->frame_sidebar->hide();
        ui->pushButton_sidebar_hide->setText(">");
    }

    if(scene)
    {
        PlotSpectrum();
        ui->graphicsView->fitInView(scene->sceneRect());
     }
    QApplication::processEvents();
}

void MainWindow::on_radioButton_ref_factory_clicked(bool checked)
{
    if(checked)
    {
        m_ref_selection = SCAN_REF_FACTORY;
        ui->pushButton_scan->setText("Scan Sample");
    }
}

void MainWindow::on_radioButton_ref_prev_clicked(bool checked)
{
    uScanConfig config;
    char ser_num[NANO_SER_NUM_LEN];

    if(checked)
    {
        m_ref_selection = SCAN_REF_PREV;

        if(PASS == scancfglist.GetItemAt(ui->comboBox_scanID->currentIndex(), &config))
        {
            NNO_GetSerialNumber(ser_num);
            strncpy(config.scanCfg.ScanConfig_serial_number, ser_num, NANO_SER_NUM_LEN);
            if(spectrum.ReadReferenceFromFile(prevRefDataBlob, &config) == PASS)
            {
                spectrum.SetData(NULL, prevRefDataBlob);
                /* Display reference file timestamp */
                ui->label_ref_timestamp->setText(QString("Reference last set on : %1").arg(spectrum.GetReferenceTimeStamp()));
                ui->pushButton_scan->setText("Scan Sample");
            }
            else
            {
                /* Clear reference file timestamp */
                ui->label_ref_timestamp->setText(QString("Previous Reference not found"));
                ui->radioButton_ref_new->setChecked(true);
                emit on_radioButton_ref_new_clicked(true);
            }
        }
    }
}

void MainWindow::on_radioButton_ref_new_clicked(bool checked)
{
    if(checked)
    {
        m_ref_selection = SCAN_REF_NEW;
        ui->pushButton_scan->setText("Scan Reference");
    }
}

void MainWindow::on_pushButton_Intensity_clicked()
{
    ui->pushButton_Absorbance->setChecked(false);
    ui->pushButton_Reflectance->setChecked(false);

    ui->checkBox_overlay->setChecked(false);
    PlotSpectrum();
}

void MainWindow::on_pushButton_Absorbance_clicked()
{
    ui->pushButton_Intensity->setChecked(false);
    ui->pushButton_Reflectance->setChecked(false);

    ui->checkBox_overlay->setChecked(false);
    PlotSpectrum();
}

void MainWindow::on_pushButton_Reflectance_clicked()
{
    ui->pushButton_Intensity->setChecked(false);
    ui->pushButton_Absorbance->setChecked(false);

    ui->checkBox_overlay->setChecked(false);
    PlotSpectrum();
}

void MainWindow::on_checkBox_overlay_clicked()
{
    if(ui->checkBox_overlay->isChecked() == false)
    {
        //scene->clear();
        spectrum.ResetPlot();
        PlotSpectrum();
    }
}

void MainWindow::on_pushButton_import_scanData_clicked()
{
    int fileSize;
    uint8 *pData;
    int i;

    for(i=ui->label_num_unread_scandata->text().toInt()-1; i>=0; i--)
    {
        fileSize = NNO_GetFileSizeToRead(NNO_FILE_SCAN_DATA_FROM_SD);

        if(fileSize <= 0)
        {
            showError("Unable to read file from EVM");
            return;
        }

        pData = (uint8 *)malloc(fileSize);
        if(pData == NULL)
            return;

        if(NNO_GetFile((unsigned char *)pData, fileSize) == fileSize)
        {
            spectrum.SetData(pData, evm.GetRefCalDataBlob());
            if(spectrum.SaveToFile() != PASS)
            {
                showError("Save scan to disk failed");
            }
            NNO_DeleteLastScanFileInSD();
        }
        else
        {
            free(pData);
            return;
        }

        free(pData);
    }
    get_num_files_in_SD_card();
    Populate_ScanList();
}

void MainWindow::on_radioButton_Prefix_clicked(bool checked)
{
    if(checked)
    {
        ui->lineEdit_FileName->setEnabled(false);
        ui->lineEdit_Prefix->setEnabled(true);
    }
}

void MainWindow::on_radioButton_FileName_clicked(bool checked)
{
    if(checked)
    {
        ui->lineEdit_FileName->setEnabled(true);
        ui->lineEdit_Prefix->setEnabled(false);
    }
}
void MainWindow::on_spinBox_numRepeat_editingFinished()
{
    int numrepeats = ui->spinBox_numRepeat->value();
    NNO_SetScanNumRepeats(numrepeats);
    double scanTime = ((float)NNO_GetEstimatedScanTime() / 1000);
    ui->label_scan_time_val->setText(QString::number(scanTime));
}

void MainWindow::on_spinBox_numRepeat_valueChanged(int arg1)
{
    NNO_SetScanNumRepeats(arg1);
    double scanTime = ((float)NNO_GetEstimatedScanTime() / 1000);
    ui->label_scan_time_val->setText(QString::number(scanTime));
}
