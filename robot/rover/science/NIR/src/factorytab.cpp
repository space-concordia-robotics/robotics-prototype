#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "goldensample.h"
#include <QTimer>
#include <QTime>


/*Global variable */
static calibCoeffs calib_coeffs;
static double myRefPeakPos[] = {452, 470, 516, 560,723,784,824};
static int numRefPeakPos_internal = 7;
extern EVM evm;
extern FilePath filepath;

#define NUM_RM_NIR_PEAKS 2
#define NUM_AR1_PEAKS 8
#define NUM_AR1_PEAKS_USED 7
#define TOTAL_PEAKS 9
#define NUM_PEAKS_BENDING 4
#define START_RMNIR_COMPARE 0.96
#define STOP_RMNIR_COMPARE 2.5
#define WAVELENGTH_ACCURACY_LIMIT 2
const double EXPECTED_PEAK_WAVELENGTHS[TOTAL_PEAKS] =  {1693.1,1446.6,1373.7,1353.8,1298.6,1245.6,1048.8,967.8,917.2};
const double WAVELENGTH_ERROR_LIMIT_ARR[6] = {0.6,2.2,3.4,0.9,0.2,3.7};

void MainWindow::SetDetAlignSliderMaxVal(int val)
    /**
	 * This function sets the Maximum Limit fir the Detector align Calibration Sliders in Factory Tab
	 * @param val - I - the value that should be set as Maximum
	 *
	 */
{
	ui->verticalSlider_det_align_1->setMaximum(val);
	ui->verticalSlider_det_align_2->setMaximum(val);
	ui->verticalSlider_det_align_3->setMaximum(val);
	ui->verticalSlider_det_align_Max1->setMaximum(val);
	ui->verticalSlider_det_align_Max2->setMaximum(val);
	ui->verticalSlider_det_align_Max3->setMaximum(val);
	ui->verticalSlider_det_align_Min1->setMaximum(val);
	ui->verticalSlider_det_align_Min2->setMaximum(val);
	ui->verticalSlider_det_align_Min3->setMaximum(val);
}

/* Review comment - SS
 * Move cosine similarity function to spectrum class?
 */

void MainWindow::compare_absorption_spectrum(double *testAbsorbance, double *refAbsorbance, int vectorLength, double simThreshold, bool *isSimilar, double *simNumber)
    /**
	 * This function compares the absorbance with known good absorbance spectrum of the test samplee using cosine_similarity
	 * This is Called during the System test Calibration.
	 * If cosine_similarity exists, then the System Test Callibration is Pass/ it is Fail
	 * @param testAbsorbance - I - absorbance data of the current scan during calibration
	 * @param refAbsorbance - I - known good refrence
	 * @param vectorLength - I - length of the abosorbance vector
     * @param simThreshold - I - threshhold for comparision
	 * @param isSimilar - O - boolean to indicate if the absorption spectrum is similar or not
	 * @param simNumber - O - calculated cosine similarity number
	 *
	 */
{
	double sumAiXBi;
	double sumAiXAi;
	double sumBiXBi;
	int i;
	double similarityNum = -1.0; //Cos(theta) = -1 to 1

	//http://en.wikipedia.org/wiki/Cosine_similarity => equation used to compare two vectors
	sumAiXBi = 0.0;
	sumAiXAi = 0.0;
	sumBiXBi = 0.0;
	for(i=0; i<vectorLength; i++)
	{
		sumAiXBi += (double) (testAbsorbance[i] * refAbsorbance[i]);
		sumAiXAi += (double) (testAbsorbance[i] * testAbsorbance[i]);
		sumBiXBi += (double) (refAbsorbance[i] * refAbsorbance[i]);
	}

	similarityNum = (double) ( sumAiXBi / ( (sqrt(sumAiXAi) * (sqrt(sumBiXBi)) ) ) );

	if(similarityNum <= simThreshold)
		*isSimilar = FALSE;
	else
		*isSimilar = TRUE;

    if(std::isnan(similarityNum) == true)
		*isSimilar = FALSE;

	printf("Computed Cosine Similarity - %f\n",similarityNum);

	*simNumber = similarityNum;

	return;
}

void MainWindow::initDetectorAlignment(void)
    /**
	 * This function initializes the Detector Align slider values
	 * If there is a file that is saved previously, the initialization values are taken from that
	 * Else the sliders are initialized to default values
	 */
{

	QFile file(filepath.GetspecDetFileName());

	if(!file.exists())
	{
		minAlignment_Short = DET_ALIGN_MIN1;
		minAlignment_Medium = DET_ALIGN_MIN2;
		minAlignment_Long = DET_ALIGN_MIN3;

		file.open(QIODevice::WriteOnly | QIODevice::Text);
		QTextStream out(&file);
		out << minAlignment_Short << "\n";
		out << minAlignment_Medium << "\n";
		out << minAlignment_Long << "\n";
		file.close();
	}
	else
	{
		file.open(QIODevice::ReadOnly | QIODevice::Text);
		QTextStream in(&file);
		minAlignment_Short = in.readLine().toInt();
		minAlignment_Medium = in.readLine().toInt();
		minAlignment_Long = in.readLine().toInt();
		file.close();
	}

	SetDetAlignSliderMaxVal(1000000);
	/* Set the minimum acceptable values */
	ui->verticalSlider_det_align_Min1->setValue(minAlignment_Short);
	ui->lineEdit_Det_Min1->setText(QString::number(minAlignment_Short));
	ui->verticalSlider_det_align_Min2->setValue(minAlignment_Medium);
	ui->lineEdit_Det_Min2->setText(QString::number(minAlignment_Medium));
	ui->verticalSlider_det_align_Min3->setValue(minAlignment_Long);
	ui->lineEdit_Det_Min3->setText(QString::number(minAlignment_Long));
}

void MainWindow::initSlitAlignment(void)
    /**
	 * This function initializes the Slit Align slider values
	 * If there is a file that is saved previiously, the initialization values are taken from that
	 * Else the sliders are initialized to default values
	 */
{
	QFile file(filepath.GetspecSlitFileName());

	if(!file.exists())
	{
		minAlignment_Short = SLIT_ALIGN_MIN1;
		minAlignment_Medium = SLIT_ALIGN_MIN2;
		minAlignment_Long = SLIT_ALIGN_MIN3;

		file.open(QIODevice::WriteOnly | QIODevice::Text);
		QTextStream out(&file);
		out << minAlignment_Short << "\n";
		out << minAlignment_Medium << "\n";
		out << minAlignment_Long << "\n";
		file.close();
	}
	else
	{
		file.open(QIODevice::ReadOnly | QIODevice::Text);
		QTextStream in(&file);
		minAlignment_Short = in.readLine().toInt();
		minAlignment_Medium = in.readLine().toInt();
		minAlignment_Long = in.readLine().toInt();
		file.close();
	}

	ui->verticalSlider_slit_align_Min1->setValue(minAlignment_Short);
	ui->lineEdit_Slit_Min1->setText(QString::number(minAlignment_Short));
	ui->verticalSlider_slit_align_Min2->setValue(minAlignment_Medium);
	ui->lineEdit_Slit_Min2->setText(QString::number(minAlignment_Medium));
	ui->verticalSlider_slit_align_Min3->setValue(minAlignment_Long);
	ui->lineEdit_Slit_Min3->setText(QString::number(minAlignment_Long));
}

/* Review comment - SS
 * Need to see if this function can be moved back to spectrum Library.
 * It was observed to give wrong results when made part of spectrum library earlier. Need
 * to investigate and see if the problem persists
 */

int dlpspec_calib_checkPeakDist(double *peak_pos, int num_values, double *ref_pos, double tolerance)
    /**
	 * Checks if the peaks conforms with the expected distribution and retuns the result
	 *
	 * @param   peak_pos 	- I - Pointer to the buffer that contains peak positions
	 * @param   num_values 	- I - Number of values present in values buffer
	 * @param   ref_pos 	- I - Pointer to the buffer that contains reference peak positions; please ensure that there are as many ref pos as peak_pos
	 * @param   tolerance 	- I - tolerance to be used while checking relative position
	 *
	 * @return  result of the check
	 *          <0 = FAIL
	 *
	 */
{
	double *ref_pos_vals;
	int ret_val = 0;
	int i = 0;

	if ((num_values <= 2) || (peak_pos == NULL) || ((ref_pos == NULL) && (num_values != numRefPeakPos_internal)))
		return ERR_DLPSPEC_INVALID_INPUT;

	// check input reference data and used the hard-coded values when not present
	if (ref_pos == NULL)
		ref_pos_vals = myRefPeakPos;
	else
		ref_pos_vals = ref_pos;

	for (i=2; i < num_values;i++)
	{
		double peak_pos_temp = (double)(peak_pos[i]-peak_pos[i-1])/(double)(peak_pos[i-1]-peak_pos[i-2]);
		double ref_pos_temp = (double)(ref_pos_vals[i]-ref_pos_vals[i-1])/(double)(ref_pos_vals[i-1]-ref_pos_vals[i-2]);

		if ((peak_pos_temp/ref_pos_temp >= (1 - tolerance)) &&
				(peak_pos_temp/ref_pos_temp <= (1 + tolerance)))
		{
		}
		else
		{
			ret_val = ERR_DLPSPEC_FAIL;
			break;
		}
	}

	return ret_val;
}

/* Review comment - SS
 * See if this function can be made part of spectrum class
 */
double MainWindow::GetInterpolatedWavelength(double pixel)
    /**
	 * This function gets the interpolated wavelength for a particular pixel
	 * @param pixel - I - the pixel for which the interpolated wavelength to be calculated
	 * @return the interpolated wavelength as double
	 *
	 */
{
	int index = floor(pixel);
	double lambda1 = spectrum.GetWavelengths()[index];
	index = ceil(pixel);
	double lambda2 = spectrum.GetWavelengths()[index];
	return (lambda2 - (ceil(pixel) - pixel)*(lambda2 - lambda1));
}

int MainWindow::PerformFullDMDScan(CALIB_SCAN_TYPES type, double *values, int *pSampleLen)
{
	int scanStatus;
    int fileSize;
	char pData[SCAN_DATA_BLOB_SIZE];
	scanResults scan_results;
	int i;
	int sampleLen=0;

	if(type == LEFT_DMD_TOP_SCAN)
	{
		ui->label_scan_progress->setText("DMD Top scan in progress");
        QApplication::processEvents(); //Update the GUI
		if(evm.GenCalibPatterns(LEFT_DMD_TOP_SCAN) == FAIL)
			return FAIL;
		type = RIGHT_DMD_TOP_SCAN;
	}
	else if(type == LEFT_DMD_MID_SCAN)
	{
		ui->label_scan_progress->setText("DMD Mid scan in progress");
        QApplication::processEvents(); //Update the GUI
		if(evm.GenCalibPatterns(LEFT_DMD_MID_SCAN) == FAIL)
			return FAIL;
		type = RIGHT_DMD_MID_SCAN;
	}
	else if(type == LEFT_DMD_BOT_SCAN)
	{
		ui->label_scan_progress->setText("DMD Bot scan in progress");
        QApplication::processEvents(); //Update the GUI
		if(evm.GenCalibPatterns(LEFT_DMD_BOT_SCAN) == FAIL)
			return FAIL;
		type = RIGHT_DMD_BOT_SCAN;
	}
	else
	{
		return FAIL;
	}

    scanStatus = PerformScanReadData(NNO_DONT_STORE_SCAN_IN_SD, NUM_REPEATS_CALIBRATION, pData, &fileSize);

	if(scanStatus != PASS)
    {
		return FAIL;
    }
    /* This is the second step of calculating Spectrum */
    ui->progressbar_cal->setValue(ui->progressbar_cal->value()+10);
    QApplication::processEvents(); //Update the GUI

	dlpspec_calib_interpret(pData, fileSize, &scan_results, LEFT_DMD_SCAN);
	for(i=0; i<scan_results.length; i++)
		values[sampleLen++] = (double)scan_results.intensity[i];

	if(evm.GenCalibPatterns(type) == FAIL)
		return FAIL;

    scanStatus = PerformScanReadData(NNO_DONT_STORE_SCAN_IN_SD, NUM_REPEATS_CALIBRATION, pData, &fileSize);

	if(scanStatus != PASS)
    {
		return FAIL;
    }
	ui->label_scan_progress->setText("Finding the peaks and checking against expected values");
    QApplication::processEvents(); //Update the GUI

	dlpspec_calib_interpret(pData, fileSize, &scan_results, RIGHT_DMD_SCAN);
	for(i=0; i<scan_results.length; i++)
		values[sampleLen++] = (double)scan_results.intensity[i];

	*pSampleLen = sampleLen;

    /* Clear subimage that gets set in the GenCalibPatterns function*/
    NNO_setScanSubImage(0, DMD_HEIGHT);

    return PASS;
}

int MainWindow::FindNPeaks(double *pValues, int sampleLen, int expectedNumPeaks, int *pPeakIndices)
/**
 * Tries to find expected_num_peaks peaks in the given array of pValues
 * Returns the actual number of peaks found. Also returns the indices of those peaks in pPeakIndices
 */
{
    double divisor = 0.0;
    int numPeaks;
    int i;

    for(i=1; i<=30; i++)
    {
        numPeaks = dlpspec_calib_findPeaks(pValues, sampleLen, i, pPeakIndices);
        for(divisor=i; divisor<i+1; divisor+=0.01)
        {
            numPeaks = dlpspec_calib_findPeaks(pValues, sampleLen, divisor, pPeakIndices);
            if(numPeaks >= expectedNumPeaks)
                break;
        }
        if(numPeaks >= expectedNumPeaks)
        {
            break;
        }
    }
    if(i>30)
    {
        showError("Unable to find peaks");
        return FAIL;
    }
    else
    {
        return numPeaks;
    }
}

int MainWindow::ScanRMNIRSample(double *pAbsorbance, int *pSampleLen)
{
    int i = 0;
    double rm_nir_ref[ADC_DATA_LEN];
    double rm_nir_ref_diff[ADC_DATA_LEN];
    double diff_mean = 0;
    double rm_nir_sample[ADC_DATA_LEN];
    double rm_nir_ref_thresh = 0;
    double absor_sig_thresh_pcnt = 0.01;
    QMessageBox::StandardButton reply;
    int retval = PASS;

    /* First perform a MID DMD scan with empty transmittance sampling module conneced */
    reply = QMessageBox::question(this, "Wavelength calibration", "Is empty transmittance sampling module connected?",
                                  QMessageBox::Yes|QMessageBox::No);

    if (reply == QMessageBox::No)
    {
        return FAIL;
    }

    retval = PerformFullDMDScan(LEFT_DMD_MID_SCAN, rm_nir_ref, pSampleLen);
    if(retval != PASS)
    {
        showError("Scan Failed");
        return FAIL;
    }

    /* Check to see if high signal first frame issue occurred */
    for (i = 1; i < *pSampleLen; ++i)
    {
        rm_nir_ref_diff[i] = abs(rm_nir_ref[i] - rm_nir_ref[i-1]);
        diff_mean += rm_nir_ref_diff[i];
    }
    diff_mean /= (*pSampleLen - 1);

    if( rm_nir_ref_diff[24] > (10*diff_mean) )
    {
        showError("Reference scan failed due to high first frame.");
        return FAIL;
    }

    /* Next perform a MID DMD scan with RM-NIR sample is placed in transmittance sampling module */
    reply = QMessageBox::question(this, "Wavelength calibration", "Is RM-NIR sample placed in sampling module?",
                                  QMessageBox::Yes|QMessageBox::No);

    if (reply == QMessageBox::No)
    {
        return FAIL;
    }

    retval = PerformFullDMDScan(LEFT_DMD_MID_SCAN, rm_nir_sample, pSampleLen);
    if(retval != PASS)
    {
        showError("Scan Failed");
        return FAIL;
    }

    /* Strip low signal areas */
    for (i = 0; i < *pSampleLen; ++i)
    {
        rm_nir_ref_thresh += rm_nir_ref[i];
    }
    rm_nir_ref_thresh = rm_nir_ref_thresh / *pSampleLen * absor_sig_thresh_pcnt;
    for (i = 0; i < *pSampleLen; ++i)
    {
        if(rm_nir_ref[i] < rm_nir_ref_thresh)
        {
            rm_nir_sample[i] = rm_nir_ref[i];
        }
    }

    /* Compute absorbance */
    for (i = 0; i < *pSampleLen; ++i)
    {
        pAbsorbance[i] = COMPUTE_ABSORPTION_VAL(rm_nir_sample[i], rm_nir_ref[i]);
        if(std::isnan(pAbsorbance[i]) == true)
            pAbsorbance[i] = 0;
    }

    return PASS;

}

int MainWindow::Find_RMNIR_Peaks(double *rm_nir_peak_indices_interp, int *p_num_rm_nir_peaks)
/**
 * This function initiates a full scan of the DMD with RMNIR calibration standard object, locates
 * the peaks in its absorption specturm and returns the locations of those peaks and the number of peaks
 * found
 */
{
    double absorbance[ADC_DATA_LEN];
    int rm_nir_peak_indices[ADC_DATA_LEN];
    int sampleLen;
    double pos_per_index = 0;
    double pos_start_index = 0;
    double rmnir_pos[ADC_DATA_LEN];
    double cur_pos = 0;
    int num_rmnir_absor = 0;
    double rmnir_absor[ADC_DATA_LEN];
    bool IsSimilar = false;
    double SimValue = 0;
    double pxOffset = 0;
    int i = 0;

    if(ScanRMNIRSample(absorbance, &sampleLen) == FAIL)
        return FAIL;

    /*Find the two peaks */
    if(FindNPeaks(absorbance, sampleLen, NUM_RM_NIR_PEAKS, rm_nir_peak_indices) != NUM_RM_NIR_PEAKS)
    {
        showError("Unable to find peaks");
        return FAIL;
    }

    /* Correct peak position for pattern offset */
    *p_num_rm_nir_peaks = NUM_RM_NIR_PEAKS;
    for(i = 0; i < NUM_RM_NIR_PEAKS; i++)
    {
        int index = rm_nir_peak_indices[i];
        dlpspec_calib_findPeaks3(absorbance[index-1], absorbance[index], absorbance[index+1], &pxOffset);
        rm_nir_peak_indices_interp[i] = rm_nir_peak_indices[i] + pxOffset + WAVELEN_CAL_PTN_CENTER_OFFSET;
    }

    /* Compute the start and end points of our RM-NIR absorbance spectrum to include when comparing to golden reference */
    if(rm_nir_peak_indices[1] - rm_nir_peak_indices[0] == 0)
    {
        showError("RM-NIR peaks not different");
        return FAIL;
    }
    pos_per_index = 1 / ((double)rm_nir_peak_indices[1] - (double)rm_nir_peak_indices[0]);
    pos_start_index = 1 - (rm_nir_peak_indices[0] * pos_per_index);
    num_rmnir_absor = 0;
    for(i = 0; i < ADC_DATA_LEN; i++)
    {
        cur_pos = pos_start_index + i * pos_per_index;
        if((cur_pos > START_RMNIR_COMPARE) && (cur_pos <= STOP_RMNIR_COMPARE))
        {
            rmnir_pos[num_rmnir_absor] = cur_pos;
            rmnir_absor[num_rmnir_absor] = absorbance[i];
            num_rmnir_absor++;
        }
    }

    if(dlpspec_interpolate_double_positions(knownRmnirPeak12Loc, rmnir_pos, rmnir_absor, KNOWN_RMNIR_LENGTH, num_rmnir_absor) != DLPSPEC_PASS)
    {
        showError("Unable to interpolate rm-nir spectrum for comparison");
        return FAIL;
    }

    compare_absorption_spectrum(rmnir_absor, knownRmnirAbsorbance, KNOWN_RMNIR_LENGTH, 0.90, &IsSimilar, &SimValue);
    if(!IsSimilar)
    {
        showError("Absorption spectrum for RM-NIR sample is suspect. Please check RM-NIR sample and try again.");
        return FAIL;
    }

    return PASS;
}

int MainWindow::SetPGAGain(int val)
{
    int gain_setting;

    gain_setting = NNO_GetPGAGain();
    if(gain_setting < 0)
    {
        showError("PGA read failed");
        return FAIL;
    }
    //Set PGA gain
    if(NNO_SetPGAGain(val) != PASS)
    {
        showError("PGA set failed");
        return FAIL;
    }
    if(NNO_GetPGAGain() != val)
    {
        showError("PGA readback failed");
        return FAIL;
    }

    return gain_setting;
}

int MainWindow::Find_AR1_Peaks(double *ar1_peak_indices, int *p_num_ar1_peaks_per_scan)
/**
 * This function performs a full scan of the DMD top, mid and bottom halfs with AR1 calibration
 * standard light source. It the finds the location of peaks on each of those scans and return
 * the locations in an array. Also returns the number of peaks located on each scan (top, mid, bot)
 */
{
    int sampleLen;
    int i;
    double pxOffset = 0;
    double tolerance = 0.5;
    double values[ADC_DATA_LEN];
    int peak_indices[ADC_DATA_LEN];
    double peak_indices_interp[ADC_DATA_LEN];
    QMessageBox::StandardButton reply;
    int retval = PASS;

    reply = QMessageBox::question(this, "Wavelength calibration", "Is AR-1 calibration light source connected?",
                                  QMessageBox::Yes|QMessageBox::No);

    if (reply == QMessageBox::No)
    {
        return FAIL;
    }

    for(int k = 0 ; k< SCAN_RANGE ;)
    {
        ui->progressbar_cal->setValue(k*25 + 25);
        switch(k)
        {
        case 0:
            retval = PerformFullDMDScan(LEFT_DMD_TOP_SCAN, values, &sampleLen);
            break;
        case 1:
            retval = PerformFullDMDScan(LEFT_DMD_MID_SCAN, values, &sampleLen);
            break;
        case 2:
            retval = PerformFullDMDScan(LEFT_DMD_BOT_SCAN, values, &sampleLen);
            break;
        }

        if(retval != PASS)
        {
            showError("Scan Failed");
            break;
        }

        /* Peak Indices are returned in peak_indices array*/
        if(FindNPeaks(values, sampleLen, NUM_AR1_PEAKS, peak_indices) != NUM_AR1_PEAKS)
        {
            reply = QMessageBox::question(this, "Wavelength calibration", "Unable to find expected number of peaks - Retry?",
                                          QMessageBox::Yes|QMessageBox::No);

            if (reply == QMessageBox::No)
            {
                retval = FAIL;
                break;
            }
            else
            {
                continue;
            }
        }

        /* Interpolate and find the most accurate peak position from the integer returned by FindNPeaks */
        for(i = 0; i < NUM_AR1_PEAKS; i++)
        {
            int index = peak_indices[i];
            dlpspec_calib_findPeaks3(values[index-1], values[index], values[index+1], &pxOffset);
            peak_indices_interp[i] = peak_indices[i] + pxOffset;
        }

        /* Skip third peak, since it contains too many close wavelengths and is not specific enough */
        /* Also add the pattern center offset so that the pattern numbers are converted to DMD column numbers */
        double peak_indices_used_interp[NUM_AR1_PEAKS_USED] = {
            peak_indices_interp[0] + WAVELEN_CAL_PTN_CENTER_OFFSET, \
            peak_indices_interp[1] + WAVELEN_CAL_PTN_CENTER_OFFSET, \
            peak_indices_interp[3] + WAVELEN_CAL_PTN_CENTER_OFFSET, \
            peak_indices_interp[4] + WAVELEN_CAL_PTN_CENTER_OFFSET, \
            peak_indices_interp[5] + WAVELEN_CAL_PTN_CENTER_OFFSET, \
            peak_indices_interp[6] + WAVELEN_CAL_PTN_CENTER_OFFSET, \
            peak_indices_interp[7] + WAVELEN_CAL_PTN_CENTER_OFFSET};

        /* Check whether the distribution is correct */
        if(dlpspec_calib_checkPeakDist(peak_indices_used_interp, NUM_AR1_PEAKS_USED , NULL , tolerance) < 0 )
        {
            reply = QMessageBox::question(this, "Wavelength calibration", "dlpspec_calib_checkPeakDist failed - Retry?",
                                          QMessageBox::Yes|QMessageBox::No);

            if (reply == QMessageBox::No)
            {
                retval = FAIL;
                break;
            }
            else
            {
                continue;
            }
        }

        for(int j=0 ; j<NUM_AR1_PEAKS_USED ; j++)
        {
            ar1_peak_indices[(k*NUM_AR1_PEAKS_USED)+j]= peak_indices_used_interp[j];
        }
        k++;
    }/*End of k for loop*/

    *p_num_ar1_peaks_per_scan = NUM_AR1_PEAKS_USED;
    return retval;

}

int MainWindow::DoWavelengthCalibration()
/**
 * This function does the wavlength calibration for the system
 * Calculates the peak locations ,the peak values & calibration co-efficients
 * Sends the calibration co-efficients to TIVA
 * These values are later svaed to a file on PC
 */
{
    double rm_nir_peak_locs[ADC_DATA_LEN];
    double ar1_peak_locs_top_mid_bot[ADC_DATA_LEN];
    int i = 0;
    int retval = PASS;
    QMessageBox::StandardButton reply;
    double peak_locs_for_curve[NUM_PEAKS_BENDING*SCAN_RANGE];
    double y_for_curve[3];
    double minWavelength=0, maxWavelength=0;
    int num_rmnir_peaks=0;
    int num_ar1_peaks_per_scan=0;
    int num_total_peaks= NUM_AR1_PEAKS + NUM_RM_NIR_PEAKS;
    int last_gain_setting;
    double *peak_locs_mid_scan;
    int err_result = 0;
    double rSquared = 0;
    double expected_peak_wavelengths[9];
    double r2_tolerance = 0.999;

    if(b_serNumSet == false)
    {
        showError("Serial Number must be set before wavelength calibration");
        return FAIL;
    }

    //Instead of DLPC and Lamp ON/OFF being controlled for each scan, we will control it for a set of scans
    NNO_SetScanControlsDLPCOnOff(false);
    NNO_DLPCEnable(true, false);

    /* First set PGA gain to 32 (64 is too high for transmissive sample) */
    last_gain_setting = SetPGAGain(32);

    if(Find_RMNIR_Peaks(rm_nir_peak_locs, &num_rmnir_peaks) != PASS)
        return FAIL;
    ui->progressbar_cal->setValue(25);

    /* Now set PGA gain to 64 for low-intensity AR-1 source */
    SetPGAGain(64);

    peak_locs_mid_scan = (double *)malloc(sizeof(double)*num_total_peaks);
    if(peak_locs_mid_scan == NULL)
    {
        showError("out of memory");
        return FAIL;
    }
    //Find x-y relationship - begin block
    do
    {
        if(Find_AR1_Peaks(ar1_peak_locs_top_mid_bot, &num_ar1_peaks_per_scan) != PASS)
            return FAIL;

        num_total_peaks = num_rmnir_peaks + num_ar1_peaks_per_scan;
        i=0;
        //insert rm-nir peaks at the beginning of AR lamp peaks
        if(num_rmnir_peaks != 0)
        {
            memcpy(&peak_locs_mid_scan[2], &ar1_peak_locs_top_mid_bot[num_ar1_peaks_per_scan], sizeof(double)*num_ar1_peaks_per_scan);
            peak_locs_mid_scan[1] = rm_nir_peak_locs[1];
            peak_locs_mid_scan[0] = rm_nir_peak_locs[0];
            expected_peak_wavelengths[i++] = EXPECTED_PEAK_WAVELENGTHS[0];
            expected_peak_wavelengths[i++] = EXPECTED_PEAK_WAVELENGTHS[1];
        }
        else
        {
            memcpy(&peak_locs_mid_scan[0], &ar1_peak_locs_top_mid_bot[num_ar1_peaks_per_scan], sizeof(double)*num_ar1_peaks_per_scan);
        }
        expected_peak_wavelengths[i++] = EXPECTED_PEAK_WAVELENGTHS[2];
        expected_peak_wavelengths[i++] = EXPECTED_PEAK_WAVELENGTHS[3];
        expected_peak_wavelengths[i++] = EXPECTED_PEAK_WAVELENGTHS[4];
        expected_peak_wavelengths[i++] = EXPECTED_PEAK_WAVELENGTHS[5];
        expected_peak_wavelengths[i++] = EXPECTED_PEAK_WAVELENGTHS[6];;
        expected_peak_wavelengths[i++] = EXPECTED_PEAK_WAVELENGTHS[7];;
        expected_peak_wavelengths[i++] = EXPECTED_PEAK_WAVELENGTHS[8];;
        /*Generate Pixel to Wavelength relationship*/
        err_result = dlpspec_calib_genPxToPyCoeffs(num_total_peaks, peak_locs_mid_scan, expected_peak_wavelengths, calib_coeffs.PixelToWavelengthCoeffs, &rSquared);

        if( (0 > err_result) || rSquared < r2_tolerance)
        {
            reply = QMessageBox::question(this, "Wavelength calibration", "dlpspec_calib_genPxToPyCoeffs failed - Retry?",
                                          QMessageBox::Yes|QMessageBox::No);

            if (reply == QMessageBox::No)
            {
                return FAIL;
            }
            else
            {
                continue;
            }
        }
        else
        {
            break;
        }
    }while(TRUE);   //Find x-y relationship - end block
    free(peak_locs_mid_scan);

    /*Compute bending coefficients */
    for(i=0; i<SCAN_RANGE; i++)
    {
        memcpy(&peak_locs_for_curve[i*NUM_PEAKS_BENDING], &ar1_peak_locs_top_mid_bot[i*num_ar1_peaks_per_scan], sizeof(double)*NUM_PEAKS_BENDING);
    }
    y_for_curve[0] = DMD_TOP_SCAN_CENTRE_Y;
    y_for_curve[1] = DMD_MID_SCAN_CENTRE_Y;
    y_for_curve[2] = DMD_BOT_SCAN_CENTRE_Y;

    if( 0 > dlpspec_calib_genPxyToCurveCoeffs(peak_locs_for_curve, y_for_curve, 4 , 3 , calib_coeffs.ShiftVectorCoeffs))
    {
        showError("dlpspec_calib_genPxToDMDColCoeffs failed");
        return FAIL;
    }

    dlpspec_util_columnToNm(0, calib_coeffs.PixelToWavelengthCoeffs, &maxWavelength);
    dlpspec_util_columnToNm(853, calib_coeffs.PixelToWavelengthCoeffs, &minWavelength);
    if ((maxWavelength < MAX_WAVELENGTH) || (minWavelength > MIN_WAVELENGTH))
    {
        showError("Computed wavelength is not in expected range. Repeat wavelength calibration");
        return FAIL;
    }

    /*Update TIVA EEPROM with computed coefficients */
    NNO_SendCalibStruct(&calib_coeffs);

    ValidateCalResults();

    NNO_DLPCEnable(false, false);
    //Give on/off control back to the scan module
    NNO_SetScanControlsDLPCOnOff(true);

    //Restore last PGA Gain setting
    SetPGAGain(last_gain_setting);

    ui->progressbar_cal->setValue(100);
    QApplication::processEvents(); //Update the GUI

    return retval;
}

int MainWindow::ValidateCalResults(void)
{
    uScanConfig config;
    double lambda[ADC_DATA_LEN];
    double peak_indices_interp[ADC_DATA_LEN];
    int peak_indices[ADC_DATA_LEN];
    double left_halfmax_loc, right_halfmax_loc;
    int scanStatus;
    int fileSize;
    void *pData;
    int i;
    double values[ADC_DATA_LEN];
    double minWavelength=0, maxWavelength=0;

    ui->progressbar_cal->setValue(75);
    ui->label_scan_progress->setText("Validation in progress");

    config.scanCfg.scan_type = COLUMN_TYPE;
    config.scanCfg.num_patterns = 427;
    config.scanCfg.num_repeats = NUM_REPEATS_CALIBRATION;
    config.scanCfg.wavelength_start_nm = MIN_WAVELENGTH;
    config.scanCfg.wavelength_end_nm = MAX_WAVELENGTH;
#ifdef PATTERN_WIDTH_CONTROL
    config.scanCfg.width_px = ui->spinBox_fwhmWidth->value();
#else
    config.scanCfg.width_px = 5;
#endif

    evm.ApplyScanCfgtoDevice(&config);

    pData = (scanData *)malloc(SCAN_DATA_BLOB_SIZE);
    if(pData == NULL)
    {
        showError("out of memory\n");
        return FAIL;
    }
    scanStatus = PerformScanReadData(NNO_DONT_STORE_SCAN_IN_SD, NUM_REPEATS_CALIBRATION, pData, &fileSize);

    QApplication::processEvents(); //Update the GUI
    if(scanStatus == PASS)
    {
        if(spectrum.SetData(pData, NULL) != PASS)
        {
            showError("Interpreting scan data failed");
            scanStatus = FAIL;
        }
    }
    else
    {
        showError("Scan failed");;
    }
    free(pData);
    if(scanStatus != PASS)
        return FAIL;

    QVector<double> samples = spectrum.GetIntensities();
    QVector<double> wavelengths = spectrum.GetWavelengths();

    for(i=0; i<samples.length(); i++)
    {
        values[i] = (double)samples[i];
        lambda[i] = (double)wavelengths[i];
    }

    if(FindNPeaks(values, samples.length(), 8, peak_indices) != 8)
    {
        showError("Unable to find expected number of peaks");
        return FAIL;
    }

    /* Calculate AR-1 peak errors */
    double ar1_peak_locations[7] = {917.2, 967.8, 1048.8, 1245.6, 1298.6, 1353.8, 1373.7};
    double peak_value_interp[8];
    double peak_deriv_interp[8];

    for(i = 0; i < 8; i++)
    {
        int j = peak_indices[i];
        if(dlpspec_calib_findPeakInterp(lambda[j-1], lambda[j], lambda[j+1], values[j-1], values[j], values[j+1], &peak_indices_interp[i], &peak_value_interp[i], &peak_deriv_interp[i]) != DLPSPEC_PASS)
        {
            showError("dlpspec_calib_findPeakInterp failed");
            return FAIL;
        }
    }

    double peak_indices_used_interp[7] = {
        peak_indices_interp[0], \
        peak_indices_interp[1], \
        peak_indices_interp[2], \
        peak_indices_interp[3], \
        peak_indices_interp[4], \
        peak_indices_interp[6], \
        peak_indices_interp[7]};

    double peak_indices_error[7];
    double peak_indices_mean_error = 0;

    for(i=0; i<7; i++)
    {
        peak_indices_error[i] = peak_indices_used_interp[i] - ar1_peak_locations[i];
        peak_indices_mean_error += fabs(peak_indices_error[i]);
    }
    peak_indices_mean_error /= 7;

    QString labelText;
    labelText.sprintf("Average wavelength error: %f nm \n %f nm error: %f nm, %f nm error: %f nm \n %f error: %f nm, %f nm error: %f nm \n %f nm error: %f nm, %f nm error: %f nm \n %f nm error: %f nm\n ",\
                      peak_indices_mean_error, \
                      ar1_peak_locations[0], peak_indices_used_interp[0], \
            ar1_peak_locations[1], peak_indices_used_interp[1], \
            ar1_peak_locations[2], peak_indices_used_interp[2], \
            ar1_peak_locations[3], peak_indices_used_interp[3], \
            ar1_peak_locations[4], peak_indices_used_interp[4], \
            ar1_peak_locations[5], peak_indices_used_interp[5], \
            ar1_peak_locations[6], peak_indices_used_interp[6]);
    ui->label_cal->setText(labelText);

    {
        FilePath mypath;
        QFile file(mypath.GetspecAr1FileName());

        file.open(QIODevice::WriteOnly | QIODevice::Text);
        QTextStream out(&file);
        out << "Known Location, Measured Location, Error" << "\n";
        for(i=0; i<7; i++)
        {
            out << ar1_peak_locations[i] << "," << peak_indices_used_interp[i] << "," << peak_indices_error[i] << "\n";
        }
        file.close();
    }

    if(peak_indices_mean_error > WAVELENGTH_ERROR_LIMIT)
    {
        showError("Wavelength location error for AR-1 wavelengths detected.");
        return FAIL;
    }

    /* Calculate FWHM */
    if(dlpspec_calib_get_halfmax_loc(values, samples.length(), peak_indices[7], &left_halfmax_loc, &right_halfmax_loc) != DLPSPEC_PASS)
    {
        showError("dlpspect_calib_get_halfmax failed");
        return FAIL;
    }
    double fwhm1 = GetInterpolatedWavelength(right_halfmax_loc) - GetInterpolatedWavelength(left_halfmax_loc);
    if(dlpspec_calib_get_halfmax_loc(values, samples.length(), peak_indices[6], &left_halfmax_loc, &right_halfmax_loc) != DLPSPEC_PASS)
    {
        showError("dlpspect_calib_get_halfmax failed");
        return FAIL;
    }
    double fwhm2 = GetInterpolatedWavelength(right_halfmax_loc) - GetInterpolatedWavelength(left_halfmax_loc);
    if(dlpspec_calib_get_halfmax_loc(values, samples.length(), peak_indices[1], &left_halfmax_loc, &right_halfmax_loc) != DLPSPEC_PASS)
    {
        showError("dlpspect_calib_get_halfmax failed");
        return FAIL;
    }
    double fwhm3 = GetInterpolatedWavelength(right_halfmax_loc) - GetInterpolatedWavelength(left_halfmax_loc);

    double nm1 = spectrum.GetWavelengths()[peak_indices[7]];
    double nm2 = spectrum.GetWavelengths()[peak_indices[6]];
    double nm3 = spectrum.GetWavelengths()[peak_indices[1]];

    QString labelTextExtra;
    dlpspec_util_columnToNm(0, calib_coeffs.PixelToWavelengthCoeffs, &maxWavelength);
    dlpspec_util_columnToNm(853, calib_coeffs.PixelToWavelengthCoeffs, &minWavelength);

    labelText.append(labelTextExtra.sprintf(" Wavelength at column 853 =  %f \n Wavelngth at column 0 = %f \n FWHM at  %f nm = %f \n FWHM at %f nm = %f \n FWHM at %f nm = %f",\
                                            minWavelength, maxWavelength, nm1, \
                                            fwhm1, nm2, fwhm2, nm3, fwhm3));
    ui->label_cal->setText(labelText);

    if((fwhm1 > FWHM_PASS_MAX_THRESHOLD) || (fwhm2 > FWHM_PASS_MAX_THRESHOLD) ||(fwhm3 > FWHM_PASS_MAX_THRESHOLD)
            || (fwhm1 < FWHM_PASS_MIN_THRESHOLD) || (fwhm2 < FWHM_PASS_MIN_THRESHOLD) ||(fwhm3 < FWHM_PASS_MIN_THRESHOLD))
    {
        showError("Some of the FWHM values outside of expected range");
        return FAIL;
    }

    return PASS;
}

int MainWindow::DoWavelengthVerification()
/**
 * This function does the wavlength verification for the system
 * Calculates the peak locations of the scanned NIST SRM 2036 sample
 */
{
    double absor[ADC_DATA_LEN];
    double lambda[ADC_DATA_LEN];
    int i = 0;
    QMessageBox::StandardButton reply;
    int retval = PASS;
    void *pData;
    int scanStatus;
    int fileSize;
    int peak_indices[ADC_DATA_LEN];
    double peak_indices_interp[ADC_DATA_LEN];

    reply = QMessageBox::question(this, "Wavelength calibration", "Is NIST SRM 2036 calibration reflectance source held to the sampling window?",
        QMessageBox::Yes|QMessageBox::No);

    if (reply == QMessageBox::No)
    {
        return FAIL;
    }


    ui->progressbar_cal->setValue(50);
    ui->label_scan_progress->setText("Full DMD scan in progress");
    uScanConfig config;
    config.scanCfg.scan_type = COLUMN_TYPE;
    config.scanCfg.num_patterns = 500;
    config.scanCfg.num_repeats = 6;
    config.scanCfg.wavelength_start_nm = MIN_WAVELENGTH;
    config.scanCfg.wavelength_end_nm = MAX_WAVELENGTH;
    #ifdef PATTERN_WIDTH_CONTROL
        config.scanCfg.width_px = ui->spinBox_fwhmWidth->value();
    #else
        config.scanCfg.width_px = 5;
    #endif

    evm.ApplyScanCfgtoDevice(&config);

    pData = (scanData *)malloc(SCAN_DATA_BLOB_SIZE);
    if(pData == NULL)
    {
        showError("out of memory\n");
        return FAIL;
    }
    scanStatus = PerformScanReadData(NNO_DONT_STORE_SCAN_IN_SD, NUM_REPEATS_CALIBRATION, pData, &fileSize);

    QApplication::processEvents(); //Update the GUI
    if(scanStatus == PASS)
    {
        if(spectrum.SetData(pData, evm.GetRefCalDataBlob()) != PASS)
        {
            showError("Interpreting scan data failed. Reference calibration must be performed first.");
            scanStatus = FAIL;
        }
        //dumping the verification data
        QString filename = "wavlencalib_verification";
        QString timestamp = spectrum.GetScanTimeStamp();
        filename.append(timestamp);
        filename.append(".csv");
        spectrum.SaveCombinedCSV(filename);
    }
    else
    {
      showError("Scan failed");
    }
    free(pData);
    if(scanStatus != PASS)
        return FAIL;

    QVector<double> absorbance = spectrum.GetAbsorbance();
    QVector<double> wavelengths = spectrum.GetWavelengths();

    for(i=0; i<absorbance.length(); i++)
    {
        absor[i] = -1 * pow(10, -1 * (double)absorbance[i]); // reflectance inverted for peak finding
        lambda[i] = (double)wavelengths[i];
    }

    /* This is the third step of Finding peaks . Peak Indices are gathered in peak_indices function*/
    if(FindNPeaks(absor, absorbance.length(), 6, peak_indices) != 6)
    {
        showError("Unable to find expected number of peaks");
        retval = FAIL;
    }

    if(retval == PASS)
    {
        /* Calculate NIST SRM 2036 peak errors */
        double srm2036_peak_locations[6] = {975.9, 1075.8, 1151.0, 1222.1, 1367.3, 1469.5};
        double peak_value_interp[6];
        double peak_deriv_interp[6];

        for(i = 0; i < 6; i++)
        {
            int j = peak_indices[i];
            if(dlpspec_calib_findPeakInterp(lambda[j-1], lambda[j], lambda[j+1], absor[j-1], absor[j], absor[j+1], &peak_indices_interp[i], &peak_value_interp[i], &peak_deriv_interp[i]) != DLPSPEC_PASS)
            {
                showError("dlpspec_calib_findPeakInterp failed");
                return FAIL;
            }
        }

        double peak_indices_used_interp[6] = {
            peak_indices_interp[0], \
            peak_indices_interp[1], \
            peak_indices_interp[2], \
            peak_indices_interp[3], \
            peak_indices_interp[4], \
            peak_indices_interp[5]};

        double peak_indices_error[6];
        double peak_indices_mean_error = 0;

        for(i=0; i<6; i++)
        {
            peak_indices_error[i] = peak_indices_used_interp[i] - srm2036_peak_locations[i];
            peak_indices_mean_error += fabs(peak_indices_error[i]);
        }
        peak_indices_mean_error /= 6;

        QString labelText;
        labelText.sprintf("Average wavelength error: %f nm \n \
                         %f nm error: %f nm \n %f nm error: %f nm \n %f error: %f nm \n \
                         %f nm error: %f nm \n %f nm error: %f nm \n %f error: %f nm",\
            peak_indices_mean_error, \
            srm2036_peak_locations[0], peak_indices_error[0], \
            srm2036_peak_locations[1], peak_indices_error[1], \
            srm2036_peak_locations[2], peak_indices_error[2], \
            srm2036_peak_locations[3], peak_indices_error[3], \
            srm2036_peak_locations[4], peak_indices_error[4], \
            srm2036_peak_locations[5], peak_indices_error[5]);
        ui->label_cal->setText(labelText);

        
        FilePath mypath;
        QFile file(mypath.GetspecSrm2036FileName());

        file.open(QIODevice::WriteOnly | QIODevice::Text);
        QTextStream out(&file);
        out << "Known Location, Measured Location, Error, Allowed Error" << "\n";
        for(i=0; i<6; i++)
        {
            out << srm2036_peak_locations[i] << "," << peak_indices_used_interp[i] << "," << peak_indices_error[i] << "," << sqrt(pow(WAVELENGTH_ERROR_LIMIT_ARR[i], 2) + pow(WAVELENGTH_ACCURACY_LIMIT, 2)) << "\n";
        }
		file.close();

        bool error_detected = false;
        for(i=0; i<6; i++)
        {
            if(fabs(peak_indices_error[i]) > sqrt(pow(WAVELENGTH_ERROR_LIMIT_ARR[i], 2) + pow(WAVELENGTH_ACCURACY_LIMIT, 2)))
            {
                out <<"maximum error limit for wavelength" << srm2036_peak_locations[i] << "is =" << sqrt(pow(WAVELENGTH_ERROR_LIMIT_ARR[i], 2) + pow(WAVELENGTH_ACCURACY_LIMIT, 2)) << "detected = " << peak_indices_error[i] << "\n";

                error_detected = true;

            }

        }
        if(error_detected)
          {
            showError("Wavelength location error for NIST SRM-2036 wavelengths detected.");
            return FAIL;
         }
    }

    ui->progressbar_cal->setValue(100);
    QApplication::processEvents(); //Update the GUI

    return retval;
}

int MainWindow::PerformRefCalScan(void *pData)
/**
 * This function does the wavlength calibration for the system
 * Calculates the peak locations ,the peak values & calibration co-efficients
 * Sends the calibration co-efficients to TIVA
 * These values are later svaed to a file on PC
 */
{
    int scanStatus;
    uScanConfig config;
    int fileSize;

    if(NNO_SetPGAGain(64) != PASS)
        return FAIL;

    ui->progressbar_cal->setValue(0);
    ui->label_scan_progress->setText("Scan in progress");
    QApplication::processEvents(); //Update the GUI

    config.scanCfg.scan_type = COLUMN_TYPE;
    config.scanCfg.num_patterns = 228;
    config.scanCfg.num_repeats = 30;
    config.scanCfg.wavelength_start_nm = MIN_WAVELENGTH;
    config.scanCfg.wavelength_end_nm = MAX_WAVELENGTH;
    config.scanCfg.width_px = 6;
    strcpy(config.scanCfg.config_name, "SystemTest");

    evm.ApplyScanCfgtoDevice(&config);
    ui->progressbar_cal->setValue(25);

    scanStatus = PerformScanReadData(NNO_DONT_STORE_SCAN_IN_SD, 30, pData, &fileSize);

    if(scanStatus != PASS)
    {
        showError("Scan failed");
        return FAIL;
	}
    ui->progressbar_cal->setValue(50);

    return PASS;
}

int MainWindow::DoReferenceCalibration(void)
{
    void *pData = malloc(SCAN_DATA_BLOB_SIZE);

    if(pData == NULL)
    {
        showError("Out of Memory");
        return FAIL;
    }
    PerformRefCalScan(pData);

    NNO_SaveRefCalPerformed();
    /* Set the reference */
    spectrum.SetData(pData, pData);
    spectrum.SaveReferenceToFile();

    free(pData);

    //Reapply the last selected scan config
    on_comboBox_scanID_currentIndexChanged(ui->comboBox_scanID->currentIndex());

    return PASS;
}

int MainWindow::DoSystemTest(bool *pSimilar, double *pSimVal)
/**
 *
 * This is for the System test Calibration.
 * @param pSimilar - O - boolean to indicate if the absorption spectrum is similar or not
 * @param pSimVal  - O - calculated cosine similarity number
 *
 */
{
    double sampleAbsorbance[228];
    int i;
    void *pData = malloc(SCAN_DATA_BLOB_SIZE);

    if(pData == NULL)
    {
        showError("Out of Memory");
        return FAIL;
    }
    evm.FetchRefCalData();
    PerformRefCalScan(pData);
    spectrum.SetData(pData, evm.GetRefCalDataBlob());

    free(pData);

    //refCal must have been performed as the previous step and set as reference for absorption computation
    //check if that's indeed the case
    QVector<double> absorbance = spectrum.GetAbsorbance();
    for(i=0; i < absorbance.size(); i++)
    {
        sampleAbsorbance[i] = absorbance[i];
    }
    //Compute absorbance and compare with known good absorbance spectrum of the test sample
    compare_absorption_spectrum(sampleAbsorbance, knownTestSampleAbsorbance, 228, 0.99, pSimilar, pSimVal);

    ui->progressbar_cal->setValue(100);
    QApplication::processEvents(); //Update the GUI

    return PASS;
}

void MainWindow::on_pushButton_startDetCal_clicked()
    /**
	 * This function is handler for pushButton_startDetCal on Factory tab clicked() event
	 * Starts Detector Alignment Calibration
	 * Increments the three sliders during the process
	 * Checks to see if the Stop button is pressed and then saves the calibrated values when stopped
	 */
{
	int max1, max2,max3;
	int index1=0, index2=0, index3=0;
	scanData *pData;
    int scanStatus;
	int fileSize;
	scanResults scan_results;

	detCalStop = false;
	max1 = max2 = max3 = 10;
	SetDetAlignSliderMaxVal(1000000);

	/* Set the minimum acceptable values */
	ui->verticalSlider_det_align_Min1->setValue(74000);
	ui->lineEdit_Det_Min1->setText(QString::number(74000));
	ui->verticalSlider_det_align_Min2->setValue(281000);
	ui->lineEdit_Det_Min2->setText(QString::number(281000));
	ui->verticalSlider_det_align_Min3->setValue(120000);
	ui->lineEdit_Det_Min3->setText(QString::number(120000));

	//Instead of DLPC and Lamp ON/OFF being controlled for each scan, we will control it for a set of scans
	NNO_SetScanControlsDLPCOnOff(false);
#ifdef USE_CALIB_PATTERNS_FROM_FLASH
    NNO_DLPCEnable(true, true);
#else
    NNO_DLPCEnable(true, true);
	NNO_GenCalibPatterns(DET_ALIGN_SCAN);
#endif

    NNO_SetPGAGain(1);

    pData = (scanData *)malloc(SCAN_DATA_BLOB_SIZE);
    if(pData == NULL)
    {
        showError("out of memory\n");
        detCalStop = true; //so that we skip the while loop below.
    }

    while(detCalStop == false)
    {
        ui->label_detAlignStatus->setText("Waiting for scan completion");
		QApplication::processEvents(); //Update the GUI

        scanStatus = PerformScanReadData(NNO_DONT_STORE_SCAN_IN_SD, 1, pData, &fileSize);
        if(scanStatus == PASS)
		{
            if(dlpspec_calib_interpret(pData, fileSize, &scan_results, DET_ALIGN_SCAN) != DLPSPEC_PASS)
            {
                showError("Scan interpret failed");
                break;
            }
		}
		else
		{
            showError("Error reading scan data");
			break;
		}

        ui->label_detAlignStatus->setText("Scan Complete");
		QApplication::processEvents(); //Update the GUI

		index1 = (int)scan_results.intensity[0];
		index2 = (int)scan_results.intensity[1];
		index3 = (int)scan_results.intensity[2];

		//Ignore data if it is not as expected to avoid any spurious scan results from being displayed
		if(index1 > index2)
		{
			continue;
		}
		ui->verticalSlider_det_align_1->setValue(index1);
		ui->verticalSlider_det_align_2->setValue(index2);
		ui->verticalSlider_det_align_3->setValue(index3);
		ui->lineEdit_Det_Cur1->setText(QString::number(index1));
		ui->lineEdit_Det_Cur2->setText(QString::number(index2));
		ui->lineEdit_Det_Cur3->setText(QString::number(index3));

		if(index1 > max1)
		{
			max1 = index1;
			ui->verticalSlider_det_align_Max1->setValue(max1);
		}

		if(index2 > max2)
		{
			max2 = index2;
			ui->verticalSlider_det_align_Max2->setValue(max2);
		}

		if(index3 > max3)
		{
			max3 = index3;
			ui->verticalSlider_det_align_Max3->setValue(max3);
		}

		QApplication::processEvents(); //Update the GUI
	}

    if(pData != NULL)
        free(pData);
	//If we are controlling DLPC and Lamp ON/OFF, turn it off now.
    NNO_DLPCEnable(false, false);
	//Give on/off control back to the scan module
	NNO_SetScanControlsDLPCOnOff(true);
}

void MainWindow::on_pushButton_stopDet_cal_clicked()
    /**
	 * This function is handler for pushButton_stopDetCal on Factory tab clicked() event
	 * Stops Detector Alignment Calibration
	 * sets the detCalStop to true so that the above function stops the process
	 * saves the calibrated values to Detector_Align_Values_*.txt file in the pC
	 */

{
	detCalStop = true;
	ui->label_detAlignStatus->setText("Stopped");

	char sernum[NANO_SER_NUM_LEN+1];

	sernum[NANO_SER_NUM_LEN] = '\0';
	QString serialNumber;

	if(NNO_GetSerialNumber(sernum) == PASS)
	{
		serialNumber =  QString::fromLocal8Bit(sernum);
	}
	else
		serialNumber = "";

	QString cur = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
	QString detAlignFileName = QString("Detector_Align_Values_%1_%2.txt").arg(serialNumber,cur);

	QString fileName = filepath.GetsaveScanSettingsDir().absoluteFilePath(detAlignFileName);

	QFile DetResultsFile(fileName);
	DetResultsFile.open(QIODevice::ReadWrite | QIODevice::Text);
	QTextStream out(&DetResultsFile);

	out << "Detector Alignment Values\n";

	out << ui->verticalSlider_det_align_1->value() << "\n";
	out << ui->verticalSlider_det_align_2->value() << "\n";
	out << ui->verticalSlider_det_align_3->value() << "\n";

	DetResultsFile.close();
}

void MainWindow::on_pushButton_startSlit_Cal_clicked()
/**
     * This function is handler for pushButton_startSlitCal on Factory tab clicked() event
     * Starts Slit Alignment Calibration
     * Increments the three sliders during the process
     * Checks to see if the Stop button is pressed and then saves the calibrated values when stopped
     */
{
    int max1=10;
    int max2=10;
    int max3=10;
    double values[ADC_DATA_LEN];
    int peak_indices[ADC_DATA_LEN];
    int i=0;
    double index1=0, index2=0, index3=0;
    int peak_height1=0;
    int peak_height2=0;
    int peak_height3=0;
    scanData *pData;
    int fileSize;
    int last_gain_setting;
    scanResults scan_results;
    int scanStatus;

    slitCalStop = false;

    //Set PGA gain to 64
    last_gain_setting = SetPGAGain(64);

    ui->verticalSlider_slit_align_Min1->setValue(4200);
    ui->lineEdit_Slit_Min1->setText(QString::number(4200));
    ui->verticalSlider_slit_align_Min2->setValue(7500);
    ui->lineEdit_Slit_Min2->setText(QString::number(7500));
    ui->verticalSlider_slit_align_Min3->setValue(8500);
    ui->lineEdit_Slit_Min3->setText(QString::number(8500));

    //Instead of DLPC and Lamp ON/OFF being controlled for each scan, we will control it for a set of scans
    NNO_SetScanControlsDLPCOnOff(false);
    NNO_DLPCEnable(true, false);
    NNO_GenCalibPatterns(SLIT_ALIGN_SCAN);

    pData = (scanData *)malloc(SCAN_DATA_BLOB_SIZE);
    if(pData == NULL)
    {
        showError("out of memory\n");
        slitCalStop = true; //so that we skip the while loop below.
    }

    while(slitCalStop == false)
    {
        ui->label_slitAlignStatus->setText("Waiting for scan completion");
        QApplication::processEvents(); //Update the GUI

        scanStatus = PerformScanReadData(NNO_DONT_STORE_SCAN_IN_SD, SLIT_ALIGN_NUM_SCANS_AVG, pData, &fileSize);
        if(scanStatus == PASS)
        {
            if(dlpspec_calib_interpret(pData, fileSize, &scan_results, SLIT_ALIGN_SCAN) != DLPSPEC_PASS)
            {
                showError("Scan interpret failed");
                break;
            }
        }
        else
        {
            showError("Error reading scan data");
            break;
        }

        for(i=0; i<scan_results.length; i++)
            values[i] = (double)scan_results.intensity[i];

        ui->label_slitAlignStatus->setText("Scan complete");
        QApplication::processEvents(); //Update the GUI

        if(FindNPeaks(values, scan_results.length, 3, peak_indices) != 3)
        {
            showError("Unable to find expected number of peaks");
            continue;
        }
        if(dlpspec_calib_getFWHM(values, scan_results.length, peak_indices[0], &peak_height1, &index1) != DLPSPEC_PASS)
        {
            showError("Find FWHM failed");
            continue;
        }
        if(dlpspec_calib_getFWHM(values, scan_results.length, peak_indices[1], &peak_height2, &index2) != DLPSPEC_PASS)
        {
            showError("Find FWHM failed");
            continue;
        }
        if(dlpspec_calib_getFWHM(values, scan_results.length, peak_indices[2], &peak_height3, &index3) != DLPSPEC_PASS)
        {
            showError("Find FWHM failed");
            continue;
        }

        /* In order to provide user feedback in a similar fashion to detector alignment interface,
             * display peak height divided by fwhm as slider position
             */

        if(index1)  //avoid division by zero
            index1 = peak_height1/index1;
        if(index2)
            index2 = peak_height2/index2;
        if(index3)
            index3 = peak_height3/index3;

        ui->verticalSlider_slit_align_1->setValue(index1);
        ui->verticalSlider_slit_align_2->setValue(index2);
        ui->verticalSlider_slit_align_3->setValue(index3);
        ui->lineEdit_Slit_Cur1->setText(QString::number(index1));
        ui->lineEdit_Slit_Cur2->setText(QString::number(index2));
        ui->lineEdit_Slit_Cur3->setText(QString::number(index3));

        if(index1 > max1)
        {
            max1 = index1;
            ui->verticalSlider_slit_align_Max1->setValue(max1);
        }
        if(index2 > max2)
        {
            max2 = index2;
            ui->verticalSlider_slit_align_Max2->setValue(max2);
        }
        if(index3 > max3)
        {
            max3 = index3;
            ui->verticalSlider_slit_align_Max3->setValue(max3);
        }
        QApplication::processEvents(); //Update the GUI
    }

    if(pData != NULL)
        free(pData);
    //If we are controlling DLPC and Lamp ON/OFF, turn it off now.
    NNO_DLPCEnable(false, false);
    //Give on/off control back to the scan module
    NNO_SetScanControlsDLPCOnOff(true);

    //Restore last PGA Gain setting
    SetPGAGain(last_gain_setting);

}

void MainWindow::on_pushButton_stopSlit_Cal_clicked()
/**
 * This function is handler for pushButton_stopSlitCal on Factory tab clicked() event
 * Stops Slit Alignment Calibration
 * sets the slitCalStop to true so that the above function stops the process
 * saves the calibrated values to Slit_Align_Values*.txt file in the pC
 */
{
	slitCalStop = true;
	ui->label_slitAlignStatus->setText("Stopped");

	char sernum[NANO_SER_NUM_LEN+1];

	sernum[NANO_SER_NUM_LEN] = '\0';
	QString serialNumber;

	if(NNO_GetSerialNumber(sernum) == PASS)
	{
		serialNumber =  QString::fromLocal8Bit(sernum);
	}
	else
		serialNumber = "";

	QString cur = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
	QString slitAlignFileName = QString("Slit_Align_Values_%1_%2.txt").arg(serialNumber,cur);

	QString fileName = filepath.GetsaveScanSettingsDir().absoluteFilePath(slitAlignFileName);

	QFile SlitResultsFile(fileName);
	SlitResultsFile.open(QIODevice::ReadWrite | QIODevice::Text);
	QTextStream out(&SlitResultsFile);

	out << "Slit Alignment Values\n";
	out << ui->verticalSlider_slit_align_1->value() << "\n";
	out << ui->verticalSlider_slit_align_2->value() << "\n";
	out << ui->verticalSlider_slit_align_3->value() << "\n";

	SlitResultsFile.close();
}

void MainWindow::on_pushButton_reg_set_clicked()
    /**
	 * This function is handler for pushButton_reg_set on Factory tab clicked() event
	 * Sets entered values form lineEdit_reg_addr and lineEdit_reg_val in TIVA
	 */
{
	if ( ui->lineEdit_reg_addr->text().isEmpty() || ui->lineEdit_reg_val->text().isEmpty())
		return;

	NNO_SetDLPCReg(ui->lineEdit_reg_addr->text().toUInt(), ui->lineEdit_reg_val->text().toUInt());

}

void MainWindow::on_pushButton_reg_get_clicked()
    /**
	 * This function is handler for pushButton_reg_get on Factory tab clicked() event
	 * Gets the values from TIVA and populates in lineEdit_reg_addr and lineEdit_reg_val
	 *
	 */
{
	uint32 value;

	if (ui->lineEdit_reg_addr->text().isEmpty())
		return;

	NNO_GetDLPCReg(ui->lineEdit_reg_addr->text().toUInt(), &value);
	ui->lineEdit_reg_val->setText(QString::number(value));
}

void MainWindow::on_pushButton_CalStart_clicked()
    /**
	 * This function is handler for pushButton_CalStart on Factory tab clicked() event
	 * Based on the Calibration Type that is slecetd, it populates the repsective GUI controls
	 * Calls the repsective calibration procedures
	 *
	 *
	 */
{
	int ret=0;
	QMessageBox::StandardButton reply;
	double simVal;
    bool isSimilar;
	QString labelText;

	ui->progressbar_cal->setValue(0);
	ui->label_Cal_Steps->setText("");
	ui->label_cal->setText("");
	// ui->label_scan_progress("");
    ui->label_scan_progress->setText("In progress");

	if(ui->comboBox_calID->currentIndex() == CAL_AR_SOURCE)
	{
		ret = DoWavelengthCalibration();
	}
    else if(ui->comboBox_calID->currentIndex() == CAL_VERIFY)
    {
        ret = DoWavelengthVerification();
    }
    else if(ui->comboBox_calID->currentIndex() == CAL_REF_SAMPLE)
    {
        ret = DoReferenceCalibration();
	}
    else if(ui->comboBox_calID->currentIndex() ==CAL_SYS_CHECK )
	{
		reply = QMessageBox::question(this, "System Test", "Is reflective piece removed and test sample held up to the unit? Then click Yes",
				QMessageBox::Yes|QMessageBox::No);

		if (reply == QMessageBox::No)
		{
			return;
		}

        ret = DoSystemTest(&isSimilar, &simVal);

        labelText.sprintf("Cosine similarity with golden unit = %1.5f",\
                simVal);
        ui->label_cal->setText(labelText);
        if(isSimilar)
            ret = PASS;
        else
            ret = FAIL;
    }
    else if(ui->comboBox_calID->currentIndex() == CAL_SNR_COMPUTE )
	{
        ret = DoSNRComputation( 0 );
	}    

    ui->progressbar_cal->setValue(100);
    if(ret != PASS)
    {
        ui->label_scan_progress->setText("Failed");
    }
    else
    {
        ui->label_scan_progress->setText("Pass");
    }
}


void MainWindow::on_comboBox_calID_currentIndexChanged(int index)
    /**
	 * This function is handler for comboBox_calID on Factory tab currentIndexChanged() event
	 * Based on the Calibration Type that is slecetd, it populates the repsective GUI controls
	 * @param index - I - index of the current selected scalibration type
	 *
	 */
{
	if(index == CAL_AR_SOURCE)
	{
		//set the picture
		QString filename = ":/new/Icons/wavecal_new.png";
		QImage image(filename);
		ui->label_cal->setPixmap(QPixmap::fromImage(image));
		ui->label_cal->setScaledContents( true );
		ui->label_cal->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );

		//set the calibration steps
		ui->label_Cal_Steps->setText("1.Install the linear end of the fiber with mark into the fiber input module,\naligning the mark on the fiber housing to the groove in the illumination module\n2.Install the round end of the fiber without a mark into the spectral calibration source\n3.Power on the spectral calibration source\n4.Connect a USB cable from the TIVA board to a Windows PC \n5.Enter the calibration tab and select ‘Wavelength Calibration\n6.Follow the prompts on the screen until calibration is complete");
	}
    if(index == CAL_VERIFY)
    {
        //set the picture
        QString filename = ":/new/Icons/NistSrm2036.png";
        QImage image(filename);
        ui->label_cal->setPixmap(QPixmap::fromImage(image));
        ui->label_cal->setScaledContents( true );
        ui->label_cal->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );

        //set the calibration steps
        ui->label_Cal_Steps->setText("1.Hold sample against sampling window\n2.Start scan");
    }
    else if(index == CAL_REF_SAMPLE)
	{
		//set the picture
		QString filename = ":/new/Icons/refcal_new.png";
		QImage image(filename);
		ui->label_cal->setPixmap(QPixmap::fromImage(image));
		ui->label_cal->setScaledContents( true );
		ui->label_cal->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );
		//set the calibration steps
		ui->label_Cal_Steps->setText("1.Connect a USB cable from the TIVA board to a Windows PC\n2.Enter the calibration tab and select ‘Reference Calibration’\n3.Follow the software prompts to complete the reference calibration");
	}
    else if(index ==CAL_SYS_CHECK )
	{
		//set the picture
		QString filename = ":/new/Icons/refcal_new.png";
		QImage image(filename);
		ui->label_cal->setPixmap(QPixmap::fromImage(image));
		ui->label_cal->setScaledContents( true );
		ui->label_cal->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );

		//set the calibration steps
		ui->label_Cal_Steps->setText("1.Connect a USB cable from the TIVA board to a Windows PC\n2.Enter the calibration tab and select ‘System Test’\n3.Follow the software prompts to complete the scan\n4.Once all scans are complete and confirmed to be within tolerance,\n a confirmation dialog will be displayed");
	}
    else if(index == CAL_SNR_COMPUTE )
    {
        //set the picture
        QString filename = ":/new/Icons/refcal_new.png";
        QImage image(filename);
        ui->label_cal->setPixmap(QPixmap::fromImage(image));
        ui->label_cal->setScaledContents( true );
        ui->label_cal->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );
    }
}

int MainWindow::on_pushButton_eeprom_write_generic_clicked()
    /**
	 *This function is a handler for eeprom_write_generic on factory tab, clicked() event
	 * Writes the EEPROM with generic calibration coefficients
	 * Used especially when the unit is not calibrated
	 */
{
	int result;
	QMessageBox::StandardButton reply;

	reply = QMessageBox::question(this, "EEPROM Calibration Write", "Overwrite the EEPROM with generic calibration coefficients?",
			QMessageBox::Yes|QMessageBox::No);
	if (reply == QMessageBox::Yes) {
		result = NNO_EEPROM_CalTest();
		if ( result == FAIL ) return result;
		showError("EEPROM Calibration Coefficients Overwritten");
	}
	result = readVersionAndUpdate();
	return result;
}

int MainWindow::on_pushButton_eeprom_read_clicked()
    /**
	 *This function is a handler for eeprom_read on factory tab, clicked() event
	 * Reads the EEPROM calibration coefficients
	 * Used especially when the unit is not calibrated
	 */
{
	int result;
	calibCoeffs calib_coeffs;

	result = NNO_GetCalibStruct(&calib_coeffs);
    if ( result == FAIL ) {
        showError("Failed to Read EEPROM Calibration Coefficients");
        return result;
    }
	
    ui->lineEdit_shiftVect0->setText(QString::number((double) calib_coeffs.ShiftVectorCoeffs[0]));
    ui->lineEdit_shiftVect1->setText(QString::number((double) calib_coeffs.ShiftVectorCoeffs[1]));
    ui->lineEdit_shiftVect2->setText(QString::number((double) calib_coeffs.ShiftVectorCoeffs[2]));
    ui->lineEdit_pix2wave0->setText(QString::number((double) calib_coeffs.PixelToWavelengthCoeffs[0]));
    ui->lineEdit_pix2wave1->setText(QString::number((double) calib_coeffs.PixelToWavelengthCoeffs[1]));
    ui->lineEdit_pix2wave2->setText(QString::number((double) calib_coeffs.PixelToWavelengthCoeffs[2]));
	return result;
}

int MainWindow::on_pushButton_eeprom_write_clicked()
    /**
	 *This function is a handler for eeprom_write on factory tab, clicked() event
	 * Writes the EEPROM with the inputed calibration coefficients values
	 * Used especially when the unit is not calibrated
	 */
{
    int result = PASS;
	calibCoeffs calib_coeffs;
	QMessageBox::StandardButton reply;

	reply = QMessageBox::question(this, "EEPROM Calibration Coefficient Write", "Overwrite the EEPROM Pixel to Wavelength Calibration coefficients?",
			QMessageBox::Yes|QMessageBox::No);
		
	if (reply == QMessageBox::Yes) {
		if ( ui->lineEdit_pix2wave0->text().isEmpty() || ui->lineEdit_pix2wave1->text().isEmpty() || ui->lineEdit_pix2wave2->text().isEmpty() ||
		     ui->lineEdit_shiftVect0->text().isEmpty() || ui->lineEdit_shiftVect1->text().isEmpty() || ui->lineEdit_shiftVect2->text().isEmpty() ) {
			showError("Please input EEPROM Pixel to Wavelength and Shift Vector Calibration Coefficients");
            return FAIL;
		}

		calib_coeffs.ShiftVectorCoeffs[0] = ui->lineEdit_shiftVect0->text().toDouble();
		calib_coeffs.ShiftVectorCoeffs[1] = ui->lineEdit_shiftVect1->text().toDouble();
		calib_coeffs.ShiftVectorCoeffs[2] = ui->lineEdit_shiftVect2->text().toDouble();
		calib_coeffs.PixelToWavelengthCoeffs[0] = ui->lineEdit_pix2wave0->text().toDouble();
		calib_coeffs.PixelToWavelengthCoeffs[1] = ui->lineEdit_pix2wave1->text().toDouble();
		calib_coeffs.PixelToWavelengthCoeffs[2] = ui->lineEdit_pix2wave2->text().toDouble();
		result = NNO_SendCalibStruct(&calib_coeffs);
        if ( result == FAIL )
            showError("EEPROM Pixel to Wavelength Calibration Coefficients Overwritten");
	}
	return result;
}


void MainWindow::on_pushButton_SetSerNumber_clicked()
    /**
	 *This function is a handler for pushButton_GetSerNumber on Factory tab, clicked() event
	 *validates and sets the value entered in textEdit_ser_num in the EVM
	 */
{
	char sernum[NANO_SER_NUM_LEN+1];
	QString str = ui->lineEdit_ser_num->text();

	/* Commenting off length check as flexibility is needed in the factory to input longer string; but we will take only the last 7 characters
	   if (str.length() > 7)
	   {
	   QMessageBox msgBox(QMessageBox::Warning, "Serial Number Length Error", "Please enter 7 or less characters for serial number!", QMessageBox::NoButton, this);
	   msgBox.exec();
	   ui->textEdit_ser_num->setText("");
	   return;
	   }
	   */

	str = str.right(8);
	str = str.rightJustified(8,'_');
	QRegExp regexp("[a-zA-Z0-9_-]*");
	if (regexp.indexIn(str,0) != -1)
	{
		if (regexp.matchedLength() != str.length())
		{
			QMessageBox msgBox(QMessageBox::Warning, "Serial Number Error", "Serial number can only be alpha numeric characters, please enter again in the correct format!", QMessageBox::NoButton, this);
			msgBox.exec();
			ui->lineEdit_ser_num->setText("");
			return;
		}
	}

	QByteArray array = str.toLocal8Bit();
	char* buffer = array.data();
	for(uint32 i=0 ; i<strlen(buffer);i++)
	{
		sernum[i] = buffer[i];
	}

	QMessageBox::StandardButton reply;

	reply = QMessageBox::question(this, "Serial Number", "Do you want to erase the serial number? Please enter a 7 character serial number",
			QMessageBox::Yes|QMessageBox::No);

	if (reply == QMessageBox::Yes)
	{
		if(NNO_SetSerialNumber(sernum) >= 0)
			b_serNumSet = true;
	}

}

void MainWindow::on_pushButton_GetSerNumber_clicked()
    /**
	 *This function is a handler for pushButton_GetSerNumber on Factory tab, clicked() event
	 *Calls the corresponding API function and displays the result in respective GUI labels
	 *@return 0 = PASS
	 *         -1 = FAIL
	 */
{
	char sernum[NANO_SER_NUM_LEN+1];

	sernum[NANO_SER_NUM_LEN] = '\0';

	if(NNO_GetSerialNumber(sernum) == PASS)
	{
		ui->lineEdit_ser_num->setText(QString::fromLocal8Bit(sernum));
	}
	else
	{
		ui->lineEdit_ser_num->clear();
	}
}


void MainWindow::on_pushButton_SetModelName_clicked()
    /**
	 *This function is a handler for pushButton_GetModelName on Factory tab, clicked() event
	 *validates and sets the value entered in textEdit_ser_num in the EVM
	 */
{
	char model_name[NANO_MODEL_NAME_LEN];
	QString str = ui->lineEdit_model_name->text();

   if (str.length() > NANO_MODEL_NAME_LEN)
   {
	   QMessageBox msgBox(QMessageBox::Warning, "Model Name Length Error", "Please enter 15 or less characters for model name!", QMessageBox::NoButton, this);
	   msgBox.exec();
       ui->lineEdit_model_name->setText("");
	   return;
   }
	   

	str = str.right(NANO_MODEL_NAME_LEN);
	QRegExp regexp("[a-zA-Z0-9_-]*");
	if (regexp.indexIn(str,0) != -1)
	{
		if (regexp.matchedLength() != str.length())
		{
			QMessageBox msgBox(QMessageBox::Warning, "Model Name Error", "Model name can only be alpha numeric characters, please enter again in the correct format!", QMessageBox::NoButton, this);
			msgBox.exec();
            ui->lineEdit_model_name->setText("");
			return;
		}
	}

	QByteArray array = str.toLocal8Bit();
	char* buffer = array.data();
	for(uint32 i=0 ; i<strlen(buffer);i++)
	{
		model_name[i] = buffer[i];
	}

	QMessageBox::StandardButton reply;

	reply = QMessageBox::question(this, "Model Name", "Do you want to erase the model name? Please enter a 15 character model name",
			QMessageBox::Yes|QMessageBox::No);

	if (reply == QMessageBox::Yes)
	{
        if(NNO_SetModelName(model_name) >= 0)
			b_serNumSet = true;
	}

}

void MainWindow::on_pushButton_GetModelName_clicked()
    /**
	 *This function is a handler for pushButton_GetModelName on Factory tab, clicked() event
	 *Calls the corresponding API function and displays the result in respective GUI labels
	 *@return 0 = PASS
	 *         -1 = FAIL
	 */
{
	char model_name[NANO_MODEL_NAME_LEN];

	model_name[NANO_MODEL_NAME_LEN-1] = '\0';

	if(NNO_GetModelName(model_name) == PASS)
	{
		ui->lineEdit_model_name->setText(QString::fromLocal8Bit(model_name));
	}
	else
	{
		ui->lineEdit_ser_num->clear();
	}
}

int MainWindow::on_pushButton_eeprom_mass_erase_clicked()
    /**
	 *This function is a handler for pushButton_eeprom_mass_erase on Factory tab, clicked() event
	 *Calls the corresponding API function and displays the result in respective GUI labels
	 *@return 0 = PASS
	 *         -1 = FAIL
	 */
{
	int result;
	QMessageBox::StandardButton reply;

	reply = QMessageBox::question(this, "EEPROM Full Erase", "This erases all calibration, reference calibration, and scan configuration data stored in the EEPROM. Do you want to proceed erasing the complete EEPROM??",
			QMessageBox::Yes|QMessageBox::No);

	if(reply == QMessageBox::Yes)
	{
		result = NNO_EEPROMMass_Erase();
		if ( result != FAIL )
		{
			showError("EEPROM Erased");
			result = readVersionAndUpdate();
		}
		return result;
	}
	else
		return FAIL;
}

#define START_ADDRESS  0x40015000
#define END_ADDRESS    0x4001504C

void MainWindow::on_pushButton_dumpreg_clicked()
    /**
	 *This function is a handler for pushButton_dumpreg on factory tab, clicked() event
	 * Saves the values of the DLPC150 registers in dump_cadc_reg.txt file
	 */
{
	uint32 addr;
	uint32 value;
	FILE* dump_reg;

	dump_reg = fopen("dump_cadc_reg.txt" , "w");
	if(dump_reg != NULL)
	{
		for(addr=START_ADDRESS;addr<=END_ADDRESS;addr+=4)
		{
			NNO_GetDLPCReg(addr, &value);
			fprintf(dump_reg , "[%x] val is [%x]\n",addr , value);
		}
		fclose(dump_reg);
	}

}
