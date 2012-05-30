#include <stdio.h>
#include <windows.h>
#include "OpenCVHelpers.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/core/core_c.h"

double cvhStd(int data[], int len) {
	return sqrt(cvhVar(data, len));
}

double cvhVar(int data[], int len) {
	double f = 1.0 / (len - 1);
	int i;
	double sum = 0;
	double d = 0;
	double avg = cvhAvg(data, len);
	for (i = 0; i < len; i++) {
		d = data[i] - avg;
		sum = sum + d * d;
	}
	return sum * f;
}

double cvhAvg(int data[], int len) {
	double sum = 0;
	int i = 0;
	for (i = 0; i < len; i++)
		sum = sum + data[i];
	return sum / len;
}

double cvhStdF(double data[], int len) {
	return sqrt(cvhVarF(data, len));
}

double cvhVarF(double data[], int len) {
	double f = 1.0 / (len - 1);
	int i;
	double sum = 0;
	double d = 0;
	double avg = cvhAvgF(data, len);
	for (i = 0; i < len; i++) {
		d = data[i] - avg;
		sum = sum + d * d;
	}
	return sum * f;
}

double cvhAvgF(double data[], int len) {
	double sum = 0;
	int i = 0;
	for (i = 0; i < len; i++)
		sum = sum + data[i];
	return sum / len;
}

double cvhMagnitude(int data[], int len) {
	double sum = 0;
	int i;
	for (i = 0; i < len; i++)
		sum = sum + data[i] * data[i];

	return sqrt(sum);
}

void cvhMinus(int left[], int right[], int dst[], int len) {
	int i;
	for (i = 0; i < len; i++)
		dst[i] = left[i] - right[i];
}

int cvhCreateImage(IplImage** img, CvSize s, int depth, int channels) {
	int R = 0;
	IplImage* src = *img;
	if (src == 0x0 || src->depth != depth || src->width != s.width
			|| src->height != s.height || src->nChannels != channels) {
		// yes it exists, but has wrong properties -> delete it!
		if (src != 0x0)
			cvReleaseImage(img);
		// now the new one can safely be created
		*img = cvCreateImage(s, depth, channels);
		R = 1;
	}
	return R;
}
int cvhCreateHist(CvHistogram** hist, int dims, int* sizes, int type,
		float** ranges, int uniform) {
	int R = 0;
	CvHistogram* src = *hist;
	if (src == 0x0) {
		// yes it exists, but has wrong properties -> delete it!
		if (src != 0x0)
			cvReleaseHist(hist);
		// now the new one can safely be created
		*hist = cvCreateHist(dims, sizes, type, ranges, uniform);
		R = 1;
	}
	return R;
}

int cvhCreateMemStorage(CvMemStorage** stor, int block_size) {
	int R = 0;
	CvMemStorage* src = *stor;
	if (src == 0x0 || src->block_size != block_size) {
		// yes it exists, but has wrong properties -> delete it!
		if (src != 0x0)
			cvReleaseMemStorage(stor);
		// now the new one can safely be created
		*stor = cvCreateMemStorage(block_size);
		R = 1;
	}
	return R;
}

int cvhSaveJPEG(const char* path, const CvArr* image, int quality) {
	int imgParams[] = { CV_IMWRITE_JPEG_QUALITY, quality, 0 };
	return cvSaveImage(path, image, imgParams);
}

void cvhPrintScalar(const char* before, CvScalar sc, const char* after) {
	cvhPrintScalarP(before, &sc, after);
}

void cvhPrintScalarP(const char* before, CvScalar* sc, const char* after) {
	printf(before);
	printf("{%.0f,%.0f,%.0f,%.0f}", sc->val[0], sc->val[1], sc->val[2],
			sc->val[3]);
	printf(after);
}

CvScalar cvhHsv2Rgb(float hue) {
	int rgb[3], p, sector;
	while ((hue >= 180))
		hue = hue - 180;

	while ((hue < 0))
		hue = hue + 180;

	int sectorData[][3] = { { 0, 2, 1 }, { 1, 2, 0 }, { 1, 0, 2 }, { 2, 0, 1 },
			{ 2, 1, 0 }, { 0, 1, 2 } };
	hue *= 0.033333333333333333333333333333333f;
	sector = cvFloor(hue);
	p = cvRound(255 * (hue - sector));
	p ^= sector & 1 ? 255 : 0;
	rgb[sectorData[sector][0]] = 255;
	rgb[sectorData[sector][1]] = 0;
	rgb[sectorData[sector][2]] = p;
	return cvScalar(rgb[2], rgb[1], rgb[0], 0);
}

void cvhDebug(const char* msg) {
#ifdef DEBUG_OUT
	printf(msg);
#endif
}

int autoIdx = 0;
char autoMsg[128];
void cvhAutoDebug() {
#ifdef DEBUG_OUT
	sprintf(autoMsg, "auto debug (%d)\n", autoIdx++);
	printf(autoMsg);
#endif
}

void cvhAutoDebugReset() {
	autoIdx = 0;
}

void cvhWaitForESC() {
	while (1) {
		//If ESC key pressed
		if ((cvWaitKey(10) & 255) == 27)
			break;
	}
}

void cvhWaitForChar(char c) {
	while (1) {
		//If ESC key pressed
		if ((cvWaitKey(10) & 255) == c)
			break;
	}
}

IplImage* cvhQueryImage(CvCapture* cap) {
	IplImage* frame = cvQueryFrame(cap);
	frame = cvQueryFrame(cap);
	return frame;
}

int cvhOddKernel(int k) {
	if (k % 2 == 0)
		k++;
	return k;
}

int fileExists(const char* file) {
	FILE *fp = fopen(file, "r");
	int ret = fp != 0x0;
	if (fp)
		fclose(fp);

	return ret;
}

void cvhSetCameraParameters(int AutoAEC, int AutoAGC, int AutoAWB,
		int Exposure, int Gain, int WhiteBalanceB, int WhiteBalanceG,
		int WhiteBalanceR) {
	HKEY hKey;
	DWORD l = sizeof(DWORD);
	char* PATH = "Software\\PS3EyeCamera\\Settings";
	int err = RegOpenKeyEx(HKEY_CURRENT_USER, PATH, 0, KEY_ALL_ACCESS, &hKey);
	if (err != ERROR_SUCCESS) {
		printf("Error: %d Unable to open reg-key:  [HKCU]\%s!", err, PATH);
		return;
	}

	if (Gain > 79)
		Gain = 79;
	if (Exposure > 511)
		Exposure = 511;

	DWORD dAutoAEC = AutoAEC > 0; // either 0 or 1
	DWORD dAutoAGC = AutoAGC > 0; // either 0 or 1
	DWORD dAutoAWB = AutoAWB > 0; // either 0 or 1
	DWORD dExposure = Exposure; // ranges from 0-511
	DWORD dGain = Gain; // ranges from 0-79
	DWORD dWhiteBalanceB = WhiteBalanceB & 0xFF; // ranges from 0-255
	DWORD dWhiteBalanceG = WhiteBalanceG & 0xFF; // ranges from 0-255
	DWORD dWhiteBalanceR = WhiteBalanceR & 0xFF; // ranges from 0-255
	if (AutoAEC >= 0)
		RegSetValueExA(hKey, "AutoAEC", 0, REG_DWORD, (CONST BYTE*) &dAutoAEC,
				l);
	if (AutoAGC >= 0)
		RegSetValueExA(hKey, "AutoAGC", 0, REG_DWORD, (CONST BYTE*) &dAutoAGC,
				l);
	if (AutoAWB >= 0)
		RegSetValueExA(hKey, "AutoAWB", 0, REG_DWORD, (CONST BYTE*) &dAutoAWB,
				l);
	if (Exposure >= 0)
		RegSetValueExA(hKey, "Exposure", 0, REG_DWORD,
				(CONST BYTE*) &dExposure, l);
	if (Gain >= 0)
		RegSetValueExA(hKey, "Gain", 0, REG_DWORD, (CONST BYTE*) &dGain, l);
	if (WhiteBalanceB >= 0)
		RegSetValueExA(hKey, "WhiteBalanceB", 0, REG_DWORD,
				(CONST BYTE*) &dWhiteBalanceB, l);
	if (WhiteBalanceG >= 0)
		RegSetValueExA(hKey, "WhiteBalanceG", 0, REG_DWORD,
				(CONST BYTE*) &dWhiteBalanceG, l);
	if (WhiteBalanceR >= 0)
		RegSetValueExA(hKey, "WhiteBalanceR", 0, REG_DWORD,
				(CONST BYTE*) &dWhiteBalanceR, l);
}

void cvhBackupCLDriverRegistry() {
	HKEY hKey;
	DWORD l = sizeof(DWORD);
	DWORD AutoAEC = 0;
	DWORD AutoAGC = 0;
	DWORD AutoAWB = 0;
	DWORD Exposure = 0;
	DWORD Gain = 0;
	DWORD wbB = 0;
	DWORD wbG = 0;
	DWORD wbR = 0;
	char* PATH = "Software\\PS3EyeCamera\\Settings";

	int err = RegOpenKeyEx(HKEY_CURRENT_USER, PATH, 0, KEY_ALL_ACCESS, &hKey);
	if (err != ERROR_SUCCESS) {
		printf("Error: %d Unable to open reg-key:  [HKCU]\%s!", err, PATH);
		return;
	}
	// if there are already backup-keys, then don't backup the values!
	err = RegQueryValueEx(hKey, "X_Gain", NULL, NULL, (LPBYTE) &Gain, &l);

	if (err != ERROR_SUCCESS) {
		RegQueryValueEx(hKey, "AutoAEC", NULL, NULL, (LPBYTE) &AutoAEC, &l);
		RegQueryValueEx(hKey, "AutoAGC", NULL, NULL, (LPBYTE) &AutoAGC, &l);
		RegQueryValueEx(hKey, "AutoAWB", NULL, NULL, (LPBYTE) &AutoAWB, &l);
		RegQueryValueEx(hKey, "Exposure", NULL, NULL, (LPBYTE) &Exposure, &l);
		RegQueryValueEx(hKey, "Gain", NULL, NULL, (LPBYTE) &Gain, &l);
		RegQueryValueEx(hKey, "WhiteBalanceB", NULL, NULL, (LPBYTE) &wbB, &l);
		RegQueryValueEx(hKey, "WhiteBalanceG", NULL, NULL, (LPBYTE) &wbG, &l);
		RegQueryValueEx(hKey, "WhiteBalanceR", NULL, NULL, (LPBYTE) &wbR, &l);

		RegSetValueExA(hKey, "X_AutoAEC", 0, REG_DWORD, (CONST BYTE*) &AutoAEC,
				l);
		RegSetValueExA(hKey, "X_AutoAGC", 0, REG_DWORD, (CONST BYTE*) &AutoAGC,
				l);
		RegSetValueExA(hKey, "X_AutoAWB", 0, REG_DWORD, (CONST BYTE*) &AutoAWB,
				l);
		RegSetValueExA(hKey, "X_Exposure", 0, REG_DWORD,
				(CONST BYTE*) &Exposure, l);
		RegSetValueExA(hKey, "X_Gain", 0, REG_DWORD, (CONST BYTE*) &Gain, l);
		RegSetValueExA(hKey, "X_WhiteBalanceB", 0, REG_DWORD,
				(CONST BYTE*) &wbB, l);
		RegSetValueExA(hKey, "X_WhiteBalanceG", 0, REG_DWORD,
				(CONST BYTE*) &wbG, l);
		RegSetValueExA(hKey, "X_WhiteBalanceR", 0, REG_DWORD,
				(CONST BYTE*) &wbR, l);
	}
}

void cvhRestoreCLDriverRegistry() {
	HKEY hKey;
	DWORD l = sizeof(DWORD);

	DWORD AutoAEC = 0;
	DWORD AutoAGC = 0;
	DWORD AutoAWB = 0;
	DWORD Exposure = 0;
	DWORD Gain = 0;
	DWORD wbB = 0;
	DWORD wbG = 0;
	DWORD wbR = 0;

	char* PATH = "Software\\PS3EyeCamera\\Settings";

	int err = RegOpenKeyEx(HKEY_CURRENT_USER, PATH, 0, KEY_ALL_ACCESS, &hKey);
	if (err != ERROR_SUCCESS) {
		printf("Error: %d Unable to open reg-key:  [HKCU]\%s!", err, PATH);
		return;
	}

	// if there is a backup, restore it!
	err = RegQueryValueEx(hKey, "X_Gain", NULL, NULL, (LPBYTE) &Gain, &l);
	if (err == ERROR_SUCCESS) {
		err = RegQueryValueEx(hKey, "X_AutoAEC", NULL, NULL, (LPBYTE) &AutoAEC,
				&l);
		if (err == ERROR_SUCCESS)
			RegSetValueExA(hKey, "AutoAEC", 0, REG_DWORD,
					(CONST BYTE*) &AutoAEC, l);

		err = RegQueryValueEx(hKey, "X_AutoAGC", NULL, NULL, (LPBYTE) &AutoAGC,
				&l);
		if (err == ERROR_SUCCESS)
			RegSetValueExA(hKey, "AutoAGC", 0, REG_DWORD,
					(CONST BYTE*) &AutoAGC, l);

		err = RegQueryValueEx(hKey, "X_AutoAWB", NULL, NULL, (LPBYTE) &AutoAWB,
				&l);
		if (err == ERROR_SUCCESS)
			RegSetValueExA(hKey, "AutoAWB", 0, REG_DWORD,
					(CONST BYTE*) &AutoAWB, l);

		err = RegQueryValueEx(hKey, "X_Exposure", NULL, NULL,
				(LPBYTE) &Exposure, &l);
		if (err == ERROR_SUCCESS)
			RegSetValueExA(hKey, "Exposure", 0, REG_DWORD,
					(CONST BYTE*) &Exposure, l);

		err = RegQueryValueEx(hKey, "X_Gain", NULL, NULL, (LPBYTE) &Gain, &l);
		if (err == ERROR_SUCCESS)
			RegSetValueExA(hKey, "Gain", 0, REG_DWORD, (CONST BYTE*) &Gain, l);

		err = RegQueryValueEx(hKey, "X_WhiteBalanceB", NULL, NULL,
				(LPBYTE) &wbB, &l);
		if (err == ERROR_SUCCESS)
			RegSetValueExA(hKey, "WhiteBalanceB", 0, REG_DWORD,
					(CONST BYTE*) &wbB, l);

		err = RegQueryValueEx(hKey, "X_WhiteBalanceG", NULL, NULL,
				(LPBYTE) &wbG, &l);
		if (err == ERROR_SUCCESS)
			RegSetValueExA(hKey, "WhiteBalanceG", 0, REG_DWORD,
					(CONST BYTE*) &wbG, l);

		err = RegQueryValueEx(hKey, "X_WhiteBalanceR", NULL, NULL,
				(LPBYTE) &wbR, &l);
		if (err == ERROR_SUCCESS)
			RegSetValueExA(hKey, "WhiteBalanceR", 0, REG_DWORD,
					(CONST BYTE*) &wbR, l);

		// remove the backup!
		RegDeleteValueA(hKey, "X_AutoAEC");
		RegDeleteValueA(hKey, "X_AutoAGC");
		RegDeleteValueA(hKey, "X_AutoAWB");
		RegDeleteValueA(hKey, "X_Exposure");
		RegDeleteValueA(hKey, "X_Gain");
		RegDeleteValueA(hKey, "X_WhiteBalanceB");
		RegDeleteValueA(hKey, "X_WhiteBalanceG");
		RegDeleteValueA(hKey, "X_WhiteBalanceR");
	}
}

