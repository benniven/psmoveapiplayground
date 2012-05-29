#include "OpenCVHelpers.h"

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

