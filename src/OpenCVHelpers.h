#include "MoveAPIOptions.h"
#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"
#include <time.h>
#include "..\psmoveapi\psmove.h"

#ifndef OPEN_CV_HELPERS_H
#define OPEN_CV_HELPERS_H

#define cvhODD(x) (((x)/2)*2+1)
#define cvhClock2Sec(clocks) (1.0 / (((double) (clocks)) / CLOCKS_PER_SEC))
#define cvhBlack cvScalarAll(0x0)
#define cvhWhite cvScalarAll(0xFF)

double cvhVar(double* src, int len);
double cvhAvg(double* src, int len);
double cvhMagnitude(double* src, int len);
void cvhMinus(double* l, double* r, double* result, int len);

// only creates the image/storage, if it does not already exist, or has different properties (size, depth, channels, blocksize ...)
// returns (1: if image/storage is created) (0: if nothing has been done)
int cvhCreateImage(IplImage** img, CvSize s, int depth, int channels);
int cvhCreateMemStorage(CvMemStorage** stor, int block_size);
int cvhCreateHist(CvHistogram** hist, int dims, int* sizes, int type,
		float** ranges, int uniform);

void cvhPutText(IplImage* img, const char* text, CvPoint p, CvScalar color);

// simly saves a CvArr* on the filesystem
int cvhSaveJPEG(const char* path, const CvArr* image, int quality);

// prints a array to system out ala {a,b,c...}
void cvhPrintArray(double* src, int len);

// converts HSV color to a BGR color and back
CvScalar cvhHSV2BGR(CvScalar hsv);
CvScalar cvhBGR2HSV(CvScalar bgr);

// prints a message to stdout if DEBUG_OUT is defined
void cvhDebug(const char* msg);
void cvhAutoDebug();
void cvhAutoDebugReset();

// waits until the uses presses ESC (only works if a windo is visible)
void cvhWaitForESC();
void cvhWaitForChar(char c);
void cvhWaitMoveButton(PSMove* move, int button);

// querys a frame from the defined capture and waits for "useconds" microseconds before returning
IplImage* cvhQueryImage(CvCapture* cap);

void cvhSetCameraParameters(int AutoAEC, int AutoAGC, int AutoAWB,
		int Exposure, int Gain, int WhiteBalanceB, int WhiteBalanceG,
		int WhiteBalanceR);

void cvhBackupCameraSettings();
void cvhRestoreCameraSettings();

#endif

