#include "MoveAPIOptions.h"
#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"

#include "..\psmoveapi\psmove.h"

#ifndef OPEN_CV_HELPERS_H
#define OPEN_CV_HELPERS_H

double cvhStd(int data[], int len);
double cvhVar(int data[], int len);
double cvhAvg(int data[], int len);

double cvhStdF(double data[], int len);
double cvhVarF(double data[], int len);
double cvhAvgF(double data[], int len);

double cvhMagnitude(int data[], int len);
void cvhMinus(int left[], int right[], int dst[], int len);

// only creates the image/storage, if it does not already exist, or has different properties (size, depth, channels, blocksize ...)
// returns (1: if image/storage is created) (0: if nothing has been done)
int cvhCreateImage(IplImage** img, CvSize s, int depth, int channels);
int cvhCreateMemStorage(CvMemStorage** stor, int block_size);
int cvhCreateHist(CvHistogram** hist, int dims, int* sizes, int type,
		float** ranges, int uniform);

// simly saves a CvArr* on the filesystem
int cvhSaveJPEG(const char* path, const CvArr* image, int quality);

// prints a [bgra]-Color to stdout: {255,0,0,0}
void cvhPrintScalar(const char* before, CvScalar sc, const char* after);
void cvhPrintScalarP(const char* before, CvScalar* sc, const char* after);

// converts a [0-160] hue value into a bgra-Color
CvScalar cvhHsv2Rgb(float hue);

// prints a message to stdout if DEBUG_OUT is defined
void cvhDebug(const char* msg);
void cvhAutoDebug();
void cvhAutoDebugReset();

// waits until the uses presses ESC (only works if a windo is visible)
void cvhWaitMoveButton(PSMove* move, int button);
void cvhWaitForESC();
void cvhWaitForChar(char c);

// querys a frame from the defined capture and waits for "useconds" microseconds before returning
IplImage* cvhQueryImage(CvCapture* cap);

// gets an odd Kernel (does +1 if even)
int cvhOddKernel(int k);

void cvhSetCameraParameters(int AutoAEC, int AutoAGC, int AutoAWB,
							int Exposure, int Gain,
							int WhiteBalanceB, int WhiteBalanceG, int WhiteBalanceR);


void cvhBackupCLDriverRegistry();
void cvhRestoreCLDriverRegistry();


#endif

