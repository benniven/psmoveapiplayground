#include "opencv2/core/core_c.h"
#include "MoveAPIOptions.h"

#ifndef OPEN_CV_MOVE_API_H
#define OPEN_CV_MOVE_API_H
CvScalar findMoveColor(IplImage* img1, IplImage* img2, int min_radius,
		int max_radius);
CvScalar findMoveColorP(char* b1, char* b2, int min_radius, int max_radiu);

CvScalar findOptimalMoveColorP(char* path, int h_bins, int kSize);
CvScalar findOptimalMoveColor(IplImage* img, int h_bins, int kSize);

void findOptimalMoveColorsP(char* path, int h_bins, int kSize, CvScalar dst[], int num);
void findOptimalMoveColors(IplImage* img, int h_bins, int kSize, CvScalar dst[], int num);
#endif
