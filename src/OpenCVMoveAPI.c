#include <stdio.h>
#include "opencv2/core/core_c.h"
#include "opencv2/imgproc/imgproc_c.h"

#include "OpenCVMoveAPI.h"
#include "tracker_helpers.h"

CvScalar findMoveColorP(char* b1, char* b2, int min_radius, int max_radius) {
	IplImage* img1 = cvLoadImage(b1, 1);
	IplImage* img2 = cvLoadImage(b2, 1);
	CvScalar res = findMoveColor(img1, img2, min_radius, max_radius);
	cvReleaseImage(&img1);
	cvReleaseImage(&img2);
	return res;
}

CvMemStorage* fmc_Storage;
IplImage* fmc_hsv1;
IplImage* fmc_hsv2;
IplImage* fmc_g1;
IplImage* fmc_g2;
IplImage* fmc_mask;
IplImage* fmc_maskD;
CvScalar findMoveColor(IplImage* img1, IplImage* img2, int min_radius,
		int max_radius) {
	CvScalar res = CV_RGB(0, 0, 0);
	// img1: the sphere may be lit
	// img2: the sphere may be off
	// res: [RESULT] the average color of the sphere
	// HINT: it doesn't mater in which image the sphere is lit

	th_create_image(&fmc_g1, cvGetSize(img1), img1->depth, 1);
	th_create_image(&fmc_g2, cvGetSize(img1), img1->depth, 1);
	th_create_image(&fmc_mask, cvGetSize(img1), img1->depth, 1);
	th_create_image(&fmc_maskD, cvGetSize(img1), img1->depth, 1);

	th_create_image(&fmc_hsv1, cvGetSize(img1), img1->depth, img1->nChannels);
	th_create_image(&fmc_hsv2, cvGetSize(img1), img1->depth, img1->nChannels);

	cvCvtColor(img1, fmc_hsv1, CV_BGR2HSV);
	cvCvtColor(img2, fmc_hsv2, CV_BGR2HSV);
	/*
	 cvCvtColor(img1, fmc_g1, CV_BGR2GRAY);
	 cvCvtColor(img2, fmc_g2, CV_BGR2GRAY);
	 cvAbsDiff(fmc_g1, fmc_g2, fmc_maskD);

	 cvShowImage("img1", img1);
	 cvShowImage("img2", img2);
	 cvShowImage("diff", fmc_maskD);
	 */

	CvScalar col = CV_RGB(0xec, 0xae, 0x62);
	CvScalar hc = th_brg2hsv(col);
	// find a single circle within at least one of these pictures
	CvScalar min = cvScalar(hc.val[0] - 15, hc.val[1] - 55, hc.val[2] - 35, 0);
	CvScalar max = cvScalar(hc.val[0] + 15, hc.val[1] + 55, hc.val[2] + 35, 0);

	//cvInRangeS(fmc_hsv1, min, max, fmc_g1);
	//cvInRangeS(fmc_hsv2, min, max, fmc_g2);
	//cvInRangeS(fmc_hsv1, min, max, fmc_maskA);
	//cvInRangeS(fmc_hsv1, min, max, fmc_maskB);
	//cvAnd(fmc_maskA, fmc_maskB, fmc_mask, 0x0);
	cvInRangeS(fmc_hsv1, min, max, fmc_mask);
	cvShowImage("in range (1st)", fmc_mask);

	CvSeq* contour;

	CvMemStorage* storage = cvCreateMemStorage(0);
	cvSmooth(fmc_mask, fmc_mask, CV_GAUSSIAN, 5, 5, 0, 0);
	cvFindContours(fmc_mask, storage, &contour, sizeof(CvContour), CV_RETR_LIST,
			CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));
	for (; contour != 0; contour = contour->h_next) {
		CvScalar color = CV_RGB( rand()&255, rand()&255, rand()&255 );
		/* replace CV_FILLED with 1 to see the outlines */
		cvDrawContours(img1, contour, color, color, -1, 1, 8, cvPoint(0, 0));
		printf("A: %.0f\n", cvContourArea(contour, CV_WHOLE_SEQ, 0));
	}
	cvShowImage("contours", img1);

	return res;
	//return res;
	/*
	 CvSalar hue = cvAvg(fmc_hsv1, fmc_mask).val[0];
	 r = 5;
	 min = cvScalar(hue - r, 0, 200, 0);
	 max = cvScalar(hue + r, 255, 255, 0);
	 cvInRangeS(fmc_hsv1, min, max, fmc_mask);
	 cvShowImage("in range (2nd)", fmc_mask);*/

	//int medK = cvhODD(img1->width / 100);
	//int gauK = cvhODD(img1->width / 300);
	//cvSmooth(fmc_mask, fmc_mask, CV_MEDIAN, medK, medK, 0, 0);
	//cvSmooth(fmc_mask, fmc_mask, CV_GAUSSIAN, gauK, gauK, 0, 0);
	//scvShowImage("in range (single)", fmc_mask);
	//cvShowImage("g1", fmc_g1);
	//cvShowImage("g2", fmc_g2);
	//cvCvtColor(img1, fmc_g1, CV_BGR2GRAY);
	//cvCvtColor(img2, fmc_g2, CV_BGR2GRAY);
	//cvAbsDiff(fmc_g1, fmc_g2, fmc_mask);
	//return res;
	th_create_mem_storage(&fmc_Storage, 0);
	CvSeq* results = cvHoughCircles(fmc_mask, fmc_Storage, CV_HOUGH_GRADIENT, 2,
			fmc_mask->width / 10, 100, 100, min_radius, max_radius);

	// clear the mask
	int i;
	printf("%d\n", results->total);
	for (i = 0; i < results->total; i++) {
		cvZero(fmc_mask);
		float* p = (float*) cvGetSeqElem(results, i);
		CvPoint pt = cvPoint(cvRound(p[0]), cvRound(p[1]));
		cvCircle(fmc_mask, pt, cvRound(p[2]), CV_RGB(0xff,0xff,0xff), CV_FILLED,
				8, 0);

		// Average color of the sphere in image 1
		CvScalar lit = cvAvg(img1, fmc_mask);
		// Average color of the sphere in image 2
		CvScalar off = cvAvg(img2, fmc_mask);

		// it is most likely, that the color with the bigger variance is the color of the lit sphere
		if (th_var(off.val, 3) > th_var(lit.val, 3)) {
			CvScalar tmp = lit;
			lit = off;
			off = tmp;
		}
		if (i == 0) {
			printf("%s", "sphere off:");
			th_print_array(off.val, 3);
			printf("%s", "\n");

			printf("%s", "sphere lit:");
			th_print_array(lit.val, 3);
			printf("%s", "\n");
			res = lit;
		}
	}
	cvShowImage("Mask", fmc_mask);
	return res;
}

CvScalar trackPSMOVE_____(IplImage* img1, IplImage* img2, int min_radius,
		int max_radius) {
	CvScalar res = CV_RGB(0, 0, 0);
	// img1: the sphere may be lit
	// img2: the sphere may be off
	// res: [RESULT] the average color of the sphere
	// HINT: it doesn't mater in which image the sphere is lit
	th_create_image(&fmc_g1, cvGetSize(img1), img1->depth, 1);
	th_create_image(&fmc_g2, cvGetSize(img2), img1->depth, 1);
	th_create_image(&fmc_mask, cvGetSize(img1), img1->depth, 1);

	th_create_image(&fmc_hsv1, cvGetSize(img1), img1->depth, img1->nChannels);
	//cvhCreateImage(&fmc_hsv2, cvGetSize(img1), img1->depth, img1->nChannels);

	cvCvtColor(img1, fmc_hsv1, CV_BGR2HSV);
	//cvCvtColor(img2, fmc_hsv2, CV_BGR2HSV);

	int hMagenta = 150;

	int hue = hMagenta; //hCyan - 2;
	int r = 20;
	int value = 120;
	CvScalar min = cvScalar(hue - r, 0, value - 40, 0);
	CvScalar max = cvScalar(hue + r, 255, value + 40, 0);

	cvCvtColor(fmc_hsv1, img1, CV_HSV2BGR);
	//cvCvtColor(fmc_hsv2, img2, CV_HSV2BGR);

	//cvInRangeS(fmc_hsv1, min, max, fmc_g1);
	//cvInRangeS(fmc_hsv2, min, max, fmc_g2);
	//cvInRangeS(fmc_hsv1, min, max, fmc_maskA);
	//cvInRangeS(fmc_hsv1, min, max, fmc_maskB);
	//cvAnd(fmc_maskA, fmc_maskB, fmc_mask, 0x0);
	cvInRangeS(fmc_hsv1, min, max, fmc_mask);

	int medK = img1->width / 150;
	int gauK = img1->width / 300;

	if (medK % 2 == 0)
		medK++;

	if (gauK % 2 == 0)
		gauK++;

	cvSmooth(fmc_mask, fmc_mask, CV_MEDIAN, medK, medK, 0, 0);
	cvSmooth(fmc_mask, fmc_mask, CV_GAUSSIAN, gauK, gauK, 0, 0);
	cvShowImage("in range (single)", fmc_mask);
	//cvShowImage("g1", fmc_g1);
	//cvShowImage("g2", fmc_g2);

	//cvCvtColor(img1, fmc_g1, CV_BGR2GRAY);
	//cvCvtColor(img2, fmc_g2, CV_BGR2GRAY);
	//cvAbsDiff(fmc_g1, fmc_g2, fmc_mask);
	//return res;

	th_create_mem_storage(&fmc_Storage, 0);
	CvSeq* results = cvHoughCircles(fmc_mask, fmc_Storage, CV_HOUGH_GRADIENT, 2,
			fmc_mask->width / 10, 100, 100, min_radius, max_radius);

	// clear the mask
	int i;
	for (i = 0; i < results->total; i++) {
		//cvZero(fmc_mask);
		float* p = (float*) cvGetSeqElem(results, i);
		CvPoint pt = cvPoint(cvRound(p[0]), cvRound(p[1]));
		cvCircle(fmc_mask, pt, cvRound(p[2]), CV_RGB(0xff,0xff,0xff), CV_FILLED,
				8, 0);

		// Average color of the sphere in image 1
		CvScalar lit = cvAvg(img1, fmc_mask);
		// Average color of the sphere in image 2
		CvScalar off = cvAvg(img2, fmc_mask);

		// it is most likely, that the color with the bigger variance is the color of the lit sphere
		if (th_var(off.val, 3) > th_var(lit.val, 3)) {
			CvScalar tmp = lit;
			lit = off;
			off = tmp;
		}
		if (i == 0) {
			printf("%s", "sphere off");
			th_print_array(off.val, 3);
			printf("%s", "sphere lit");
			th_print_array(lit.val, 3);
			res = lit;
		}
		break;
	}
	cvShowImage("Mask", fmc_mask);
	return res;
}

CvScalar findOptimalMoveColorP(char* path, int h_bins, int kSize) {
	CvScalar cols[] = { CV_RGB(0, 0, 0) };
	findOptimalMoveColorsP(path, h_bins, kSize, cols, 1);
	return cols[0];
}
CvScalar findOptimalMoveColor(IplImage* img, int h_bins, int kSize) {
	CvScalar cols[] = { CV_RGB(0, 0, 0) };
	findOptimalMoveColors(img, h_bins, kSize, cols, 1);
	return cols[0];
}

void findOptimalMoveColorsP(char* path, int h_bins, int kSize, CvScalar dst[],
		int num) {
	IplImage* img = cvLoadImage(path, 1);
	findOptimalMoveColors(img, h_bins, kSize, dst, num);
	cvReleaseImage(&img);
}

IplImage* ahsv;
IplImage* ah_plane;
CvHistogram* ahist;
IplImage* ahist_img;

void findOptimalMoveColors(IplImage* img, int h_bins, int kSize, CvScalar dst[],
		int num) {
	// Compute HSV image and separate into colors
	th_create_image(&ahsv, cvGetSize(img), img->depth, 3);
	th_create_image(&ah_plane, cvGetSize(img), 8, 1);
	cvCvtColor(img, ahsv, CV_BGR2HSV);
	cvSplit(ahsv, ah_plane, 0, 0, 0);
	// Build and fill the histogram
	IplImage* planes[] = { ah_plane };
	int hist_size[] = { h_bins };
	float h_ranges[] = { 0, 180 };
	float* ranges[] = { h_ranges };
	th_create_hist(&ahist, 1, hist_size, CV_HIST_ARRAY, ranges, 1);
	cvCalcHist(planes, ahist, 0, 0); // Compute histogram
	cvNormalizeHist(ahist, 1.0); // Normalize it

	// Create an image to visualize the histogram
	int scale = 10;
	th_create_image(&ahist_img, cvSize(h_bins * scale, 8 * scale), 8, 3);

	cvZero(ahist_img);
	// populate the visualization
	float max_value = 0;
	cvGetMinMaxHistValue(ahist, 0, &max_value, 0, 0);
	// insert artificial maxima at the [borders] of the image
	cvSetReal1D(ahist->bins, 0, max_value / 2);
	cvSetReal1D(ahist->bins, h_bins - 1, max_value / 2);

	int h;
	CvPoint p1, p2;
	CvScalar col;
	// draw intensity values
	for (h = 0; h < h_bins; h++) {
		double bin_val = cvGetReal1D(ahist->bins, h);
		int intensity = cvRound(bin_val * 255 / max_value);
		p1 = cvPoint(h * scale, 0);
		p2 = cvPoint((h + 1) * scale - 1, ahist_img->height - 1);
		cvRectangle(ahist_img, p1, p2, cvScalarAll(intensity), CV_FILLED, 8, 0);
	}

	// adjust the smoothing kernel size to the scale
	kSize = kSize * scale;
	if (kSize > 0) {
		if (kSize % 2 == 0)
			kSize = kSize + 1;
		// average the intensities
		cvSmooth(ahist_img, ahist_img, CV_GAUSSIAN, kSize, kSize, 0, 0);
	}

	int i, j;
	int histIdx;
	double minProb;
	int bestHueIdxs[num];
	for (i = 0; i < num; i++) {
		int minDist = (int) round(h_bins * 0.5 / num);
		int maxDist = (int) round(h_bins * 1.0 + (0.1 * num) / num);
		int distOK;

		// find the next best suitable color
		histIdx = -1;
		minProb = 256;
		for (h = 0; h < h_bins; h++) {
			int prob = cvGet2D(ahist_img, 0, h * scale).val[0];
			if (prob < minProb) {
				distOK = 1;
				// does it satisfy the dist-constraint?
				for (j = 0; j < i; j++) {
					int x = abs(bestHueIdxs[j] - h);
					// it shall not be too near/far!
					if (x < minDist || x > maxDist) {
						distOK = 0;
						break;
					}
				}
				// if dist-constraint is held --> this one is okay!
				if (distOK) {
					minProb = prob;
					histIdx = h;
				}
			}
		}
		// save the next best suitable color
		bestHueIdxs[i] = histIdx;
		if (histIdx != -1) {
			dst[i] = th_hsv2bgr(cvScalar(histIdx * 180 / h_bins, 255, 255, 0));
			dst[i].val[3] = minProb;
		} else
			printf("Unable to find color: %d!\n", i);
	}

	// draw histogram colors
	for (h = 0; h < h_bins; h++) {
		col = th_hsv2bgr(cvScalar(h * 180 / h_bins, 255, 255, 0));
		p1 = cvPoint(h * scale, ahist_img->height / 2);
		p2 = cvPoint((h + 1) * scale - 1, ahist_img->height - 1);
		cvRectangle(ahist_img, p1, p2, col, CV_FILLED, 8, 0);
	}

	// mark all best colors
	for (i = 0; i < num; i++) {
		histIdx = bestHueIdxs[i];
		p1 = cvPoint(histIdx * scale, ahist_img->height / 2);
		p2 = cvPoint((histIdx + 1) * scale - 1, ahist_img->height - 1);
		cvRectangle(ahist_img, p1, p2, cvScalarAll(255), 2, 8, 0);
	}

	// Show histogram equalized
	cvShowImage("H Histogram (best colors)", ahist_img);
}
