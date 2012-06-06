#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <windows.h>

#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"

#include "OpenCVHelpers.h"
#include "OpenCVMoveAPI.h"
#include "..\psmoveapi\psmove.h"
#include "HighPrecisionTimer.h"

void calibrate();
void getDiff(CvCapture** capture, PSMove* controller, int exp, IplImage* on,
		IplImage* diff);

void videoHist();
void videoGuess();
void diff();
int adaptToLighting(int lumMin, int expMin, int expMax);
void autoWB();
PSMove* connectController();

#define BCr 0x00
#define BCg 0xff
#define BCb 0xFF

int main(int arg, char** args) {
	//cvhBackupCLDriverRegistry();
	calibrate();
	//autoWB();
	//diff();
	//videoHist();
	//videoGuess();
	return 0;
}

void calibrate() {
	PSMove* controller = connectController();
	HPTimer* timer = createTimer();
	int exp = adaptToLighting(20, 0x10, 0x18);
	cvhSetCameraParameters(0, 0, 0, exp, 0, 0xff, 0xff, 0xff);
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	CvMemStorage* storage = cvCreateMemStorage(0);
	if (!capture) {
		fprintf(stderr, "ERROR: capture is NULL \n");
		getchar();
	}

	int i;
	int blinks = 4;
	CvScalar r = cvScalar(15, 85, 35, 0);
	IplImage* images[blinks];
	IplImage* diffs[blinks];
	double sizes[blinks];
	IplImage* frame = cvQueryFrame(capture);
	IplImage* mask = cvCreateImage(cvGetSize(frame), frame->depth, 1);
	IplImage* hsvFrame = cvCreateImage(cvGetSize(frame), frame->depth,
			frame->nChannels);
	for (i = 0; i < blinks; i++) {
		images[i] = cvCreateImage(cvGetSize(frame), frame->depth,
				frame->nChannels);
		diffs[i] = cvCreateImage(cvGetSize(frame), frame->depth, 1);
	}
	CvScalar c;
	CvScalar ic;
	CvScalar hc;
	CvScalar min, max;
	char fpss[256];
	while (1) {
		// wait until the user presses a controller button
		//cvhWaitMoveButton(controller, Btn_T);
		for (i = 0; i < blinks; i++) {
			getDiff(&capture, controller, -1, images[i], diffs[i]);

			//sprintf(fpss, "%d.diff-raw.jpg", (int) images[i]);
			//cvhSaveJPEG(fpss, diffs[i], 100);
			//cvShowImage("raw diff", diffs[i]);
			// TODO: ist das eine gute schwelle?
			cvThreshold(diffs[i], diffs[i], 20, 0xFF, CV_THRESH_BINARY);

			cvErode(diffs[i], diffs[i], 0x0, 1);
			cvDilate(diffs[i], diffs[i], 0x0, 1);

			//sprintf(fpss, "%d.diff-th.jpg", (int) images[i]);
			//cvhSaveJPEG(fpss, diffs[i], 100);

			//cvShowImage("thresholded diff", diffs[i]);
			//cvShowImage("original image", images[i]);
			//cvhWaitForESC();
		}
		// put the diff images together!
		for (i = 1; i < blinks; i++) {
			cvAnd(diffs[0], diffs[i], diffs[0], 0x0);
		}
		//cvErode(diffs[0], diffs[0], 0x0, 1);
		//cvDilate(diffs[0], diffs[0], 0x0, 1);
		//cvShowImage("diff-final", diffs[0]);
		//cvhWaitForESC();

		//sprintf(fpss, "diff-final.jpg");
		//cvhSaveJPEG(fpss, diffs[0], 100);

		// calculate the avg color!
		c = cvAvg(images[blinks - 1], diffs[0]);
		ic = cvScalar(0xff - c.val[0], 0xff - c.val[1], 0xff - c.val[1], 0);
		hc = cvhBGR2HSV(c);
		// find a single contour with that color within the calibration pictures
		cvhMinus(hc.val, r.val, min.val, 3);
		cvhPlus(hc.val, r.val, max.val, 3);
		for (i = 0; i < blinks; i++) {
			cvCvtColor(images[i], hsvFrame, CV_BGR2HSV);
			cvInRangeS(hsvFrame, min, max, mask);

			//cvShowImage("raw image", mask);
			cvErode(mask, mask, 0x0, 1);
			cvDilate(mask, mask, 0x0, 1);

			CvSeq* contour;
			//cvShowImage("filtered image", mask);
			//cvShowImage("original image", images[i]);
			//cvhWaitForESC();

			cvFindContours(mask, storage, &contour, sizeof(CvContour),
					CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

			// there may be only a single contour in this picture with at least 100px size
			sizes[i] = 0;
			if (contour != 0x0)
				sizes[i] = cvContourArea(contour, CV_WHOLE_SEQ, 0);

			if (contour == 0x0 || contour->h_next != 0x0
					|| contour->h_prev != 0x0 || sizes[i] <= 100) {
				if (contour == 0x0)
					printf("%s\n", "no contours!");
				else if (contour->h_next != 0x0 || contour->h_prev != 0x0)
					printf("%s\n", "there are other contours!");
				else if (sizes[i] <= 100)
					printf("%s\n", "the contour is to small!");
				sizes[i] = 0;
			}
		}

		// check if the size of the found contours are near to each other
		// and if there were found contours at al :P
		double stdSizes = sqrt(cvhVar(sizes, blinks));
		cvhPrintArray(sizes, blinks);
		//printf("V: %.0f\n", stdSizes);
		//printf("%s", "C: ");
		//cvhPrintArray(c.val, 3);
		if (stdSizes < (cvhAvg(sizes, blinks) * 0.10) && sizes[0] > 0.0)
			break;
	}

	CvRect roi = cvRect(0, 0, frame->width, frame->height);
	CvRect br;
	int rois = 5;
	IplImage* roiI[rois];
	IplImage* roiM[rois];
	int roiIdx = 0;
	int w;
	roiI[0] = hsvFrame;
	roiM[0] = mask;
	for (i = 1; i < rois; i++) {
		IplImage* z = roiI[i - 1];
		w = z->width;
		roiI[i] = cvCreateImage(cvSize(w * 0.7, w * 0.7), z->depth,
				z->nChannels);
		roiM[i] = cvCreateImage(cvSize(w * 0.7, w * 0.7), z->depth, 1);
	}

	cvhMinus(hc.val, r.val, min.val, 3);
	cvhPlus(hc.val, r.val, max.val, 3);
	float avgLum = 0;
	CvScalar avgC;
	// this seems to be a nice color
	while (1) {
		int key = (cvWaitKey(10) & 255);
		frame = cvhQueryImage(capture);

		//frame = cvhQueryEqualizedImage(capture);
		avgC = cvAvg(frame, 0x0);
		avgLum = cvhAvg(avgC.val, 3);
		//cvSubS(frame, wb, frame, 0x0);
		if (!frame)
			continue;
		CvPoint p;

		// this is the tracking algo
		startTimer(timer);
		while (1) {
			// cut out the roi!
			cvSetImageROI(frame, roi);
			cvCvtColor(frame, roiI[roiIdx], CV_BGR2HSV);
			// find the blob
			cvInRangeS(roiI[roiIdx], min, max, roiM[roiIdx]);

			//cvShowImage("mask step 0", mask);

			// this will remove small distortions that have a similar color

			cvErode(roiM[roiIdx], roiM[roiIdx], 0x0, 1);
			cvDilate(roiM[roiIdx], roiM[roiIdx], 0x0, 1);

			//cvShowImage("mask step 1", mask);

			CvSeq* contour = 0x0;
			CvSeq* aC = 0x0;
			CvSeq* best = 0x0;
			cvFindContours(roiM[roiIdx], storage, &contour, sizeof(CvContour),
					CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

			float sizeC = 0;
			float f = 0;

			for (aC = contour; aC != 0x0; aC = aC->h_next) {
				f = cvContourArea(aC, CV_WHOLE_SEQ, 0);
				if (f > sizeC) {
					sizeC = f;
					best = aC;
					cvDrawContours(roiM[roiIdx], aC, cvScalar(0, 0xff, 0, 0),
							cvScalar(0, 0xff, 0, 0), -1, 1, 8, cvPoint(0, 0));
				}
			}
			cvSet(roiM[0], cvhBlack, 0x0);
			cvSet(roiM[roiIdx], cvhBlack, 0x0);
			if (best) {
				cvDrawContours(roiM[roiIdx], best, cvhWhite, cvhWhite, -1,
						CV_FILLED, 8, cvPoint(0, 0));

				cvDrawContours(roiM[0], best, cvhWhite, cvhWhite, -1, CV_FILLED,
						8, cvPoint(roi.x, roi.y));

				CvMoments mu;
				cvMoments(roiM[roiIdx], &mu, 0);
				p = cvPoint(mu.m10 / mu.m00, mu.m01 / mu.m00);
				p.x = p.x + roi.x;
				p.y = p.y + roi.y;

				// update the feature roi box!
				br = cvBoundingRect(best, 0);
				br.width = cvhMAX(br.width, br.height) * 2;
				br.height = br.width;
				for (i = 0; i < rois; i++) {
					if (br.width > roiI[i]->width
							&& br.height > roiI[i]->height)
						break;
					roiIdx = i;
					roi.width = roiI[i]->width;
					roi.height = roiI[i]->height;
				}
				roi.x = p.x - roi.width / 2;
				roi.y = p.y - roi.height / 2;
				if (roi.x < 0)
					roi.x = 0;
				if (roi.y < 0)
					roi.y = 0;

				if (roi.x + roi.width > roiI[0]->width)
					roi.x = roiI[0]->width - roi.width;
				if (roi.y + roi.height > roiI[0]->height)
					roi.y = roiI[0]->height - roi.height;
			}
			cvResetImageROI(frame);
			cvRectangle(frame, cvPoint(roi.x, roi.y),
					cvPoint(roi.x + roi.width, roi.y + roi.height), cvhWhite, 3,
					8, 0);
			cvRectangle(frame, cvPoint(roi.x, roi.y),
					cvPoint(roi.x + roi.width, roi.y + roi.height), cvhRed, 1,
					8, 0);

			if (best || roi.width == roiI[0]->width) {
				break;
			} else {
				if (roiIdx == 0)
					break;
				roiIdx = roiIdx - 1;
				roi.x = roi.x + (roiI[roiIdx]->width - roi.width) / 2;
				roi.y = roi.y + (roiI[roiIdx]->height - roi.height) / 2;
				roi.width = roiI[roiIdx]->width;
				roi.height = roiI[roiIdx]->height;

				if (roi.x < 0)
					roi.x = 0;
				if (roi.y < 0)
					roi.y = 0;
				if (roi.x + roi.width > roiI[0]->width)
					roi.x = roiI[0]->width - roi.width;
				if (roi.y + roi.height > roiI[0]->height)
					roi.y = roiI[0]->height - roi.height;
			}
		}

		stopTimer(timer);
		cvShowImage("mask", roiM[0]);
		sprintf(fpss,
				"@%.0ffps   Sphere-RGB(%.0f,%.0f,%.0f)  avg/Lum(%.0f)   exp(%d)",
				1.0 / getElapsedTime(timer), c.val[0], hc.val[1], hc.val[2],
				avgLum, exp);
		cvhPutText(frame, fpss, cvPoint(10, 20), c);
		cvCircle(frame, p, 4, cvhBlack, 4, 8, 0);
		cvCircle(frame, p, 4, ic, 2, 8, 0);
		cvShowImage("live camera feed", frame);
		if (cvhMoveButton(controller, Btn_T))
			psmove_set_leds(controller, 0, 0, 0);
		else
			psmove_set_leds(controller, BCr, BCg, BCb);
		psmove_update_leds(controller);
		//If ESC key pressed
		if (key == 27)
			break;
	}
	releaseTimer(timer);
	psmove_disconnect(controller);
	cvReleaseCapture(&capture);
}

void getDiff(CvCapture** capture, PSMove* controller, int exp, IplImage* on,
		IplImage* diff) {
	int delay = 1000000 / 4;
	IplImage* frame;
	if (exp >= 0) {
		cvReleaseCapture(capture);
		cvhSetCameraParameters(0, 0, 1, exp, 0, -1, -1, -1);
		usleep(delay);
		cvCaptureFromCAM(CV_CAP_ANY);
	}
	psmove_set_leds(controller, BCr, BCg, BCb);
	psmove_update_leds(controller);
	usleep(delay);
	frame = cvhQueryImage(*capture);
// copy the color picture!
	cvCopy(frame, on, 0x0);

	psmove_set_leds(controller, 0, 0, 0);
	psmove_update_leds(controller);
	usleep(delay);
	frame = cvhQueryImage(*capture);

	IplImage* grey1 = cvCloneImage(diff);
	IplImage* grey2 = cvCloneImage(diff);
	IplImage* mask1 = cvCloneImage(diff);
	IplImage* mask2 = cvCloneImage(diff);

	cvCvtColor(frame, grey1, CV_BGR2GRAY);
	cvCvtColor(on, grey2, CV_BGR2GRAY);
//char text[256];
//sprintf(text, "%d.original_ON.jpg", (int) on);
//cvhSaveJPEG(text, on, 100);
//sprintf(text, "%d.original_OFF.jpg", (int) on);
//cvhSaveJPEG(text, frame, 100);
	/*
	 // find all white areas
	 CvScalar u = cvScalarAll(250);
	 CvScalar l = cvScalarAll(0);
	 cvInRangeS(grey1, l, u, mask1);
	 cvInRangeS(grey2, l, u, mask2);

	 // blurr the white areas (aka Korona)
	 int k = 13;
	 cvSmooth(mask1, mask1, CV_GAUSSIAN, k, k, 0, 0);
	 cvSmooth(mask2, mask2, CV_GAUSSIAN, k, k, 0, 0);

	 cvThreshold(mask1, mask1, 254, 0xFF, CV_THRESH_BINARY);
	 cvThreshold(mask2, mask2, 254, 0xFF, CV_THRESH_BINARY);

	 // remove all white areas(inc. korona) from the target images
	 cvAnd(grey1, mask1, grey1, 0x0);
	 cvAnd(grey2, mask1, grey2, 0x0);
	 cvAnd(grey1, mask2, grey1, 0x0);
	 cvAnd(grey2, mask2, grey2, 0x0);
	 */
// calculate the diff of these two cleaned up images
	cvAbsDiff(grey1, grey2, diff);

	cvReleaseImage(&grey1);
	cvReleaseImage(&grey2);
	cvReleaseImage(&mask1);
	cvReleaseImage(&mask2);
}

PSMove* connectController() {
	PSMove *move;
	enum PSMove_Connection_Type ctype;
	int i;

	i = psmove_count_connected();
	printf("Connected controllers: %d\n", i);

	move = psmove_connect();

	if (move == NULL) {
		printf("Could not connect to default Move controller.\n"
				"Please connect one via USB or Bluetooth.\n");
		exit(1);
	}

	ctype = psmove_connection_type(move);
	switch (ctype) {
	case Conn_USB:
		printf("Connected via USB.\n");
		break;
	case Conn_Bluetooth:
		printf("Connected via Bluetooth.\n");
		break;
	case Conn_Unknown:
		printf("Unknown connection type.\n");
		break;
	}

	if (ctype == Conn_USB) {
		PSMove_Data_BTAddr addr;
		psmove_get_btaddr(move, &addr);
		printf("Current BT Host: ");
		for (i = 0; i < 6; i++) {
			printf("%02x ", addr[i]);
		}
		printf("\n");
	}

	return move;
}

int adaptToLighting(int lumMin, int expMin, int expMax) {

	int exp = expMin;
	cvhSetCameraParameters(0, 0, 0, exp, 0, 0xff, 0xff, 0xff);

	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);

	if (!capture) {
		fprintf(stderr, "ERROR: capture is NULL \n");
		getchar();
	}

	IplImage* frame;
	int step = expMax / expMin;
	if (step == 0)
		step = 1;

	char text[512];
	int counter = 0;
	int lastExp = exp;
	while (1) {
		frame = cvhQueryImage(capture);

		if (!frame)
			continue;

		CvScalar avgColor = cvAvg(frame, 0x0);
		float avgLum = cvhAvg(avgColor.val, 3);
		sprintf(text, "Exposure: %d (0x%x)", exp, exp);
		//cvhPutText(frame, text, cvPoint(10, 20), CV_RGB(0,200,0));
		sprintf(text, "Avg Lum=%.0f", avgLum);
		//cvhPutText(frame, text, cvPoint(10, 60), CV_RGB(0,200,0));

		//cvShowImage("live camera feed", frame);

		int key = (cvWaitKey(10) & 255);
		if (key == 27)
			break;

		if (counter >= 10) {
			printf("Exposure: %d (0x%x)\n", exp, exp);
			printf("Avg Lum=%.0f\n", avgLum);

			if (avgLum < lumMin)
				exp = exp + step;

			if (exp < expMin)
				exp = expMin;
			if (exp > expMax)
				exp = expMax;

			if (lastExp != exp) {
				// reconfigure the camera!
				cvhSetCameraParameters(0, 0, 0, exp, 0, 0xff, 0xff, 0xff);
				cvReleaseCapture(&capture);
				capture = cvCaptureFromCAM(CV_CAP_ANY);
				usleep(10000);
				counter = 0;
				lastExp = exp;
			} else
				break;
		}
		counter++;
	}
// Release the capture device housekeeping
	cvReleaseCapture(&capture);
	return exp;
}

void autoWB() {

	int gain = 0;
	int exp = adaptToLighting(20, 0x10, 0x18);

	cvhSetCameraParameters(0, 0, 0, exp, gain, 0xff, 0xff, 0xff);

	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);

	if (!capture) {
		fprintf(stderr, "ERROR: capture is NULL \n");
		getchar();
	}

	int h_bins = 64;
	int hist_size[] = { h_bins };
	float h_ranges[] = { 0, 0xFF };
	float* ranges[] = { h_ranges };
	CvHistogram* ahist = cvCreateHist(1, hist_size, CV_HIST_ARRAY, ranges, 1);

	IplImage* frame = 0x0;
	IplImage* hsv = 0x0;
	IplImage* h_plane = 0x0;
	IplImage* s_plane = 0x0;
	IplImage* v_plane = 0x0;

	CvScalar avgColor;
	char text[512];
	int GATE = 1;
	while (1) {
		frame = cvhQueryImage(capture);

		if (!frame)
			continue;

		cvhCreateImage(&hsv, cvGetSize(frame), frame->depth, frame->nChannels);
		cvhCreateImage(&h_plane, cvGetSize(frame), frame->depth, 1);
		cvhCreateImage(&s_plane, cvGetSize(frame), frame->depth, 1);
		cvhCreateImage(&v_plane, cvGetSize(frame), frame->depth, 1);

		cvCvtColor(frame, hsv, CV_BGR2HSV);
		cvSplit(hsv, h_plane, s_plane, v_plane, 0);

		cvCalcHist(&v_plane, ahist, 0, 0); // Compute histogram
		cvNormalizeHist(ahist, 255); // Normalize it
		cvhPlotHistogram(ahist, h_bins, "Value Histogram", cvhWhite);

		avgColor = cvAvg(frame, 0x0);
		sprintf(text, "Exposure: %d (0x%x)", exp, exp);
		cvhPutText(frame, text, cvPoint(10, 20), CV_RGB(0,200,0));
		sprintf(text, "Gain: %d (0x%x)", gain, gain);
		cvhPutText(frame, text, cvPoint(10, 40), CV_RGB(0,200,0));
		sprintf(text, "Avg Lum=%.0f", cvhAvg(avgColor.val, 3));
		cvhPutText(frame, text, cvPoint(10, 60), CV_RGB(0,200,0));

		cvShowImage("live camera feed", frame);

		//cvShowImage("h frame", h_plane);
		//cvShowImage("s frame", s_plane);
		cvShowImage("v frame", v_plane);

		//If ESC key pressed
		int key = (cvWaitKey(10) & 255);
		if (key == 27)
			break;
		if (key == '+')
			exp = (exp * 5) / 4 + 1;
		if (key == '-')
			exp = (exp * 4) / 5;
		if (key == '1')
			gain = gain * 2 + 1;
		if (key == '2')
			gain = gain / 2;

		if (GATE && (key == '+' || key == '-' || key == '1' || key == '2')) {
			GATE = 0;
			cvhSetCameraParameters(0, 0, 0, exp, gain, 0xff, 0xff, 0xff);
			cvReleaseCapture(&capture);
			capture = cvCaptureFromCAM(CV_CAP_ANY);
			usleep(10000);
			GATE = 1;
		}

	}
// Release the capture device housekeeping
	cvReleaseCapture(&capture);
}

void diff() {
	IplImage* img1 = cvLoadImage("out.jpg", 0);
	IplImage* img2 = cvLoadImage("out1.jpg", 0);
	IplImage* img3 = cvLoadImage("out2.jpg", 0);

	int medK = cvhODD(img1->width / 100);
	cvShowImage("1", img1);
	cvShowImage("2", img2);
	cvShowImage("3", img3);

	cvAnd(img1, img2, img1, 0x0);
	cvAnd(img1, img3, img1, 0x0);

	cvSmooth(img1, img1, CV_MEDIAN, medK, medK, 0, 0);
	cvThreshold(img1, img1, 50, 255, CV_THRESH_BINARY);
	cvShowImage("M+G", img1);
//cvhSaveJPEG("erg.jpg", img1, 100);
	cvhWaitForESC();
}

void videoGuess() {
	cvNamedWindow("videoGuess", 1);
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);

	if (!capture) {
		fprintf(stderr, "ERROR: capture is NULL \n");
		getchar();
	}

	CvFont font;
	double hScale = 0.5;
	double vScale = 0.5;
	int lineWidth = 1;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC, hScale, vScale,
			0, lineWidth, CV_AA);

	IplImage* cap;

	while (1) {
		cap = cvhQueryImage(capture);
		if (!cap)
			continue;
		/*
		 printf("press 'p' for frame 1\n");
		 cvhWaitForChar('p');
		 // Get 1st frame
		 cap = cvhQueryImage(capture);
		 // init memory
		 cvhCreateImage(&frame1, cvGetSize(cap), cap->depth, cap->nChannels);
		 cvhCreateImage(&frame2, cvGetSize(cap), cap->depth, cap->nChannels);
		 // copy the frame
		 cvCopy(cap, frame1, 0x0);

		 printf("press 'p' for frame 2\n");
		 cvhWaitForChar('p');
		 // Get 2nd frame
		 cap = cvhQueryImage(capture);
		 cvCopy(cap, frame2, 0x0);*/
		findMoveColor(cap, cap, 0, 0);
		//If ESC key pressed
		int key = (cvWaitKey(10) & 255);
		if (key == 27)
			break;
	}
// Release the capture device housekeeping
	cvReleaseCapture(&capture);
}

void videoHist() {

	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);

	if (!capture) {
		fprintf(stderr, "ERROR: capture is NULL \n");
		getchar();
	}

	cvSetCaptureProperty(capture, CV_CAP_PROP_SATURATION, 0.23);
	cvSetCaptureProperty(capture, CV_CAP_PROP_CONTRAST, 0.08);
	cvSetCaptureProperty(capture, CV_CAP_PROP_EXPOSURE, 0.58);
	CvFont font;
	double hScale = 0.5;
	double vScale = 0.5;
	int lineWidth = 1;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC, hScale, vScale,
			0, lineWidth, CV_AA);

// Create a window in which the captured images will be presented
// Show the image captured from the camera in the window and repeat
	float diff = 0;
	int tic;
	char fps[256];
	CvScalar bestColors[2];
	while (1) {
		// Get one frame

		IplImage* frame = cvQueryFrame(capture);
		//image = frame;
		if (!frame) {
			fprintf(stderr, "ERROR: frame is null...\n");
			getchar();
			break;
		}

		tic = clock();
		findOptimalMoveColors(frame, 64, 5, bestColors, 2);
		diff = 0.7 * diff + 0.3 * cvhClock2Sec(clock() - tic);

		sprintf(fps, "@%.0ffps", diff);
		cvPutText(frame, fps, cvPoint(10, 20), &font, CV_RGB(0,200,0));
		cvShowImage("Camera video feed", frame);

		//If ESC key pressed
		int key = (cvWaitKey(10) & 255);
		if (key == 27)
			break;
		//If 's' key pressed
		if (key == 's') {
			cvhSaveJPEG("out.jpg", frame, 100);
		}

	}
// Release the capture device housekeeping
	cvReleaseCapture(&capture);
}

