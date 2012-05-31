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

void calibrate();
void getDiff(CvCapture** capture, PSMove* controller, int exp, IplImage* on,
		IplImage* diff);
void videoHist(int arg, char** args);
void videoGuess(int arg, char** args);
void diff();
void adaptToLighting();
PSMove* connectController();

int main(int arg, char** args) {

	//cvhBackupCLDriverRegistry();
	calibrate();
	//adaptToLighting();
	//diff();
	//videoHist(arg, args);
	//videoHist(arg, args);
	//cvhWaitForChar('p');
	//videoHist(arg, args);
	//videoGuess(arg, args);/
	//cvhRestoreCLDriverRegistry();
	return 0;
}

void calibrate() {
	//cvNamedWindow("calibration ....", 1);
	PSMove* controller = connectController();
	cvhSetCameraParameters(0, 0, -1, 16, 0, -1, -1, -1);
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	if (!capture) {
		fprintf(stderr, "ERROR: capture is NULL \n");
		getchar();
	}

	int i;
	int blinks = 3;
	IplImage* images[blinks];
	IplImage* diffs[blinks];
	IplImage* frame = cvQueryFrame(capture);
	for (i = 0; i < blinks; i++) {
		images[i] = cvCreateImage(cvGetSize(frame), frame->depth,
				frame->nChannels);
		diffs[i] = cvCreateImage(cvGetSize(frame), frame->depth, 1);
	}
	CvScalar c;
	while (1) {
		// wait until the user presses a controller button
		cvhWaitMoveButton(controller, Btn_T);
		for (i = 0; i < blinks; i++) {
			getDiff(&capture, controller, -1, images[i], diffs[i]);
			double max;
			// TODO: sind das gute werte?
			cvMinMaxLoc(diffs[i], 0x0, &max, 0x0, 0x0, 0x0);
			cvThreshold(diffs[i], diffs[i], max / 3, 0xFF, CV_THRESH_BINARY);
		}
		// put the diff images together!
		for (i = 1; i < blinks; i++) {
			cvAnd(diffs[0], diffs[i], diffs[0], 0x0);
		}
		// calculate the avg color!
		c = cvAvg(images[blinks - 1], diffs[0]);

		// if the color is realy a color and not something white/black/greyisch
		if (cvhAvgF(c.val, 3) > 40)
			break;
	}

	// this seems to be a nice color
	while (1) {
		frame = cvhQueryImage(capture);
		if (!frame)
			continue;

		IplImage* mask = cvCreateImage(cvGetSize(frame), frame->depth, 1);
		IplImage* hsvFrame = cvCreateImage(cvGetSize(frame), frame->depth,
				frame->nChannels);

		cvCvtColor(images[blinks - 1], hsvFrame, CV_BGR2HSV);
		CvScalar hc = cvAvg(hsvFrame, diffs[0]);

		cvCvtColor(frame, hsvFrame, CV_BGR2HSV);

		CvScalar min = cvScalar(hc.val[0] - 15, hc.val[1] - 35, hc.val[2] - 35,
				0);
		CvScalar max = cvScalar(hc.val[0] + 15, hc.val[1] + 35, hc.val[2] + 35,
				0);

		// find the blob
		cvInRangeS(hsvFrame, min, max, mask);

		// blurr it a little
		//cvSmooth(mask, mask, CV_MEDIAN, 13, 13, 0, 0);

		cvShowImage("live camera feed", frame);
		cvSet(frame, c, mask);

		cvShowImage("color seperated", mask);

		psmove_set_leds(controller, 0xFF, 0xFF, 0);
		psmove_update_leds(controller);

		//If ESC key pressed
		int key = (cvWaitKey(10) & 255);
		if (key == 27)
			break;
	}
	psmove_disconnect(controller);
	cvReleaseCapture(&capture);
}
void getDiff(CvCapture** capture, PSMove* controller, int exp, IplImage* on,
		IplImage* diff) {
	int delay = 1000000 / 2;
	IplImage* frame;
	if (exp >= 0) {
		cvReleaseCapture(capture);
		cvhSetCameraParameters(0, 0, 1, exp, 0, -1, -1, -1);
		usleep(delay);
		cvCaptureFromCAM(CV_CAP_ANY);
	}
	psmove_set_leds(controller, 0xFF, 0xFF, 0);
	psmove_update_leds(controller);
	usleep(delay);
	frame = cvQueryFrame(*capture);
	frame = cvQueryFrame(*capture);
	// copy the color picture!
	cvCopy(frame, on, 0x0);

	psmove_set_leds(controller, 0, 0, 0);
	psmove_update_leds(controller);
	usleep(delay);
	frame = cvQueryFrame(*capture);
	frame = cvQueryFrame(*capture);

	IplImage* grey1 = cvCloneImage(diff);
	IplImage* grey2 = cvCloneImage(diff);
	IplImage* mask1 = cvCloneImage(diff);
	IplImage* mask2 = cvCloneImage(diff);

	cvCvtColor(frame, grey1, CV_BGR2GRAY);
	cvCvtColor(on, grey2, CV_BGR2GRAY);

	// find all white areas
	CvScalar u = cvScalarAll(220);
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

void adaptToLighting() {

	cvNamedWindow("videoGuess", 1);
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);

	if (!capture) {
		fprintf(stderr, "ERROR: capture is NULL \n");
		getchar();
	}

	IplImage* frame;
	IplImage* ranged;

	CvFont font;
	CvScalar avgColor;
	double hScale = 0.5;
	double vScale = 0.5;
	int lineWidth = 1;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC, hScale, vScale,
			0, lineWidth, CV_AA);

	int gain = 0;
	int exp = 0x1FF / 2;
	char text[512];
	int GATE = 1;
	cvhSetCameraParameters(0, 0, -1, exp, gain, -1, -1, -1);
	while (1) {
		frame = cvhQueryImage(capture);
		if (!frame)
			continue;
		cvhCreateImage(&ranged, cvGetSize(frame), frame->depth, 1);
		avgColor = cvAvg(frame, 0x0);
		sprintf(text, "Exposure: %d (0x%x)", exp, exp);
		cvPutText(frame, text, cvPoint(10, 20), &font, CV_RGB(0,200,0));
		//sprintf(text, "Gain: %d (0x%x)", gain, gain);
		//cvPutText(frame, text, cvPoint(10, 40), &font, CV_RGB(0,200,0));
		sprintf(text, "Avg BGR Color=(%.0f,%.0f,%.0f)", avgColor.val[0],
				avgColor.val[1], avgColor.val[2]);
		cvPutText(frame, text, cvPoint(10, 60), &font, CV_RGB(0,200,0));

		cvShowImage("live camera feed", frame);
		CvScalar min = cvScalar(234, 234, 234, 0);
		CvScalar max = cvScalar(255, 255, 255, 0);

		cvInRangeS(frame, min, max, ranged);
		cvShowImage("very bright areas", ranged);

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
			cvhSetCameraParameters(0, 0, 1, exp, gain, -1, -1, -1);
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

	int medK = cvhOddKernel(img1->width / 100);

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

void videoGuess(int arg, char** args) {
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

	IplImage* frame1 = 0x0;
	IplImage* frame2 = 0x0;

	IplImage* cap;

	while (1) {

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
		cvCopy(cap, frame2, 0x0);

		findMoveColor(frame1, frame2, 0, 0);
	}
	// Release the capture device housekeeping
	cvReleaseCapture(&capture);
}

void videoHist(int arg, char** args) {

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
	float diff;
	int tic, toc;
	while (1) {
		// Get one frame
		tic = clock();
		IplImage* frame = cvQueryFrame(capture);
		//image = frame;
		if (!frame) {
			fprintf(stderr, "ERROR: frame is null...\n");
			getchar();
			break;
		}

		CvScalar bestColors[2];
		findOptimalMoveColors(frame, 64, 5, bestColors, 2);

		toc = clock();
		diff = 0.0 * diff + 1.0
				* (1 / (((float) (toc - tic)) / CLOCKS_PER_SEC));
		char fps[256];
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

