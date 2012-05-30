#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <windows.h>

#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"

#include "OpenCVHelpers.h"
#include "OpenCVMoveAPI.h"

void videoHist(int arg, char** args);
void videoGuess(int arg, char** args);
void diff();
void adaptToLighting();

int main(int arg, char** args) {
	cvhBackupCLDriverRegistry();
	adaptToLighting();
	//diff();
	//videoHist(arg, args);
	//videoHist(arg, args);
	//cvhWaitForChar('p');
	//videoHist(arg, args);
	//videoGuess(arg, args);/
	cvhRestoreCLDriverRegistry();
	return 0;
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
	int i = 0;
	cvhSetCameraParameters(0, 0, 1, exp, gain, -1, -1, -1);
	while (1) {
		frame = cvhQueryImage(capture);
		if (!frame)
			continue;
		cvhCreateImage(&ranged, cvGetSize(frame), frame->depth, 1);
		avgColor = cvAvg(frame, 0x0);
		sprintf(text, "Exposure: %d (0x%x)", exp, exp);
		cvPutText(frame, text, cvPoint(10, 20), &font, CV_RGB(0,200,0));
		sprintf(text, "Gain: %d (0x%x)", gain, gain);
		cvPutText(frame, text, cvPoint(10, 40), &font, CV_RGB(0,200,0));
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
			i++;
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
		diff = 0.8 * diff
				+ 0.2 * (1 / (((float) (toc - tic)) / CLOCKS_PER_SEC));
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

