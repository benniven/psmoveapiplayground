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

int main(int arg, char** args) {
	/*
	 HKEY hKey;
	 char* dst = "\\Software\\PS3EyeCamera\\Settings";
	 RegOpenKeyEx(HKEY_CURRENT_USER, dst, NULL, KEY_READ, &hKey);
	 RegQueryValueEx(hKey, "Start Page", NULL, NULL, (LPBYTE) lpData,
	 &buffersize);
	 */
	//diff();
	videoHist(arg, args);
	//videoGuess(arg, args);
	/*CvScalar moveColor = CV_RGB(0,0,0);
	 findMoveColorP("../out1.jpg", "../out2.jpg", &moveColor);
	 printScalar("col: ", &moveColor, "\n");
	 waitForESC();

	 CvScalar moveColor = findOptimalMoveColorP("../out1.jpg");
	 printScalar("col: ", &moveColor, "\n");
	 waitForESC();
	 */
	return 0;
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

