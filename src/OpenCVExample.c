#include <stdio.h>
#include <time.h>
#include <unistd.h>

#ifdef WIN32
#    include <windows.h>
#endif

#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"

#include "tracker_helpers.h"
#include "OpenCVMoveAPI.h"
#include "high_precision_timer.h"

#include "psmove.h"
#include "psmove_tracker.h"

/* Define which camera to use (zero-based index or CV_CAP_ANY) */
#define CAM_TO_USE 1

void calibrate();
void videoHist();
void autoWB();
PSMove* connectController();

int main(int arg, char** args) {
	calibrate();
	// autoWB();
	// videoHist();
	return 0;
}

void calibrate() {
	PSMove* controller = connectController();
	PSMoveTracker* tracker = psmove_tracker_new();
	HPTimer* timer = hp_timer_create();
	CvCapture* capture = cvCaptureFromCAM(CAM_TO_USE);
	IplImage* frame;
	CvScalar c;
	CvScalar ic;
	float fps = 30;
	char text[256];
	unsigned char r, g, b;
	while (1) {
		int erg = psmove_tracker_enable(tracker, controller);
		if (erg == Tracker_CALIBRATED) {
			psmove_tracker_get_color(tracker, controller, &r, &g, &b);
			c.val[0] = b;
			c.val[1] = g;
			c.val[2] = r;
			ic = cvScalar(0xff - c.val[0], 0xff - c.val[1], 0xff - c.val[2], 0x0);
			break;
		} else {
			cvNamedWindow("unable to calibrate (ESC to retry)", 0);
			th_wait_esc();
		}
	}

	float avgLum = 0;
	CvScalar avgC;
	CvPoint p;

	// this seems to be a nice color
	while (1) {
		int key = (cvWaitKey(10) & 255);

		psmove_tracker_update_image(tracker);
		frame = psmove_tracker_get_image(tracker);
		if (!frame)
			continue;

		psmove_set_leds(controller, r, g, b);
		psmove_update_leds(controller);

		// this is the tracking algo
		hp_timer_start(timer);
		psmove_tracker_update(tracker, controller);
		psmove_tracker_get_position(tracker, controller, &p.x, &p.y, 0x0);
		hp_timer_stop(timer);

		//cvRectangle(frame, cvPoint(roi.x, roi.y), cvPoint(roi.x + roi.width, roi.y + roi.height), th_white, 3, 8, 0);
		//cvRectangle(frame, cvPoint(roi.x, roi.y), cvPoint(roi.x + roi.width, roi.y + roi.height), th_red, 1, 8, 0);

		fps = 0.85 * fps + 0.15 * (1.0 / hp_timer_get_seconds(timer));

		cvRectangle(frame, cvPoint(0, 0), cvPoint(frame->width, 25), th_black, CV_FILLED, 8, 0);
		sprintf(text, "fps:%.0f", fps);
		th_put_text(frame, text, cvPoint(10, 20), c);

		sprintf(text, "RGB:%x,%x,%x", (int) c.val[2], (int) c.val[1], (int) c.val[0]);
		th_put_text(frame, text, cvPoint(110, 20), c);

		avgC = cvAvg(frame, 0x0);
		avgLum = th_avg(avgC.val, 3);
		sprintf(text, "avg(lum):%.0f", avgLum);
		th_put_text(frame, text, cvPoint(255, 20), c);

		//sprintf(text, "exp:%d", exp);
		//th_put_text(frame, text, cvPoint(400, 20), c);

		//sprintf(text, "ROI:%dx%d", roi.width, roi.height);
		//th_put_text(frame, text, cvPoint(505, 20), c);

		cvCircle(frame, p, 4, th_black, 4, 8, 0);
		cvCircle(frame, p, 4, ic, 2, 8, 0);
		cvShowImage("live camera feed", frame);
		//If ESC key pressed
		if (key == 27)
			break;
	}
	hp_timer_release(timer);
	psmove_disconnect(controller);
	cvReleaseCapture(&capture);
}

/*void tracker_get_diff(CvCapture** capture, PSMove* controller, int exp, IplImage* on, IplImage* diff) {
 int delay = 1000000 / 4;
 IplImage* frame;
 if (exp >= 0) {
 cvReleaseCapture(capture);
 th_set_camera_params(0, 0, 1, exp, 0, -1, -1, -1);
 usleep(delay);
 cvCaptureFromCAM(CAM_TO_USE);
 }
 psmove_set_leds(controller, BCr, BCg, BCb);
 psmove_update_leds(controller);
 usleep(delay);
 frame = th_query_frame(*capture);
 // copy the color picture!
 cvCopy(frame, on, 0x0);

 psmove_set_leds(controller, 0, 0, 0);
 psmove_update_leds(controller);
 usleep(delay);
 frame = th_query_frame(*capture);

 IplImage* grey1 = cvCloneImage(diff);
 IplImage* grey2 = cvCloneImage(diff);

 cvCvtColor(frame, grey1, CV_BGR2GRAY);
 cvCvtColor(on, grey2, CV_BGR2GRAY);

 // calculate the diff of these two cleaned up images
 cvAbsDiff(grey1, grey2, diff);

 cvReleaseImage(&grey1);
 cvReleaseImage(&grey2);
 }*/

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

void autoWB() {
	int gain = 0;
	int exp = 0x10;		//tracker_adapt_to_light(20, 0x10, 0x18);

	th_set_camera_params(0, 0, 0, exp, gain, 0xff, 0xff, 0xff);

	CvCapture* capture = cvCaptureFromCAM(CAM_TO_USE);

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
		frame = th_query_frame(capture);

		if (!frame)
			continue;

		th_create_image(&hsv, cvGetSize(frame), frame->depth, frame->nChannels);
		th_create_image(&h_plane, cvGetSize(frame), frame->depth, 1);
		th_create_image(&s_plane, cvGetSize(frame), frame->depth, 1);
		th_create_image(&v_plane, cvGetSize(frame), frame->depth, 1);

		cvCvtColor(frame, hsv, CV_BGR2HSV);
		cvSplit(hsv, h_plane, s_plane, v_plane, 0);

		cvCalcHist(&v_plane, ahist, 0, 0); // Compute histogram
		cvNormalizeHist(ahist, 255); // Normalize it
		th_plot_hist(ahist, h_bins, "Value Histogram", th_white);

		avgColor = cvAvg(frame, 0x0);
		sprintf(text, "Exposure: %d (0x%x)", exp, exp);
		th_put_text(frame, text, cvPoint(10, 20), CV_RGB(0,200,0));
		sprintf(text, "Gain: %d (0x%x)", gain, gain);
		th_put_text(frame, text, cvPoint(10, 40), CV_RGB(0,200,0));
		sprintf(text, "Avg Lum=%.0f", th_avg(avgColor.val, 3));
		th_put_text(frame, text, cvPoint(10, 60), CV_RGB(0,200,0));

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
			th_set_camera_params(0, 0, 0, exp, gain, 0xff, 0xff, 0xff);
			cvReleaseCapture(&capture);
			capture = cvCaptureFromCAM(CAM_TO_USE);
			usleep(10000);
			GATE = 1;
		}

	}
	// Release the capture device housekeeping
	cvReleaseCapture(&capture);
}

void videoHist() {

	HPTimer* timer = hp_timer_create();
	CvCapture* capture = cvCaptureFromCAM(CAM_TO_USE);

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
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC, hScale, vScale, 0, lineWidth, CV_AA);

	// Create a window in which the captured images will be presented
	// Show the image captured from the camera in the window and repeat
	float diff = 0;
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

		hp_timer_start(timer);
		findOptimalMoveColors(frame, 64, 5, bestColors, 2);
		hp_timer_stop(timer);
		diff = 0.85 * diff + 0.15 * (1.0 / hp_timer_get_seconds(timer));

		sprintf(fps, "@%.0ffps", diff);
		cvPutText(frame, fps, cvPoint(10, 20), &font, CV_RGB(0,200,0));
		cvShowImage("Camera video feed", frame);

		//If ESC key pressed
		int key = (cvWaitKey(10) & 255);
		if (key == 27)
			break;
		//If 's' key pressed
		if (key == 's') {
			th_save_jpg("out.jpg", frame, 100);
		}

	}
	// Release the capture device housekeeping
	cvReleaseCapture(&capture);
}

