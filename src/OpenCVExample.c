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

/* Define which camera to use (zero-based index or CV_CAP_ANY) */
#define CAM_TO_USE 1

void calibrate();
void tracker_get_diff(CvCapture** capture, PSMove* controller, int exp,
		IplImage* on, IplImage* diff);

void videoHist();
int adaptToLighting(int lumMin, int expMin, int expMax);
void autoWB();
PSMove* connectController();

int BCr;
int BCg;
int BCb;

int main(int arg, char** args) {
	calibrate();
	// autoWB();
	// videoHist();
	return 0;
}

void calibrate() {
	PSMove* controller = connectController();
	HPTimer* timer = hp_timer_create();
	int exp = adaptToLighting(25, 0x10, 0x40);
	float f = 1;

	if (exp > 20)
		f = 0.7;
	if (exp > 30)
		f = 0.5;

	BCr = 0x00 * f;
	BCg = 0x00 * f;
	BCb = 0xFF * f;

	th_set_camera_params(0, 0, 0, exp, 0, 0xff, 0xff, 0xff);
	CvCapture* capture = cvCaptureFromCAM(CAM_TO_USE);
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
	IplConvKernel* cK = cvCreateStructuringElementEx(11, 11, 6, 6,
			CV_SHAPE_RECT, 0x0);
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
	float fps = 30;
	char text[256];
	while (1) {
		// wait until the user presses a controller button
		//cvhWaitMoveButton(controller, Btn_T);
		for (i = 0; i < blinks; i++) {
			tracker_get_diff(&capture, controller, -1, images[i], diffs[i]);

			cvThreshold(diffs[i], diffs[i], 20, 0xFF, CV_THRESH_BINARY);

			cvErode(diffs[i], diffs[i], cK, 1);
			cvDilate(diffs[i], diffs[i], cK, 1);
		}
		// put the diff images together!
		for (i = 1; i < blinks; i++) {
			cvAnd(diffs[0], diffs[i], diffs[0], 0x0);
		}

		// calculate the avg color!
		c = cvAvg(images[blinks - 1], diffs[0]);
		ic = cvScalar(0xff - c.val[0], 0xff - c.val[1], 0xff - c.val[1], 0);
		hc = th_brg2hsv(c);
		// find a single contour with that color within the calibration pictures
		th_minus(hc.val, r.val, min.val, 3);
		th_plus(hc.val, r.val, max.val, 3);
		for (i = 0; i < blinks; i++) {
			cvCvtColor(images[i], hsvFrame, CV_BGR2HSV);
			cvInRangeS(hsvFrame, min, max, mask);

			cvErode(mask, mask, 0x0, 1);
			cvDilate(mask, mask, 0x0, 1);

			CvSeq* contour;
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
		// and if there were found contours at all
		double stdSizes = sqrt(th_var(sizes, blinks));
		if (stdSizes < (th_avg(sizes, blinks) * 0.10) && sizes[0] > 0.0)
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
		roiI[i] = cvCreateImage(cvSize(w * 0.6, w * 0.6), z->depth,
				z->nChannels);
		roiM[i] = cvCreateImage(cvSize(w * 0.6, w * 0.6), z->depth, 1);
	}

	th_minus(hc.val, r.val, min.val, 3);
	th_plus(hc.val, r.val, max.val, 3);
	float avgLum = 0;
	CvScalar avgC;
	// this seems to be a nice color
	while (1) {
		int key = (cvWaitKey(10) & 255);
		frame = th_query_frame(capture);

		if (!frame)
			continue;
		CvPoint p;

		// this is the tracking algo
		hp_timer_start(timer);
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
			cvSet(roiM[0], th_black, 0x0);
			cvSet(roiM[roiIdx], th_black, 0x0);
			if (best) {
				cvDrawContours(roiM[roiIdx], best, th_white, th_white, -1,
						CV_FILLED, 8, cvPoint(0, 0));

				cvDrawContours(roiM[0], best, th_white, th_white, -1, CV_FILLED,
						8, cvPoint(roi.x, roi.y));

				CvMoments mu;
				cvMoments(roiM[roiIdx], &mu, 0);
				p = cvPoint(mu.m10 / mu.m00, mu.m01 / mu.m00);
				p.x = p.x + roi.x;
				p.y = p.y + roi.y;

				// update the feature roi box!
				br = cvBoundingRect(best, 0);
				br.width = th_max(br.width, br.height) * 2;
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
		hp_timer_stop(timer);

		cvRectangle(frame, cvPoint(roi.x, roi.y),
				cvPoint(roi.x + roi.width, roi.y + roi.height), th_white, 3, 8,
				0);
		cvRectangle(frame, cvPoint(roi.x, roi.y),
				cvPoint(roi.x + roi.width, roi.y + roi.height), th_red, 1, 8,
				0);

		cvShowImage("mask", roiM[0]);

		fps = 0.85 * fps + 0.15 * (1.0 / hp_timer_get_seconds(timer));

		cvRectangle(frame, cvPoint(0, 0), cvPoint(frame->width, 25), th_black,
				CV_FILLED, 8, 0);
		sprintf(text, "fps:%.0f", fps);
		th_put_text(frame, text, cvPoint(10, 20), c);

		sprintf(text, "RGB:%x,%x,%x", (int) c.val[2], (int) c.val[1],
				(int) c.val[1]);
		th_put_text(frame, text, cvPoint(110, 20), c);

		avgC = cvAvg(frame, 0x0);
		avgLum = th_avg(avgC.val, 3);
		sprintf(text, "avg(lum):%.0f", avgLum);
		th_put_text(frame, text, cvPoint(255, 20), c);

		sprintf(text, "exp:%d", exp);
		th_put_text(frame, text, cvPoint(400, 20), c);

		sprintf(text, "ROI:%dx%d", roi.width, roi.height);
		th_put_text(frame, text, cvPoint(505, 20), c);

		cvCircle(frame, p, 4, th_black, 4, 8, 0);
		cvCircle(frame, p, 4, ic, 2, 8, 0);
		cvShowImage("live camera feed", frame);
		if (th_move_button(controller, Btn_T))
			psmove_set_leds(controller, 0, 0, 0);
		else
			psmove_set_leds(controller, BCr, BCg, BCb);
		psmove_update_leds(controller);
		//If ESC key pressed
		if (key == 27)
			break;
	}
	hp_timer_release(timer);
	psmove_disconnect(controller);
	cvReleaseCapture(&capture);
}

void tracker_get_diff(CvCapture** capture, PSMove* controller, int exp,
		IplImage* on, IplImage* diff) {
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
	th_set_camera_params(0, 0, 0, exp, 0, 0xff, 0xff, 0xff);

	CvCapture* capture = cvCaptureFromCAM(CAM_TO_USE);

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
		frame = th_query_frame(capture);

		if (!frame)
			continue;

		CvScalar avgColor = cvAvg(frame, 0x0);
		float avgLum = th_avg(avgColor.val, 3);
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
				th_set_camera_params(0, 0, 0, exp, 0, 0xff, 0xff, 0xff);
				cvReleaseCapture(&capture);
				capture = cvCaptureFromCAM(CAM_TO_USE);
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
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC, hScale, vScale,
			0, lineWidth, CV_AA);

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

