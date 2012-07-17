#include <stdio.h>
#include <time.h>
#include <unistd.h>

#ifdef WIN32
#    include <windows.h>
#endif

#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"

#include "opencv2/calib3d/calib3d.hpp"

#include "tracker_helpers.h"
#include "OpenCVMoveAPI.h"
#include "high_precision_timer.h"

#include "psmove.h"
#include "psmove_tracker.h"
#include "camera/camera_control.h"

/* Define which camera to use (zero-based index or CV_CAP_ANY) */
#define CAM_TO_USE 1

void startTracker();
void videoHist();
void autoWB();

void cameraCalibration();

PSMove* connectController();

int main(int arg, char** args) {
	startTracker();
	//cameraCalibration();
	return 0;
}

int n_boards = 0;
int board_w;
int board_h;

void cameraCalibration() {
	board_w = 5; // Board width in squares
	board_h = 8; // Board height
	n_boards = 8; // Number of boards
	int board_n = board_w * board_h;
	CvSize board_sz = cvSize(board_w, board_h);
	CameraControl* cc = camera_control_new();


	//cvNamedWindow("Calibration", 0);
	// Allocate Sotrage
	CvMat* image_points = cvCreateMat(n_boards * board_n, 2, CV_32FC1);
	CvMat* object_points = cvCreateMat(n_boards * board_n, 3, CV_32FC1);
	CvMat* point_counts = cvCreateMat(n_boards, 1, CV_32SC1);
	CvMat* intrinsic_matrix = cvCreateMat(3, 3, CV_32FC1);
	CvMat* distortion_coeffs = cvCreateMat(5, 1, CV_32FC1);
	IplImage *image;

	CvPoint2D32f corners[board_n];
	int i = 0;
	int j = 0;

	for (i = 0; i < board_n; i++)
		corners[i] = cvPoint2D32f(0, 0);

	int corner_count;
	int successes = 0;
	int step = 0;



	while (1) {
		cvWaitKey(10);
		image = camera_control_query_frame(cc);
		if (image)
			break;
	}
	IplImage *gray_image = cvCreateImage(cvGetSize(image), 8, 1);

	// Capture Corner views loop until we've got n_boards
	// succesful captures (all corners on the board are found)
	while (successes < n_boards) {
		// Skp every board_dt frames to allow user to move chessboard
		// skip a second to allow user to move the chessboard
		image = camera_control_query_frame(cc); // Get next image
		//if (frame++ % board_dt == 0)
		{
			// Find chessboard corners:
			int found = cvFindChessboardCorners(image, board_sz, corners, &corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
			int key = cvWaitKey(1);

			// Get subpixel accuracy on those corners
			cvCvtColor(image, gray_image, CV_BGR2GRAY);
			cvFindCornerSubPix(gray_image, corners, corner_count, cvSize(11, 11), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

			// Draw it
			cvDrawChessboardCorners(image, board_sz, corners, corner_count, found);
			char text[222];
			sprintf(text, "calibration image %d/%d", successes, n_boards);
			th_put_text(image, text, cvPoint(20, 20), th_white, 1.0);
			cvShowImage("Calibration", image);

			// If we got a good board, add it to our data
			if (corner_count == board_n ) {
				step = successes * board_n;
				for (i = step, j = 0; j < board_n; ++i, ++j) {
					CV_MAT_ELEM( *image_points, float, i, 0 ) = corners[j].x;
					CV_MAT_ELEM( *image_points, float, i, 1 ) = corners[j].y;
					CV_MAT_ELEM( *object_points, float, i, 0 ) = j / board_w;
					CV_MAT_ELEM( *object_points, float, i, 1 ) = j % board_w;
					CV_MAT_ELEM( *object_points, float, i, 2 ) = 0.0f;
				}
				CV_MAT_ELEM( *point_counts, int, successes, 0 ) = board_n;
				successes++;
			}

		}
	}

	// Allocate matrices according to how many chessboards found
	CvMat* object_points2 = cvCreateMat(successes * board_n, 3, CV_32FC1);
	CvMat* image_points2 = cvCreateMat(successes * board_n, 2, CV_32FC1);
	CvMat* point_counts2 = cvCreateMat(successes, 1, CV_32SC1);

	// Transfer the points into the correct size matrices
	for (i = 0; i < successes * board_n; ++i) {
		CV_MAT_ELEM( *image_points2, float, i, 0) = CV_MAT_ELEM( *image_points, float, i, 0 );
		CV_MAT_ELEM( *image_points2, float, i, 1) = CV_MAT_ELEM( *image_points, float, i, 1 );
		CV_MAT_ELEM( *object_points2, float, i, 0) = CV_MAT_ELEM( *object_points, float, i, 0 );
		CV_MAT_ELEM( *object_points2, float, i, 1) = CV_MAT_ELEM( *object_points, float, i, 1 );
		CV_MAT_ELEM( *object_points2, float, i, 2) = CV_MAT_ELEM( *object_points, float, i, 2 );
	}

	for (i = 0; i < successes; ++i) {
		CV_MAT_ELEM( *point_counts2, int, i, 0 ) = CV_MAT_ELEM( *point_counts, int, i, 0 );
	}
	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
	cvReleaseMat(&point_counts);

	// At this point we have all the chessboard corners we need
	// Initiliazie the intrinsic matrix such that the two focal lengths
	// have a ratio of 1.0

	CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0;
	CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0;

	// Calibrate the camera
	CvTermCriteria default_termination = cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, DBL_EPSILON);
	cvCalibrateCamera2(object_points2, image_points2, point_counts2, cvGetSize(image), intrinsic_matrix, distortion_coeffs, NULL, NULL,
			CV_CALIB_FIX_ASPECT_RATIO, default_termination);

	// Save the intrinsics and distortions
	CvAttrList empty_attribues = cvAttrList(0, 0);
	cvSave("Intrinsics.xml", intrinsic_matrix, 0, 0, empty_attribues);
	cvSave("Distortion.xml", distortion_coeffs, 0, 0, empty_attribues);

	// Example of loading these matrices back in
	CvMat *intrinsic = (CvMat*) cvLoad("Intrinsics.xml", 0, 0, 0);
	CvMat *distortion = (CvMat*) cvLoad("Distortion.xml", 0, 0, 0);

	image = camera_control_query_frame(cc);

	// Build the undistort map that we will use for all subsequent frames
	IplImage* mapx = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
	IplImage* mapy = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
	cvInitUndistortMap(intrinsic, distortion, mapx, mapy);

	// Run the camera to the screen, now showing the raw and undistorted image
	//cvNamedWindow("Undistort", 0);

	//http://www.ovt.com/download_document.php?type=sensor&sensorid=80
	float fovX, fovY, f, ar;
	CvPoint2D64f p;
	// 3948,2952
	cvCalibrationMatrixValues(intrinsic, cvGetSize(image), 0, 0, &fovX, &fovY, &f, &p, &ar);

	printf("fovX: %.2f°\n", fovX);
	printf("fovY: %.2f°\n", fovY);
	printf("f: %.2f°\n", f);
	printf("pp: %.2f/%.2f°\n", p.x, p.y);
	printf("aspect ration: %.2f°\n", ar);

	while (image) {
		IplImage *t = cvCloneImage(image);
		cvShowImage("Calibration", image); // Show raw image
		cvRemap(t, image, mapx, mapy, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(0)); // undistort image
		cvReleaseImage(&t);
		cvShowImage("Undistort", image); // Show corrected image

		// Handle pause/unpause and esc
		int c = cvWaitKey(15);
		if (c == 'p') {
			c = 0;
			while (c != 'p' && c != 27) {
				c = cvWaitKey(250);
			}
		}
		if (c == 27)
			break;
		image = camera_control_query_frame(cc);
	}
}

void startTracker() {
	int i;
	int numCtrls = psmove_count_connected();
	PSMove* controllers[numCtrls];

	printf("%s", "### Trying to init PSMoveTracker...");
	PSMoveTracker* tracker = psmove_tracker_new();
	printf("%s\n", "OK");
	printf("### Found %d controllers.\n", numCtrls);

	IplImage* frame;
	unsigned char r, g, b;
	int erg;

	for (i = 0; i < numCtrls; i++) {
		printf("### Trying to init Controller(%d)...\n", i);
		controllers[i] = connectController(i);
		printf("### Controller(%d): OK\n", i);

		while (1) {
			printf("### Trying to calibrate controller(%d)...", i);
			erg = psmove_tracker_enable(tracker, controllers[i]);
			if (erg == Tracker_CALIBRATED) {
				printf("%s\n", "OK");
				break;
			} else {
				printf("%s\n", "ERROR");
				printf("--> Unable to calibrate controllers %d see 'debug.html' for details.\n", i);
				cvNamedWindow("Unable to calibrate (ESC to retry)", 0);

				th_wait_esc();
			}
		}

	}
	printf("### All Controllers enabled.\n");
	// this seems to be a nice color
	while (1) {
		int key = (cvWaitKey(10) & 255);

		psmove_tracker_update_image(tracker);
		frame = psmove_tracker_get_image(tracker);
		if (!frame)
			continue;
		for (i = 0; i < numCtrls; i++) {
			psmove_tracker_get_color(tracker, controllers[i], &r, &g, &b);
			psmove_set_leds(controllers[i], r, g, b);
			psmove_update_leds(controllers[i]);

			psmove_tracker_get_position(tracker, controllers[i], 0, 0, 0);
		}

		psmove_tracker_update(tracker, 0x0);
		cvShowImage("live camera feed", frame);
		//If ESC key pressed
		if (key == 27)
			break;
	}
	for (i = 0; i < numCtrls; i++) {
		psmove_disconnect(controllers[i]);
	}
}

PSMove* connectController(int id) {
	PSMove *move;
	enum PSMove_Connection_Type ctype;
	int i;

	move = psmove_connect_by_id(id);

	if (move == NULL) {
		printf("Could not connect to default Move controller.\n"
				"Please connect one via USB or Bluetooth.\n");
		exit(1);
	}

	ctype = psmove_connection_type(move);
#ifndef WIN32
	// TODO: wieso geht das nicht onne (unter linux)
	ctype = Conn_USB;
#endif

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
	int exp = 0x10; //tracker_adapt_to_light(20, 0x10, 0x18);

	// TODO: Camera Control
	//th_set_camera_params(0, 0, 0, exp, gain, 0xff, 0xff, 0xff);

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
		// TODO: camera control
		//frame = th_query_frame(capture);

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
		th_put_text(frame, text, cvPoint(10, 20), CV_RGB(0,200,0), 0.5);
		sprintf(text, "Gain: %d (0x%x)", gain, gain);
		th_put_text(frame, text, cvPoint(10, 40), CV_RGB(0,200,0), 0.5);
		sprintf(text, "Avg Lum=%.0f", th_avg(avgColor.val, 3));
		th_put_text(frame, text, cvPoint(10, 60), CV_RGB(0,200,0), 0.5);

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
			// TODO: Camera Control
			//th_set_camera_params(0, 0, 0, exp, gain, 0xff, 0xff, 0xff);
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

