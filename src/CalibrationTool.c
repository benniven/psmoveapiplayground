#include <stdio.h>

#ifdef WIN32
#    include <windows.h>
#endif

#include "./camera/camera_control.h"
#include "./tracker/tracker_helpers.h"

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"

#define CAM_TO_USE 0

int Xmain(int arg, char** args) {
	int board_w = 4; // Board width in squares
	int board_h = 7; // Board height
	int n_boards = 5; // Number of boards
	int skip_n = 60; // skip every n frames gives user time to adjust the checkboard
	int board_n = board_w * board_h;
	CvSize board_sz = cvSize(board_w, board_h);
	CameraControl* cc = camera_control_new(0);

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
	int frame = 0;
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
		// Skp every skip_n frames to allow user to move chessboard
		// skip a second to allow user to move the chessboard
		image = camera_control_query_frame(cc); // Get next image
		cvCvtColor(image, gray_image, CV_BGR2GRAY);
		if (frame++ % skip_n == 0) {
			cvWaitKey(1);
			corner_count = 0;

			// Find chessboard corners:
			int found = cvFindChessboardCorners(gray_image, board_sz, corners, &corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

			// Get subpixel accuracy on those corners
			cvFindCornerSubPix(gray_image, corners, corner_count, cvSize(11, 11), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			// Draw it
			cvDrawChessboardCorners(image, board_sz, corners, corner_count, found);

			char text[222];
			sprintf(text, "calibration image %d/%d", successes, n_boards);
			th_put_text(image, text, cvPoint(20, 20), th_white, 1.0);
			cvShowImage("Calibration", image);

			// If we got a good board, add it to our data
			if (corner_count == board_n) {
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
	while (image) {
		IplImage *t = cvCloneImage(image);
		cvShowImage("Calibration", image); // Show raw image
		cvRemap(t, image, mapx, mapy, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(0)); // undistort image
		cvReleaseImage(&t);
		cvShowImage("Undistorted", image); // Show corrected image

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
	return 0;
}

