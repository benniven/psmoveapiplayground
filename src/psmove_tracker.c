/**
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2012 Thomas Perl <m@thp.io>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "opencv2/core/core_c.h"
#include "opencv2/imgproc/imgproc_c.h"

#include "psmove_tracker.h"
#include "tracker_helpers.h"
#include "tracked_controller.h"
#include "tracked_color.h"
#include "high_precision_timer.h"
#include "tracker_trace.h"

#define BLINKS 4                 // number of diff images to create during calibration
#define ROIS 5                   // the number of levels of regions of interest (roi)
#define CALIB_MIN_SIZE 100		 // minimum size of the estimated glowing sphere during calibration process
#define CALIB_SIZE_STD 10	     // maximum standard deviation (in %) of the glowing spheres found during calibration process
struct _PSMoveTracker {
	int cam; // the camera to use
	CvCapture* capture; // the camera device opened for capture
	IplImage* frame; // the current frame of the camera
	int exposure; // the exposure to use
	IplImage* roiI[ROIS]; // array of images for each level of roi (colored)
	IplImage* roiM[ROIS]; // array of images for each level of roi (greyscale)
	CvMemStorage* storage; // data structure used to hold contours found by "cvFindContours"
	IplConvKernel* kCalib; // kernel used for morphological operations during calibration
	IplConvKernel* kTrack; // kernel used for morphological operations during tracking
	CvScalar rHSV; // the range of the color filter
	TrackedController* controllers; // a pointer to a linked list of connected controllers
	PSMoveTrackingColor* available_colors; // a pointer to a linked list of available tracking colors
	int debug;
};

/* Macro: Print a critical message if an assertion fails */
#define tracker_CRITICAL(x) \
        {fprintf(stderr, "[TRACKER] Assertion fail in %s: %s\n", __func__, x);}

/**
 * Adapts the cameras exposure to the current lighting conditions
 * This function will adapt to the most suitable exposure, it will start
 * with "expMin" and increases step by step to "expMax" until it reaches "lumMin" or "expMax"
 *
 * tracker - A valid PSMoveTracker * instance
 * limMin  - Minimal luminance to reach
 * expMin  - Minimal exposure to test
 * expMax  - Maximal exposure to test
 *
 * Returns: the most suitable exposure within range
 **/
int tracker_adapt_to_light(PSMoveTracker *tracker, int lumMin, int expMin, int expMax);

/**
 * This function switches the sphere of the given PSMove on to the given color and takes
 * a picture via the given capture. Then it switches it of and takes a picture again. A difference image
 * is calculated from these two images. It stores the image of the lit sphere and
 * of the diff-image in the passed parameter "on" and "diff". Before taking
 * a picture it waits for the specified delay (in microseconds).
 *
 * capture - the video capture to take pictures from
 * move    - the PSMove controller to use
 * r,g,b   - the RGB color to use to lit the sphere
 * on	   - the pre-allocated image to store the captured image when the sphere is lit
 * diff    - the pre-allocated image to store the calculated diff-image
 * delay   - the time to wait before taking a picture (in microseconds)
 **/
void tracker_get_diff(CvCapture* capture, PSMove* move, int r, int g, int b, IplImage* on, IplImage* diff, int delay);

/**
 * This function assures thate the roi (region of interest) is always within the bounds
 * of the camera image.
 *
 * tc         - The TrackableController containing the roi to check & fix
 * roi_width  - the width of the roi
 * roi_height - the height of the roi
 * cam_width  - the width of the camera image
 * cam_height - the height of the camera image
 **/
void tracker_fix_roi(TrackedController* tc, int roi_width, int image_height, int cam_width, int cam_height);

/**
 * This function prepares the linked list of suitable colors, that can be used for tracking.
 */
void tracker_prepare_tracking_colors(PSMoveTracker* tracker);

/**
 * This function is just the internal implementation of "psmove_tracker_update"
 */
int psmove_tracker_update_controller(PSMoveTracker* tracker, TrackedController* tc);

PSMoveTracker *
psmove_tracker_new() {
	int i = 0;
	PSMoveTracker* t = (PSMoveTracker*) calloc(1, sizeof(PSMoveTracker));
	t->controllers = 0x0;
	t->rHSV = cvScalar(10, 85, 85, 0);
	t->cam = CV_CAP_ANY;

	// use dynamic exposure (adapts only on startup)
	//t->exposure = tracker_adapt_to_light(t, 25, 0x10, 0x40);

	// use static exposure
	t->exposure = 0x10;
	th_set_camera_params(0, 0, 0, t->exposure, 0, 0xFF, 0xFF, 0xFF);

	// prepare available colors for tracking
	tracker_prepare_tracking_colors(t);

	// start the video capture device for tracking
	t->capture = cvCaptureFromCAM(t->cam);
	assert(t->capture);
	if (!t->capture) {
		tracker_CRITICAL("unable to open the capture device");
	}

	// just query a frame
	IplImage* frame;
	while (1) {
		// TODO: why wait so long?
		usleep(10000);
		frame = th_query_frame(t->capture);
		if (!frame)
			continue;
		else
			break;
	}

	// prepare ROI data structures
	t->roiI[0] = cvCreateImage(cvGetSize(frame), frame->depth, frame->nChannels);
	t->roiM[0] = cvCreateImage(cvGetSize(frame), frame->depth, 1);
	for (i = 1; i < ROIS; i++) {
		IplImage* z = t->roiI[i - 1];
		int w = z->width;
		t->roiI[i] = cvCreateImage(cvSize(w * 0.6, w * 0.6), z->depth, z->nChannels);
		t->roiM[i] = cvCreateImage(cvSize(w * 0.6, w * 0.6), z->depth, 1);
	}

	t->kCalib = cvCreateStructuringElementEx(5, 5, 3, 3, CV_SHAPE_RECT, 0x0);
	t->kTrack = 0x0; // use default kernel
	t->storage = cvCreateMemStorage(0);
	return t;
}

enum PSMoveTracker_Status psmove_tracker_enable(PSMoveTracker *tracker, PSMove *move) {
	// check if there is a free color, return on error immediately
	PSMoveTrackingColor* color = tracker->available_colors;
	for (; color != 0x0 && color->is_used; color = color->next)
		;

	if (color == 0x0)
		return Tracker_CALIBRATION_ERROR;

	unsigned char r = color->r;
	unsigned char g = color->g;
	unsigned char b = color->b;

	return psmove_tracker_enable_with_color(tracker, move, r, g, b);
}

enum PSMoveTracker_Status psmove_tracker_enable_with_color(PSMoveTracker *tracker, PSMove *move, unsigned char r, unsigned char g, unsigned char b) {
	// check if the controller is already enabled!
	if (tracked_controller_find(tracker->controllers, move))
		return Tracker_CALIBRATED;

	// check if the color is already in use, if not, mark it as used, return if it is already used
	PSMoveTrackingColor* tracked_color = tracked_color_find(tracker->available_colors, r, g, b);
	if (tracked_color != 0x0 && tracked_color->is_used)
		return Tracker_CALIBRATION_ERROR;

	// clear the calibration html trace
	psmove_html_trace_clear();

	IplImage* images[BLINKS]; // array of images saved during calibration for estimation of sphere color
	IplImage* diffs[BLINKS]; // array of masks saved during calibration for estimation of sphere color
	double sizes[BLINKS]; // array of blob sizes saved during calibration for estimation of sphere color
	int i;
	for (i = 0; i < BLINKS; i++) {
		images[i] = cvCloneImage(tracker->roiI[0]);
		diffs[i] = cvCloneImage(tracker->roiM[0]);
	}
	// controller is not enabled ... enable it!
	PSMoveTracker* t = tracker;
	CvScalar assignedColor = cvScalar(b, g, r, 0);
	psmove_html_trace_var_color("assignedColor", assignedColor);

	// for each blink
	for (i = 0; i < BLINKS; i++) {
		// create a diff image
		tracker_get_diff(t->capture, move, r, g, b, images[i], diffs[i], 1000000 / 4);

		psmove_html_trace_image(images[i], i, "originals");
		psmove_html_trace_image(diffs[i], i, "rawdiffs");
		// threshold it to reduce image noise
		cvThreshold(diffs[i], diffs[i], 20, 0xFF, CV_THRESH_BINARY);

		psmove_html_trace_image(diffs[i], i, "threshdiffs");

		// use morphological operations to further remove noise
		cvErode(diffs[i], diffs[i], t->kCalib, 1);
		cvDilate(diffs[i], diffs[i], t->kCalib, 1);

		psmove_html_trace_image(diffs[i], i, "erodediffs");
	}

	// put the diff images together!
	for (i = 1; i < BLINKS; i++) {
		cvAnd(diffs[0], diffs[i], diffs[0], 0x0);
	}

	// find the biggest contour
	CvSeq* contour;
	cvFindContours(diffs[0], t->storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));
	float sizeBest = 0;
	CvSeq* contourBest = 0x0;
	for (; contour != 0x0; contour = contour->h_next) {
		float f = cvContourArea(contour, CV_WHOLE_SEQ, 0);
		if (f > sizeBest) {
			sizeBest = f;
			contourBest = contour;
		}
	}

	// use only the biggest contour for the following color estimation
	cvSet(diffs[0], th_black, 0x0);
	if (contourBest)
		cvDrawContours(diffs[0], contourBest, th_white, th_white, -1, CV_FILLED, 8, cvPoint(0, 0));

	psmove_html_trace_image(diffs[0], 0, "finaldiff");

	if (cvCountNonZero(diffs[0]) < CALIB_MIN_SIZE) {
		psmove_html_trace_log_entry("WARNING", "The final mask my not be representative for color estimation.");
	}

	// calculate the avg color within the biggest contour
	CvScalar color = cvAvg(images[0], diffs[0]);
	CvScalar hsv_assigned = th_brg2hsv(assignedColor);
	CvScalar hsv_color = th_brg2hsv(color);
	psmove_html_trace_var_color("estimatedColor", color);

	if (abs(hsv_assigned.val[0]-hsv_color.val[0]) >10) {
				psmove_html_trace_log_entry("WARNING", "The estimated color seems not to be similar to the color it should be.");
			}

	// just reusing the data structure
	IplImage* mask = diffs[0];

	int valid_countours = 0;
	// calculate upper & lower bounds for the color filter
	CvScalar min, max;
	th_minus(hsv_color.val, t->rHSV.val, min.val, 3);
	th_plus(hsv_color.val, t->rHSV.val, max.val, 3);
	// for each image (where the sphere was lit)

	for (i = 0; i < BLINKS; i++) {
		// convert to HSV
		cvCvtColor(images[i], images[i], CV_BGR2HSV);

		// apply color filter
		cvInRangeS(images[i], min, max, mask);
		// remove noise with morphological operations
		cvErode(mask, mask, t->kTrack, 1);
		cvDilate(mask, mask, t->kTrack, 1);

		psmove_html_trace_image(mask, i, "filtered");

		// try to find the sphere as a contour
		cvFindContours(mask, t->storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

		// there may be only a single contour in this picture with at least 100px size
		sizes[i] = 0;
		if (contour != 0x0)
			sizes[i] = cvContourArea(contour, CV_WHOLE_SEQ, 0);

		// check for errors (no contour, more than one contour, or contour too small)
		if (contour == 0x0) {
			psmove_html_trace_array_item_at(i, "contours", "no contour");
		} else if (contour->h_next != 0x0 || contour->h_prev != 0x0) {
			psmove_html_trace_array_item_at(i, "contours", "too many");
		} else if (sizes[i] <= CALIB_MIN_SIZE) {
			psmove_html_trace_array_item_at(i, "contours", "too small");
		} else {
			psmove_html_trace_array_item_at(i, "contours", "OK");
			valid_countours++;
		}

	}

	cvClearMemStorage(t->storage);

	// clean up all temporary images
	for (i = 0; i < BLINKS; i++) {
		cvReleaseImage(&images[i]);
		cvReleaseImage(&diffs[i]);
	}

	if (valid_countours < BLINKS) {
		psmove_html_trace_log_entry("ERROR", "The sphere could not be found in all images.");
		return Tracker_CALIBRATION_ERROR;
	}

	// check if the size of the found contours are near to each other
	double stdSizes = sqrt(th_var(sizes, BLINKS));
	if (stdSizes < (th_avg(sizes, BLINKS) * 1.0 / CALIB_SIZE_STD)) {
		// insert to list
		TrackedController* itm = tracked_controller_insert(&tracker->controllers, move);
		// set current color
		itm->dColor = cvScalar(b, g, r, 0);
		// set estimated color
		itm->eColor = color;
		itm->eColorHSV = hsv_color;
		itm->roi_x = 0;
		itm->roi_y = 0;
		itm->roi_level = 0;
		// set the state of the available colors
		if (tracked_color != 0x0)
			tracked_color->is_used = 1;
		return Tracker_CALIBRATED;
	}

	psmove_html_trace_log_entry("ERROR", "The spheres found differ too much in size.");
	return Tracker_CALIBRATION_ERROR;
}

int psmove_tracker_get_color(PSMoveTracker *tracker, PSMove *move, unsigned char *r, unsigned char *g, unsigned char *b) {
	TrackedController* tc = tracked_controller_find(tracker->controllers, move);
	if (tc != 0x0) {
		*r = tc->dColor.val[2];
		*g = tc->dColor.val[1];
		*b = tc->dColor.val[0];
		return 1;
	} else
		return 0;
}

void psmove_tracker_disable(PSMoveTracker *tracker, PSMove *move) {
	TrackedController* tc = tracked_controller_find(tracker->controllers, move);
	PSMoveTrackingColor* color = tracked_color_find(tracker->available_colors, tc->dColor.val[2], tc->dColor.val[1], tc->dColor.val[0]);
	if (tc != 0x0) {
		tracked_controller_remove(&tracker->controllers, move);
		tracked_controller_release(&tc, 0);
	}
	if (color != 0x0)
		color->is_used = 0;
}

enum PSMoveTracker_Status psmove_tracker_get_status(PSMoveTracker *tracker, PSMove *move) {
	TrackedController* tc = tracked_controller_find(tracker->controllers, move);
	if (tc != 0x0) {
		return Tracker_CALIBRATED;
	} else {
		return Tracker_UNCALIBRATED;
	}
}

IplImage*
psmove_tracker_get_image(PSMoveTracker *tracker) {
	return tracker->frame;
}

void psmove_tracker_update_image(PSMoveTracker *tracker) {
	tracker->frame = th_query_frame(tracker->capture);
}

int psmove_tracker_update_controller(PSMoveTracker *tracker, TrackedController* tc) {
	PSMoveTracker* t = tracker;
	int i = 0;
	int sphere_found = 0;

	// calculate upper & lower bounds for the color filter
	CvScalar min, max;
	th_minus(tc->eColorHSV.val, t->rHSV.val, min.val, 3);
	th_plus(tc->eColorHSV.val, t->rHSV.val, max.val, 3);
	// this is the tracking algo
	while (1) {
		IplImage *roi_i = t->roiI[tc->roi_level];
		IplImage *roi_m = t->roiM[tc->roi_level];
		// cut out the roi!
		cvSetImageROI(t->frame, cvRect(tc->roi_x, tc->roi_y, roi_i->width, roi_i->height));
		cvCvtColor(t->frame, roi_i, CV_BGR2HSV);
		// apply color filter
		cvInRangeS(roi_i, min, max, roi_m);

		// this will remove small distortions that have a similar color
		cvErode(roi_m, roi_m, 0x0, 1);
		cvDilate(roi_m, roi_m, 0x0, 1);

		CvSeq* contour = 0x0;
		CvSeq* aC = 0x0;
		CvSeq* best = 0x0;
		cvFindContours(roi_m, t->storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

		float sizeC = 0;
		float f = 0;

		for (aC = contour; aC != 0x0; aC = aC->h_next) {
			f = cvContourArea(aC, CV_WHOLE_SEQ, 0);
			if (f > sizeC) {
				sizeC = f;
				best = aC;
			}
		}
		cvClearMemStorage(t->storage);
		if (best) {
			sphere_found = 1;
			CvMoments mu;
			CvRect br = cvBoundingRect(best, 0);

			// whenn calling "cvFindContours", the original image is partly consumed -> restoring the contour in the binary image
			cvDrawContours(roi_m, best, th_white, th_white, -1, CV_FILLED, 8, cvPoint(0, 0));

			// calucalte image-moments to estimate the center off mass (x/y position of the blob)
			cvSetImageROI(roi_m, br);
			cvMoments(roi_m, &mu, 0);
			cvResetImageROI(roi_m);
			CvPoint p = cvPoint(mu.m10 / mu.m00, mu.m01 / mu.m00);
			tc->x = p.x + tc->roi_x + br.x;
			tc->y = p.y + tc->roi_y + br.y;

			// update the future roi box
			br.width = th_max(br.width, br.height) * 2;
			br.height = br.width;
			for (i = 0; i < ROIS; i++) {
				if (br.width > t->roiI[i]->width && br.height > t->roiI[i]->height)
					break;

				tc->roi_level = i;
				// update easy accessors
				roi_i = t->roiI[tc->roi_level];
				roi_m = t->roiM[tc->roi_level];
			}

			tc->roi_x = tc->x - roi_i->width / 2;
			tc->roi_y = tc->y - roi_i->height / 2;

			// assure thate the roi is within the target image
			tracker_fix_roi(tc, roi_i->width, roi_i->height, t->roiI[0]->width, t->roiI[0]->height);
		}
		cvResetImageROI(t->frame);

		if (sphere_found || roi_i->width == t->roiI[0]->width) {
			break;
		} else {
			tc->roi_x += roi_i->width / 2;
			tc->roi_y += roi_i->height / 2;

			tc->roi_level = tc->roi_level - 1;
			// update easy accessors
			roi_i = t->roiI[tc->roi_level];
			roi_m = t->roiM[tc->roi_level];

			tc->roi_x -= roi_i->width / 2;
			tc->roi_y -= roi_i->height / 2;

			// assure that the roi is within the target image
			tracker_fix_roi(tc, roi_i->width, roi_i->height, t->roiI[0]->width, t->roiI[0]->height);
		}
	}
	return sphere_found;
}
int psmove_tracker_update(PSMoveTracker *tracker, PSMove *move) {
	TrackedController* tc;
	int spheres_found = 0;
	int UPDATE_ALL_CONTROLLERS = move == 0x0;

	if (UPDATE_ALL_CONTROLLERS) {
		// iterate trough all controllers and find their lit spheres
		tc = tracker->controllers;
		for (; tc != 0x0 && tracker->frame; tc = tc->next) {
			spheres_found += psmove_tracker_update_controller(tracker, tc);
		}
	} else {
		// find just that specific controller
		tc = tracked_controller_find(tracker->controllers, move);
		if (tracker->frame && tc) {
			spheres_found = psmove_tracker_update_controller(tracker, tc);
		}
	}

	// return the number of spheres found
	return spheres_found;

}

int psmove_tracker_get_position(PSMoveTracker *tracker, PSMove *move, int *x, int *y, int *radius) {
	TrackedController* tc = tracked_controller_find(tracker->controllers, move);
	if (x != 0x0)
		*x = tc->x;

	if (y != 0x0)
		*y = tc->y;

	return 1;
}

void psmove_tracker_free(PSMoveTracker *tracker) {

}

// [PRIVATE][implementation] internal functions used for the tracker

int tracker_adapt_to_light(PSMoveTracker *tracker, int lumMin, int expMin, int expMax) {

	int exp = expMin;
	// the delay in microseconds to wait for the camera to take the new parameters to take effect
	int delay = 10;
	// se the camera parameters to minimal exposure
	th_set_camera_params(0, 0, 0, exp, 0, 0xff, 0xff, 0xff);

#ifdef WIN32
	// restart the camera (only Windows)
	if (tracker->capture)
		cvReleaseCapture(&tracker->capture);

	tracker->capture = cvCaptureFromCAM(tracker->cam);
	if (!tracker->capture) {
		tracker_CRITICAL("unable to open the capture device");
	}
	assert(tracker->capture);

#endif

	IplImage* frame;
	// calculate a stepsize to increase the exposure, so that not more than 5 steps are neccessary
	int step = (expMax - expMin) / 10;
	if (step == 0)
		step = 1;
	int lastExp = exp;
	int cntr;
	while (1) {
		// wait for the changes to take effect
		// TODO: delay is problably windows specific
		cntr = delay;
		while (cntr > 0) {
			cvWaitKey(1);
			cntr = cntr - 1;
			frame = th_query_frame(tracker->capture);
		}
		if (!frame)
			continue;

		// calculate the average color
		CvScalar avgColor = cvAvg(frame, 0x0);
		// calculate the average luminance (energy)
		float avgLum = th_avg(avgColor.val, 3);

		// ignore empty image
		// TODO: zero luminance is problably windows specific
		if (avgLum == 0)
			continue;

		printf("%d: %f\n", exp, avgLum);
		// if the minimal luminance "limMin" has not been reached, increase the current exposure "exp"
		if (avgLum < lumMin)
			exp = exp + step;

		// check exposure boundaries
		if (exp < expMin)
			exp = expMin;
		if (exp > expMax)
			exp = expMax;

		// if the current exposure has been modified, apply it!
		if (lastExp != exp) {
			// reconfigure the camera
			th_set_camera_params(0, 0, 0, exp, 0, 0xff, 0xff, 0xff);

#ifdef WIN32
			// reset the camera (windows only)
			if (tracker->capture)
				cvReleaseCapture(&tracker->capture);
			tracker->capture = cvCaptureFromCAM(tracker->cam);
#endif
			lastExp = exp;
		} else
			break;
	}

#ifdef WIN32
	//Release the capture device (windows only)
	cvReleaseCapture(&tracker->capture);
#endif

	printf("exposure set to %d(0x%x)\n", exp, exp);
	return exp;
}

void tracker_get_diff(CvCapture* capture, PSMove* move, int r, int g, int b, IplImage* on, IplImage* diff, int delay) {
	// the time to wait for the controller to set the color up
	IplImage* frame;
	// switch the LEDs ON and wait for the sphere to be fully lit
	psmove_set_leds(move, r, g, b);
	psmove_update_leds(move);
	usleep(delay);

	/*
	 long started_at = get_millis();
	 while (get_millis() < started_at + delay) {
	 query frame
	 }*/
	// query the frame and save it
	frame = th_query_frame(capture);
	{
		// on windows it seems like i have to query the frame twice
		// TODO: why query twice?
		cvWaitKey(1);
		frame = th_query_frame(capture);
	}
	cvCopy(frame, on, 0x0);

	// switch the LEDs OFF and wait for the sphere to be off
	psmove_set_leds(move, 0, 0, 0);
	psmove_update_leds(move);
	usleep(delay);
	frame = th_query_frame(capture);
	{
		// on windows it seems like i have to query the frame twice
		// TODO: why query twice?
		cvWaitKey(1);
		frame = th_query_frame(capture);
	}

	// convert both to grayscale images
	IplImage* grey1 = cvCloneImage(diff);
	IplImage* grey2 = cvCloneImage(diff);
	cvCvtColor(frame, grey1, CV_BGR2GRAY);
	cvCvtColor(on, grey2, CV_BGR2GRAY);

	// calculate the diff of to images and save it in "diff"
	cvAbsDiff(grey1, grey2, diff);

	// clean up
	cvReleaseImage(&grey1);
	cvReleaseImage(&grey2);
}

void tracker_fix_roi(TrackedController* tc, int roi_width, int roi_height, int cam_width, int cam_height) {
	if (tc->roi_x < 0)
		tc->roi_x = 0;
	if (tc->roi_y < 0)
		tc->roi_y = 0;

	if (tc->roi_x + roi_width > cam_width)
		tc->roi_x = cam_width - roi_width;
	if (tc->roi_y + roi_height > cam_height)
		tc->roi_y = cam_height - roi_height;
}

void tracker_prepare_tracking_colors(PSMoveTracker* tracker) {
	// create CYAN
	tracked_color_insert(&tracker->available_colors, 0x0, 0xff, 0xff);

	// create MAGENTA
	tracked_color_insert(&tracker->available_colors, 0xff, 0x00, 0xff);
}
