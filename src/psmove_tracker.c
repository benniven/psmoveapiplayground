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
#include "high_precision_timer.h"

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
 * TODO
 **/
void tracker_get_diff(CvCapture* capture, PSMove* move, int r, int g, int b, IplImage* on, IplImage* diff);

#define TRACE_NEW_LINE "<div style=\"clear:both;\"></div>"
#define TRACE_TEXT_OK "<H1 style=\"font-size:150%; color:green\">OK</H1>"
#define TRACE_TEXT_ERR "<H1 style=\"font-size:150%; color:red\">ERROR</H1>"

void psmove_tracker_image_trace(PSMoveTracker *tracker, IplImage *image, const char *message, int n_prefix, const char* s_prefix, const char* line_postfix);
void psmove_tracker_clear_trace();
void psmove_tracker_text_trace(char *message, const char* line_postfix);
void psmove_tracker_color_trace(CvScalar color, const char* line_postfix);

#define psmove_tracker_trace(tracker, image, message, n_prefix, s_prefix, line_postfix) psmove_tracker_image_trace((tracker),(image),(message),(n_prefix),(s_prefix),(line_postfix))
#define psmove_tracker_trace_clear() psmove_tracker_clear_trace()
#define psmove_tracker_trace_text(message, line_postfix) psmove_tracker_text_trace((message),(line_postfix))
#define psmove_tracker_trace_color(color, line_postfix) psmove_tracker_color_trace((color),(line_postfix))

PSMoveTracker *
psmove_tracker_new() {
	int i = 0;
	PSMoveTracker* t = (PSMoveTracker*) calloc(1, sizeof(PSMoveTracker));
	t->controllers = 0x0;
	t->rHSV = cvScalar(10, 85, 85, 0);
	t->cam = CV_CAP_ANY;

	// start the video capture
	t->capture = cvCaptureFromCAM(t->cam);
	assert(t->capture);
	if (!t->capture) {
		tracker_CRITICAL("unable to open the capture device");
	}

	// adapt to current lighting condition
	t->exposure = tracker_adapt_to_light(t, 25, 0x10, 0x40);

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

	t->kCalib = cvCreateStructuringElementEx(11, 11, 6, 6, CV_SHAPE_RECT, 0x0);
	t->kTrack = 0x0; // use default kernel
	t->storage = cvCreateMemStorage(0);
	return t;
}

enum PSMoveTracker_Status psmove_tracker_enable(PSMoveTracker *tracker, PSMove *move) {
	// check if the controller is already enabled!
	if (tracked_controller_find(tracker->controllers, move))
		return Tracker_CALIBRATED;

	psmove_tracker_trace_clear();

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
	float f = 1.0;
	if (tracker->exposure > 20)
		f = 0.7;
	if (tracker->exposure > 30)
		f = 0.5;

	int r = 0xFF * f;
	int g = 0 * 0xFF * f;
	int b = 0xFF * f;

	psmove_tracker_trace_text("Starting calibration ...", TRACE_NEW_LINE);
	// for each blink
	for (i = 0; i < BLINKS; i++) {
		// create a diff image
		tracker_get_diff(t->capture, move, r, g, b, images[i], diffs[i]);

		psmove_tracker_trace(tracker, images[i], "original image", i, "org", "");
		psmove_tracker_trace(tracker, diffs[i], "raw diff", i, "1diff", "");
		// threshold it to reduce image noise
		cvThreshold(diffs[i], diffs[i], 20, 0xFF, CV_THRESH_BINARY);

		psmove_tracker_trace(tracker, diffs[i], "thresh diff", i, "thresh", "");

		// use morphological operations to further remove noise
		cvErode(diffs[i], diffs[i], t->kCalib, 1);
		cvDilate(diffs[i], diffs[i], t->kCalib, 1);

		psmove_tracker_trace(tracker, diffs[i], "cleaned diff", i, "morph", TRACE_NEW_LINE);
	}

	// put the diff images together!
	for (i = 1; i < BLINKS; i++) {
		cvAnd(diffs[0], diffs[i], diffs[0], 0x0);
	}

	psmove_tracker_trace_text("Final diff image: ", "");
	psmove_tracker_trace(tracker, diffs[0], "final diff", BLINKS, "final", "");
	int pixels = cvCountNonZero(diffs[0]);
	if(pixels<CALIB_MIN_SIZE){
		psmove_tracker_trace_text(TRACE_TEXT_ERR, "");
		psmove_tracker_trace_text("(not enough pixels set for estimation)", TRACE_NEW_LINE);
	}
	else
		psmove_tracker_trace_text(TRACE_TEXT_OK, TRACE_NEW_LINE);

	// calculate the avg color within the remaining area
	CvScalar color = cvAvg(images[0], diffs[0]);
	CvScalar hsv_color = th_brg2hsv(color);

	psmove_tracker_trace_text("LED color: ", "");
	psmove_tracker_trace_color(cvScalar(b,g,r,0), TRACE_NEW_LINE);

	psmove_tracker_trace_text("Estimated color: ", "");
	psmove_tracker_trace_color(color, TRACE_NEW_LINE);

	// just reusing the data structure
	IplImage* mask = diffs[0];

	int all_contours_valid = 1;
	// calculate upper & lower bounds for the color filter
	CvScalar min, max;
	th_minus(hsv_color.val, t->rHSV.val, min.val, 3);
	th_plus(hsv_color.val, t->rHSV.val, max.val, 3);
	// for each image (where the sphere was lit)

	psmove_tracker_trace_text("Trying to find the sphere by color ...", TRACE_NEW_LINE);
	for (i = 0; i < BLINKS; i++) {
		psmove_tracker_trace(tracker,images[i], "original", i, "orig_unfiltered", "");
		// convert to HSV
		cvCvtColor(images[i], images[i], CV_BGR2HSV);

		// apply color filter
		cvInRangeS(images[i], min, max, mask);
		// remove noise with morphological operations
		cvErode(mask, mask, t->kTrack, 1);
		cvDilate(mask, mask, t->kTrack, 1);

		psmove_tracker_trace(tracker,mask, "filtered", i, "filtered", "");

		// try to find the sphere as a contour
		CvSeq* contour;
		cvFindContours(mask, t->storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

		// there may be only a single contour in this picture with at least 100px size
		sizes[i] = 0;
		if (contour != 0x0)
			sizes[i] = cvContourArea(contour, CV_WHOLE_SEQ, 0);

		// check for errors (no contour, more than one contour, or contour too small)
		if (contour == 0x0 || contour->h_next != 0x0 || contour->h_prev != 0x0 || sizes[i] <= CALIB_MIN_SIZE) {
			psmove_tracker_trace_text(TRACE_TEXT_ERR, "");

			if (contour == 0x0) {
				psmove_tracker_trace_text("(no contour)","");
				printf("%s\n", "no contours!");
			} else if (contour->h_next != 0x0 || contour->h_prev != 0x0) {
				psmove_tracker_trace_text("(there are other contours!)","");
				printf("%s\n", "there are other contours!");
			} else if (sizes[i] <= 100) {
				psmove_tracker_trace_text("(the contour is to small!)","");
				printf("%s\n", "the contour is to small!");
			}
			psmove_tracker_trace_text("",TRACE_NEW_LINE);
			// at least on contour was either not found or is invalid
			all_contours_valid = 0;
		} else {
			psmove_tracker_trace_text(TRACE_TEXT_OK, TRACE_NEW_LINE);
		}
	}

	cvClearMemStorage(t->storage);

	// clean up all temporary images
	for (i = 0; i < BLINKS; i++) {
		cvReleaseImage(&images[i]);
		cvReleaseImage(&diffs[i]);
	}

	if (!all_contours_valid)
		return Tracker_UNCALIBRATED;

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
		return Tracker_CALIBRATED;
	}
	return Tracker_UNCALIBRATED;
}

enum PSMoveTracker_Status psmove_tracker_enable_with_color(PSMoveTracker *tracker, PSMove *move, unsigned char r, unsigned char g, unsigned char b) {

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

}

enum PSMoveTracker_Status psmove_tracker_get_status(PSMoveTracker *tracker, PSMove *move) {

}

IplImage*
psmove_tracker_get_image(PSMoveTracker *tracker) {
	return tracker->frame;
}

void psmove_tracker_update_image(PSMoveTracker *tracker) {
	tracker->frame = th_query_frame(tracker->capture);
}

int psmove_tracker_update(PSMoveTracker *tracker, PSMove *move) {
	int i = 0;
	PSMoveTracker* t = tracker;
	TrackedController* tc = tracked_controller_find(t->controllers, move);

	if (!t->frame || !tc) {
		printf("err\n");
		return 0;
	}

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
		cvSet(t->roiM[0], th_black, 0x0);
		cvSet(roi_m, th_black, 0x0);
		if (best) {
			cvDrawContours(roi_m, best, th_white, th_white, -1, CV_FILLED, 8, cvPoint(0, 0));
			cvDrawContours(t->roiM[0], best, th_white, th_white, -1, CV_FILLED, 8, cvPoint(tc->roi_x, tc->roi_y));

			//cvShowImage("big mask: ", t->roiM[0]);
			//cvShowImage("small mask: ", roi_m);

			// calucalte image-moments to estimate the center off mass (x/y position of the blob)
			CvMoments mu;
			cvMoments(roi_m, &mu, 0);
			CvPoint p = cvPoint(mu.m10 / mu.m00, mu.m01 / mu.m00);
			tc->x = p.x + tc->roi_x;
			tc->y = p.y + tc->roi_y;

			// update the future roi box
			CvRect br = cvBoundingRect(best, 0);
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
			// is the x/y position of the future roi not within the image (<0)
			// TODO: auslagern
			{
				if (tc->roi_x < 0)
					tc->roi_x = 0;
				if (tc->roi_y < 0)
					tc->roi_y = 0;

				if (tc->roi_x + roi_i->width > t->roiI[0]->width)
					tc->roi_x = t->roiI[0]->width - roi_i->width;
				if (tc->roi_y + roi_i->height > t->roiI[0]->height)
					tc->roi_y = t->roiI[0]->height - roi_i->height;
			}
		}
		cvResetImageROI(t->frame);

		if (best || roi_i->width == t->roiI[0]->width) {
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

			// TODO: (siehe oben) AUSLAGERN
			if (tc->roi_x < 0)
				tc->roi_x = 0;
			if (tc->roi_y < 0)
				tc->roi_y = 0;

			if (tc->roi_x + roi_i->width > t->roiI[0]->width)
				tc->roi_x = t->roiI[0]->width - roi_i->width;
			if (tc->roi_y + roi_i->height > t->roiI[0]->height)
				tc->roi_y = t->roiI[0]->height - roi_i->height;
		}
	}
	// TODO: proper return value
	return 1;
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
	// Release the capture device (windows only)
	//cvReleaseCapture(&tracker->capture);
#endif

	printf("exposure set to %d(0x%x)\n", exp, exp);
	return exp;
}

void tracker_get_diff(CvCapture* capture, PSMove* move, int r, int g, int b, IplImage* on, IplImage* diff) {
	// the time to wait for the controller to set the color up
	int delay = 1000000 / 4;
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

void psmove_tracker_clear_trace() {
	FILE *pFile;
	pFile = fopen("debug.html", "w");
	if (pFile != NULL) {

		fclose(pFile);
	}
}

void psmove_tracker_color_trace(CvScalar color, const char* line_postfix) {
	FILE *pFile;
	pFile = fopen("debug.html", "a");
	char text[512];

	unsigned int r = (unsigned int) round(color.val[2]);
	unsigned int g = (unsigned int) round(color.val[1]);
	unsigned int b = (unsigned int) round(color.val[0]);

	sprintf(text, "<div style=\"float:left; background-color:%02X%02X%02X; width: 100px;\">0x%02X%02X%02X</div>\n", r, g, b, r, g, b);
	if (pFile != NULL) {
		fputs(text, pFile);
		fputs(line_postfix, pFile);
		fclose(pFile);
	}
}

void psmove_tracker_text_trace(char *message, const char* line_postfix) {
	FILE *pFile;
	pFile = fopen("debug.html", "a");
	char text[512];

	sprintf(text, "<div style=\"float:left;\">%s</div>\n", message);
	if (pFile != NULL) {
		fputs(text, pFile);
		fputs(line_postfix, pFile);
		fclose(pFile);
	}
}
void psmove_tracker_image_trace(PSMoveTracker *tracker, IplImage *image, const char *message, int n_prefix, const char* s_prefix, const char* line_postfix) {
	FILE *pFile;
	pFile = fopen("debug.html", "a");
	char img_name[512];
	char img_tag[512];

	sprintf(img_name, "%s/%d_%s%s", "./debug/", n_prefix, s_prefix, ".jpg");
	th_save_jpg(img_name, image, 100);

	const char* tag_prototype = "<div style=\"float:left;\"><table>"
			"<tr><td><img src=\"%s\""
			" style=\"width:100px; border:1px solid black;\""
			" onmouseover=\"this.style.width='%dpx'\""
			" onmouseout=\"this.style.width='100px'\"/></td></tr>"
			"</table></div>\n";

	sprintf(img_tag, tag_prototype, img_name, image->width);

	if (pFile != NULL) {
		fputs(img_tag, pFile);
		fputs(line_postfix, pFile);
		fclose(pFile);
	}
}
