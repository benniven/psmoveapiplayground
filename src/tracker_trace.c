/**
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2012 Benjamin Venditti <benjamin.venditti@gmail.com>
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

#include <stdio.h>
#include "tracker_trace.h"
#include "tracker_helpers.h"

int TRACE_IMG_COUNT;
#define TRACE_OUTPUT "debug.js"

void psmove_trace_clear() {
	TRACE_IMG_COUNT = 0;
	FILE *pFile;
	pFile = fopen(TRACE_OUTPUT, "w");
	if (pFile != NULL) {
		fputs("originals = new Array();\n", pFile);
		fputs("rawdiffs = new Array();\n", pFile);
		fputs("threshdiffs = new Array();\n", pFile);
		fputs("erodediffs = new Array();\n", pFile);
		fputs("finaldiff = new Array();\n", pFile);
		fputs("filtered = new Array();\n", pFile);
		fputs("contours = new Array();\n", pFile);
		fputs("log_table = new Array();\n\n", pFile);
		fclose(pFile);
	}
}

void psmove_trace_image(IplImage *image, int index, const char* target) {
	char img_name[256];
	// write image to file sysxtem
	sprintf(img_name, "%s%d%s", "./debug/image_", TRACE_IMG_COUNT, ".jpg");
	th_save_jpg(img_name, image, 100);
	TRACE_IMG_COUNT++;
	// write image-name to java script array
	psmove_html_trace_array_item_at(index, target, img_name);
}

void psmove_trace_array_item_at(int index, const char* target, const char* value) {
	FILE *pFile;
	pFile = fopen(TRACE_OUTPUT, "a");
	char array[256];
	// write string to array
	sprintf(array, "%s[%d]='%s';\n", target, index, value);

	if (pFile != NULL) {
		fputs(array, pFile);
		fclose(pFile);
	}
}

void psmove_trace_array_item(const char* target, const char* value) {
	FILE *pFile;
	pFile = fopen(TRACE_OUTPUT, "a");
	char array[256];
	// write string to array
	sprintf(array, "%s.push('%s');\n", target, value);

	if (pFile != NULL) {
		fputs(array, pFile);
		fclose(pFile);
	}
}

void psmove_trace_put_log_entry(char* type, char* value) {
	FILE *pFile;
	pFile = fopen(TRACE_OUTPUT, "a");
	char array[256];
	// write string to array
	sprintf(array, "log_table.push({type:'%s', value:'%s'});\n", type, value);

	if (pFile != NULL) {
		fputs(array, pFile);
		fclose(pFile);
	}
}

void psmove_trace_put_text(char* text) {
	FILE *pFile;
	pFile = fopen(TRACE_OUTPUT, "a");
	if (pFile != NULL) {
		fputs(text, pFile);
		fputs("\n", pFile);
		fclose(pFile);
	}
}

void psmove_trace_put_int_var(char* var, int value) {
	FILE *pFile;
	pFile = fopen(TRACE_OUTPUT, "a");
	char text[256];
	sprintf(text, "%s=%d;\n", var, value);
	if (pFile != NULL) {
		fputs(text, pFile);
		fclose(pFile);
	}
}
void psmove_trace_put_text_var(char* var, char* value) {
	FILE *pFile;
	pFile = fopen(TRACE_OUTPUT, "a");
	char text[256];
	sprintf(text, "%s='%s';\n", var, value);
	if (pFile != NULL) {
		fputs(text, pFile);
		fclose(pFile);
	}
}

void psmove_trace_put_color_var(char* var, CvScalar color) {
	char text[32];
	unsigned int r = (unsigned int) round(color.val[2]);
	unsigned int g = (unsigned int) round(color.val[1]);
	unsigned int b = (unsigned int) round(color.val[0]);
	sprintf(text, "%02X%02X%02X", r, g, b);
	psmove_trace_put_text_var(var, text);
}
