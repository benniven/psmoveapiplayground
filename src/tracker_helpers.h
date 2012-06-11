#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"
#include <time.h>

#include "psmove.h"

#ifndef TRACKER_HELPERS_H
#define TRACKER_HELPERS_H

#define th_odd(x) (((x)/2)*2+1)
#define th_black cvScalarAll(0x0)
#define th_white cvScalarAll(0xFF)
#define th_green cvScalar(0,0xff,0,0)
#define th_red cvScalar(0,0,0xff,0)
#define th_blue cvScalar(0xff,0,0,0)
#define th_min(x,y) ((x)<(y)?(y):(x))
#define th_max(x,y) ((x)<(y)?(x):(y))

// some basic statistical functions on arrays
double th_var(double* src, int len);
double th_avg(double* src, int len);
double th_magnitude(double* src, int len);
void th_minus(double* l, double* r, double* result, int len);
void th_plus(double* l, double* r, double* result, int len);

// only creates the image/storage, if it does not already exist, or has different properties (size, depth, channels, blocksize ...)
// returns (1: if image/storage is created) (0: if nothing has been done)
int th_create_image(IplImage** img, CvSize s, int depth, int channels);
int th_create_mem_storage(CvMemStorage** stor, int block_size);
int th_create_hist(CvHistogram** hist, int dims, int* sizes, int type,
		float** ranges, int uniform);

void th_put_text(IplImage* img, const char* text, CvPoint p, CvScalar color);
IplImage* th_plot_hist(CvHistogram* hist, int bins, const char* windowName,
		CvScalar lineColor);
//IplImage* cvhPlotHistogram(CvHistogram* hist, int bins, const char* windowName, CvScalar lineColor, IplImage* in);

// simly saves a CvArr* on the filesystem
int th_save_jpg(const char* path, const CvArr* image, int quality);

// prints a array to system out ala {a,b,c...}
void th_print_array(double* src, int len);

// converts HSV color to a BGR color and back
CvScalar th_hsv2bgr(CvScalar hsv);
CvScalar th_brg2hsv(CvScalar bgr);

// waits until the uses presses ESC (only works if a windo is visible)
void th_wait_esc();
void th_wait(char c);
void th_wait_move_button(PSMove* move, int button);
int th_move_button(PSMove* move, int button);

// querys a frame from the defined capture and waits for "useconds" microseconds before returning
IplImage* th_query_frame(CvCapture* cap);
void th_equalize_image(IplImage* img);

// functions used to modify the parameters of the camera.
// in windows, this is currently implemented by modifying regestry values
// capture device needs to be recreated in order to use the new values
void th_set_camera_params(int AutoAEC, int AutoAGC, int AutoAWB, int Exposure,
		int Gain, int WhiteBalanceB, int WhiteBalanceG, int WhiteBalanceR);
void th_backup_camera_params();
void th_restore_camera_params();

#endif // TRACKER_HELPERS_H

