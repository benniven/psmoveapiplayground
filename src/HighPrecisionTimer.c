#include "HighPrecisionTimer.h"
#include <stdlib.h>

struct _HPTimer {
	double startTimeInMicroSec; // starting time in micro-second
	double endTimeInMicroSec; // ending time in micro-second
	int stopped; // stop flag
#ifdef WIN32
	LARGE_INTEGER frequency; // ticks per second
	LARGE_INTEGER startCount; //
	LARGE_INTEGER endCount; //
#else
	struct timeval startCount;
	struct timeval endCount;
#endif
};

///////////////////////////////////////////////////////////////////////////////
// constructor
///////////////////////////////////////////////////////////////////////////////
HPTimer* createTimer() {
	HPTimer* t = (HPTimer*) calloc(1, sizeof(HPTimer));
#ifdef WIN32
	QueryPerformanceFrequency(&t->frequency);
	t->startCount.QuadPart = 0;
	t->endCount.QuadPart = 0;
#else
	t->startCount.tv_sec = t->startCount.tv_usec = 0;
	t->endCount.tv_sec = t->endCount.tv_usec = 0;
#endif
	t->stopped = 0;
	t->startTimeInMicroSec = 0;
	t->endTimeInMicroSec = 0;
	return t;
}

void releaseTimer(HPTimer* t) {
	free(t);
}

///////////////////////////////////////////////////////////////////////////////
// start timer.
// startCount will be set at this point.
///////////////////////////////////////////////////////////////////////////////
void startTimer(HPTimer* t) {
	t->stopped = 0; // reset stop flag
#ifdef WIN32
	QueryPerformanceCounter(&t->startCount);
#else
	gettimeofday(&t->startCount, NULL);
#endif
}

///////////////////////////////////////////////////////////////////////////////
// stop the timer.
// endCount will be set at this point.
///////////////////////////////////////////////////////////////////////////////
void stopTimer(HPTimer* t) {
	t->stopped = 1; // set timer stopped flag

#ifdef WIN32
	QueryPerformanceCounter(&t->endCount);
#else
	gettimeofday(&t->endCount, NULL);
#endif
}

///////////////////////////////////////////////////////////////////////////////
// compute elapsed time in micro-second resolution.
// other getElapsedTime will call this first, then convert to correspond resolution.
///////////////////////////////////////////////////////////////////////////////
double getElapsedTimeInMicroSec(HPTimer* t) {
#ifdef WIN32
	if (!t->stopped)
		QueryPerformanceCounter(&t->endCount);

	t->startTimeInMicroSec = t->startCount.QuadPart
			* (1000000.0 / t->frequency.QuadPart);
	t->endTimeInMicroSec = t->endCount.QuadPart
			* (1000000.0 / t->frequency.QuadPart);
#else
	if(!t->stopped)
	gettimeofday(&t->endCount, NULL);

	t->startTimeInMicroSec = (t->startCount.tv_sec * 1000000.0) + t->startCount.tv_usec;
	t->endTimeInMicroSec = (t->endCount.tv_sec * 1000000.0) + t->endCount.tv_usec;
#endif

	return t->endTimeInMicroSec - t->startTimeInMicroSec;
}

///////////////////////////////////////////////////////////////////////////////
// divide elapsedTimeInMicroSec by 1000
///////////////////////////////////////////////////////////////////////////////
double getElapsedTimeInMilliSec(HPTimer* t) {
	return getElapsedTimeInMicroSec(t) * 0.001;
}

///////////////////////////////////////////////////////////////////////////////
// divide elapsedTimeInMicroSec by 1000000
///////////////////////////////////////////////////////////////////////////////
double getElapsedTimeInSec(HPTimer* t) {
	return getElapsedTimeInMicroSec(t) * 0.000001;
}

///////////////////////////////////////////////////////////////////////////////
// same as getElapsedTimeInSec()
///////////////////////////////////////////////////////////////////////////////
double getElapsedTime(HPTimer* t) {
	return getElapsedTimeInSec(t);
}
