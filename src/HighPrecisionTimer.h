#ifndef HIGH_PRECISION_TIMER_H_DEF
#define HIGH_PRECISION_TIMER_H_DEF

#ifdef WIN32   // Windows system specific
#include <windows.h>
#else          // Unix based system specific
#include <sys/time.h>
#endif

/* Opaque data type for the PS Move internal data */
struct _HPTimer;
typedef struct _HPTimer HPTimer;

HPTimer* createTimer(); // start timer
void releaseTimer(HPTimer* t); // start timer
void startTimer(HPTimer* t); // start timer
void stopTimer(HPTimer* t); // stop the timer
double getElapsedTime(HPTimer* t); // get elapsed time in second
double getElapsedTimeInSec(HPTimer* t); // get elapsed time in second (same as getElapsedTime)
double getElapsedTimeInMilliSec(HPTimer* t); // get elapsed time in milli-second
double getElapsedTimeInMicroSec(HPTimer* t); // get elapsed time in micro-second

#endif // HIGH_PRECISION_TIMER_H_DEF
