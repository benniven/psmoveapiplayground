#include <stdio.h>
#include "opencv2/highgui/highgui_c.h"
#ifdef WIN32
#    include <windows.h>
#endif

#include "psmove.h"
#include "psmove_tracker.h"
#include "tracker/tracker_helpers.h"

PSMove* connectController(int id) {
	PSMove *move;
	move = psmove_connect_by_id(id);
	if (move == NULL) {
		printf("Could not connect to default Move controller.\n"
				"Please connect one via USB or Bluetooth.\n");
		exit(1);
	}
	return move;
}

int main(int arg, char** args) {
	int i;
	int numCtrls = psmove_count_connected();
	PSMove* controllers[numCtrls];

	printf("%s", "### Trying to init PSMoveTracker...");
	PSMoveTracker* tracker = psmove_tracker_new(0);
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

		if (key == 'p')
			th_wait('p');

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
	return 0;
}

