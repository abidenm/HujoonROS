/*
--------------------------------ggw--------------------------------
ggw:capy spnav.h
	change file name spav_ggw.h
	and remove "USE_X11 PART"
--------------------------------ggw--------------------------------
*/

#ifndef INCLUDE_SPNAV_GGW_H_
#define INCLUDE_SPNAV_GGW_H_

#include <spnav_config.h>

enum {
	SPNAV_EVENT_ANY,	/* used by spnav_remove_events() */
	SPNAV_EVENT_MOTION,
	SPNAV_EVENT_BUTTON	 /* includes both press and release */
};

struct spnav_event_motion {
	int type;
	int x, y, z;
	int rx, ry, rz;
	unsigned int period;
	int *data;
};

struct spnav_event_button {
	int type;
	int press;
	int bnum;
};

typedef union spnav_event {
	int type;
	struct spnav_event_motion motion;
	struct spnav_event_button button;
} spnav_event;


#ifdef __cplusplus
extern "C" {
#endif

/* Open connection to the daemon via AF_UNIX socket.
 * The unix domain socket interface is an alternative to the original magellan
 * protocol, and it is *NOT* compatible with the 3D connexion driver. If you wish
 * to remain compatible, use the X11 protocol (spnav_x11_open, see below).
 * Returns -1 on failure.
 */
int spnav_open(void);

/* Close connection to the daemon. Use it for X11 or AF_UNIX connections.
 * Returns -1 on failure
 */
int spnav_close(void);

/* Retrieves the file descriptor used for communication with the daemon, for
 * use with select() by the application, if so required.
 * If the X11 mode is used, the socket used to communicate with the X server is
 * returned, so the result of this function is always reliable.
 * If AF_UNIX mode is used, the fd of the socket is returned or -1 if
 * no connection is open / failure occured.
 */
int spnav_fd(void);

/* TODO: document */
int spnav_sensitivity(double sens);

/* blocks waiting for space-nav events. returns 0 if an error occurs */
int spnav_wait_event(spnav_event *event);

/* checks the availability of space-nav events (non-blocking)
 * returns the event type if available, or 0 otherwise.
 */
int spnav_poll_event(spnav_event *event);

/* Removes any pending events from the specified type, or all pending events
 * events if the type argument is SPNAV_EVENT_ANY. Returns the number of
 * removed events.
 */
int spnav_remove_events(int type);





#ifdef __cplusplus
}
#endif



#endif /* INCLUDE_SPNAV_GGW_H_ */
