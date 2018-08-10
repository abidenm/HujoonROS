
/*
--------------------------------ggw--------------------------------
ggw:capy spnav.c
	change file name spav_ggw.cpp
	and remove "USE_X11 PART"
--------------------------------ggw--------------------------------
*/

/*
This file is part of libspnav, part of the spacenav project (spacenav.sf.net)
Copyright (C) 2007-2010 John Tsiombikas <nuclear@member.fsf.org>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. The name of the author may not be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/select.h>
#include "spnav_ggw.h"

#define SPNAV_SOCK_PATH "/var/run/spnav.sock"

#define IS_OPEN		(sock != -1)



struct event_node {
	spnav_event event;
	struct event_node *next;
};

/* only used for non-X mode, with spnav_remove_events */
static struct event_node *ev_queue, *ev_queue_tail;

/* AF_UNIX socket used for alternative communication with daemon */
static int sock = -1;


int spnav_open(void)
{
	int s;
	struct sockaddr_un addr;

	if(IS_OPEN) {
		return -1;
	}

	if(!(ev_queue = (struct event_node *) malloc(sizeof *ev_queue))) {
		return -1;
	}
	ev_queue->next = 0;
	ev_queue_tail = ev_queue;

	if((s = socket(PF_UNIX, SOCK_STREAM, 0)) == -1) {
		return -1;
	}

	memset(&addr, 0, sizeof addr);
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, SPNAV_SOCK_PATH, sizeof(addr.sun_path));


	if(connect(s, (struct sockaddr*)&addr, sizeof addr) == -1) {
		perror("connect failed");
		close(s);
		return -1;
	}

	sock = s;
	return 0;
}


int spnav_close(void)
{
	if(!IS_OPEN) {
		return -1;
	}

	if(sock) {
		while(ev_queue) {
			void *tmp = ev_queue;
			ev_queue = ev_queue->next;
			free(tmp);
		}

		close(sock);
		sock = -1;
		return 0;
	}
	return -1;
}




int spnav_sensitivity(double sens)
{
	if(sock) {
		ssize_t bytes;
		float fval = sens;

		while((bytes = write(sock, &fval, sizeof fval)) <= 0 && errno == EINTR);
		if(bytes <= 0) {
			return -1;
		}
		return 0;
	}

	return -1;
}

int spnav_fd(void)
{
	return sock;
}


/* Checks both the event queue and the daemon socket for pending events.
 * In either case, it returns immediately with true/false values (doesn't block).
 */
static int event_pending(int s)
{
	fd_set rd_set;
	struct timeval tv;

	if(ev_queue->next) {
		return 1;
	}

	FD_ZERO(&rd_set);
	FD_SET(s, &rd_set);

	/* don't block, just poll */
	tv.tv_sec = tv.tv_usec = 0;

	if(select(s + 1, &rd_set, 0, 0, &tv) > 0) {
		return 1;
	}
	return 0;
}

/* If there are events waiting in the event queue, dequeue one and
 * return that, otherwise read one from the daemon socket.
 * This might block unless we called event_pending() first and it returned true.
 */
static int read_event(int s, spnav_event *event)
{
	int i, rd;
	int data[8];

	/* if we have a queued event, deliver that one */
	if(ev_queue->next) {
		struct event_node *node = ev_queue->next;
		ev_queue->next = ev_queue->next->next;

		/* dequeued the last event, must update tail pointer */
		if(ev_queue_tail == node) {
			ev_queue_tail = ev_queue;
		}

		memcpy(event, &node->event, sizeof *event);
		free(node);
		return event->type;
	}

	/* otherwise read one from the connection */
	do {
		rd = read(s, data, sizeof data);
	} while(rd == -1 && errno == EINTR);

	if(rd <= 0) {
		return 0;
	}

	if(data[0] < 0 || data[0] > 2) {
		return 0;
	}
	event->type = data[0] ? SPNAV_EVENT_BUTTON : SPNAV_EVENT_MOTION;

	if(event->type == SPNAV_EVENT_MOTION) {
		event->motion.data = &event->motion.x;
		for(i=0; i<6; i++) {
			event->motion.data[i] = data[i + 1];
		}
		event->motion.period = data[7];
	} else {
		event->button.press = data[0] == 1 ? 1 : 0;
		event->button.bnum = data[1];
	}

	return event->type;
}


int spnav_wait_event(spnav_event *event)
{
	if(sock) {
		if(read_event(sock, event) > 0) {
			return event->type;
		}
	}
	return 0;
}

int spnav_poll_event(spnav_event *event)
{
	if(sock) {
		if(event_pending(sock)) {
			if(read_event(sock, event) > 0) {
				return event->type;
			}
		}
	}
	return 0;
}

/* Appends an event to an event list.
 * Tailptr must be a pointer to the tail pointer of the list. NULL means
 * append to the global event queue.
 */
static int enqueue_event(spnav_event *event, struct event_node **tailptr)
{
	struct event_node *node;
	if(!(node = (struct event_node * ) malloc(sizeof *node))) {
		return -1;
	}

	node->event = *event;
	node->next = 0;

	if(!tailptr) {
		tailptr = &ev_queue_tail;
	}

	(*tailptr)->next = node;
	*tailptr = node;
	return 0;
}

int spnav_remove_events(int type)
{
	int rm_count = 0;

	if(sock) {
		struct event_node *tmplist, *tmptail;

		if(!(tmplist = tmptail = (struct event_node *) malloc(sizeof *tmplist))) {
			return -1;
		}
		tmplist->next = 0;

		/* while there are events in the event queue, or the daemon socket */
		while(event_pending(sock)) {
			spnav_event event;

			read_event(sock, &event);	/* remove next event */
			if(event.type != type) {
				/* We don't want to drop this one, wrong type. Keep the event
				 * in the temporary list, for deferred reinsertion
				 */
				enqueue_event(&event, &tmptail);
			} else {
				rm_count++;
			}
		}

		/* reinsert any events we removed that we didn't mean to */
		while(tmplist->next) {
			struct event_node *node = tmplist->next;

			enqueue_event(&node->event, 0);

			free(tmplist);
			tmplist = node;
		}
		free(tmplist);

		return rm_count;
	}
	return 0;
}

