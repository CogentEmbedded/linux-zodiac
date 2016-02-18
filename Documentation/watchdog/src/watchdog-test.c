/*
 * Watchdog Driver Test Program
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/watchdog.h>

int fd;

/*
 * This function simply sends an IOCTL to the driver, which in turn ticks
 * the PC Watchdog card to reset its internal timer so it doesn't trigger
 * a computer reset.
 */
static void keep_alive(void)
{
	int dummy;

	if (ioctl(fd, WDIOC_KEEPALIVE, &dummy))
		fprintf(stderr, "Unable to pet the dog! %s\n", strerror(errno));
}

/*
 * The main program.  Run the program with "-d" to disable the card,
 * or "-e" to enable the card.
 */

static void term(int sig)
{
	close(fd);
	fprintf(stderr, "Stopping watchdog ticks...\n");
	exit(0);
}

void print_usage(void)
{
	fprintf(stderr, "<dev_name> -d to disable, -e to enable, -t <n> to set " \
		"the timeout,\n-p <n> to set the ping rate, and \n");
	fprintf(stderr, "run by itself to tick the card.\n");
	fflush(stderr);
}

int main(int argc, char *argv[])
{
	int flags;
	unsigned int ping_rate = 1;

	if (argc < 2 || argv[1][0] == '-') {
		print_usage();
		return -1;
	}

	fd = open(argv[1], O_WRONLY);

	if (fd == -1) {
		fprintf(stderr, "Watchdog device not enabled! %s\n", strerror(errno));
		fflush(stderr);
		exit(-1);
	}

	if (argc > 2) {
		if (!strncasecmp(argv[2], "-d", 2)) {
			flags = WDIOS_DISABLECARD;
			if (ioctl(fd, WDIOC_SETOPTIONS, &flags))
				fprintf(stderr, "Unable to disable watchdog! %s\n", strerror(errno));
			else
				fprintf(stderr, "Watchdog card disabled.\n");
			fflush(stderr);
			goto end;
		} else if (!strncasecmp(argv[2], "-e", 2)) {
			flags = WDIOS_ENABLECARD;
			if (ioctl(fd, WDIOC_SETOPTIONS, &flags))
				fprintf(stderr, "Unable to enable watchdog! %s\n", strerror(errno));
			else
				fprintf(stderr, "Watchdog card enabled.\n");
			fflush(stderr);
			goto end;
		} else if (!strncasecmp(argv[2], "-t", 2) && argv[3]) {
			flags = atoi(argv[3]);
			if (ioctl(fd, WDIOC_SETTIMEOUT, &flags))
				fprintf(stderr, "Unable to set timeout! %s\n", strerror(errno));
			else
				fprintf(stderr, "Watchdog timeout set to %u seconds.\n", flags);
			fflush(stderr);
			goto end;
		} else if (!strncasecmp(argv[2], "-p", 2) && argv[3]) {
			ping_rate = strtoul(argv[3], NULL, 0);
			fprintf(stderr, "Watchdog ping rate set to %u seconds.\n", ping_rate);
			fflush(stderr);
		} else {
			print_usage();
			goto end;
		}
	}

	fprintf(stderr, "Watchdog Ticking Away!\n");
	fflush(stderr);

	signal(SIGINT, term);

	while(1) {
		keep_alive();
		sleep(ping_rate);
	}

end:
	close(fd);
	return 0;
}
