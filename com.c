/* Serial Port
 * 647
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <assert.h>
#include <stdarg.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <limits.h>
#ifdef USE_FLOCK
#include <sys/file.h>
#endif
#define _GNU_SOURCE
#include <getopt.h>

#include "fdio.h"
#include "term.h"
#include "custbaud.h"


/**********************************************************************/

/* parity modes names */
const char *parity_str[] = {
    [P_NONE] = "none",
    [P_EVEN] = "even",
    [P_ODD] = "odd",
    [P_MARK] = "mark",
    [P_SPACE] = "space",
    [P_ERROR] = "invalid parity mode",
};

/* flow control modes names */
const char *flow_str[] = {
    [FC_NONE] = "none",
    [FC_RTSCTS] = "RTS/CTS",
    [FC_XONXOFF] = "xon/xoff",
    [FC_OTHER] = "other",
    [FC_ERROR] = "invalid flow control mode",
};

/**********************************************************************/



/**********************************************************************/

/* implemented character mappings */

#define M_CRCRLF  (1 << 1)  /* map CR  --> CR + LF */
#define M_DELBS   (1 << 6)  /* map DEL --> BS */


/* default character mappings */
#define M_I_DFL 0
#define M_O_DFL 0
#define M_E_DFL (M_DELBS | M_CRCRLF)

#define CKEY(c) ((c) & 0x1f)



/**********************************************************************/

struct {
    char *port;
    int baud;
    enum flowcntrl_e flow;
    enum parity_e parity;
    int databits;
    int stopbits;
    int lecho;
    int noinit;
    int noreset;
    int hangup;
#if defined (UUCP_LOCK_DIR) || defined (USE_FLOCK)
    int nolock;
#endif
    unsigned char escape;
    int noescape;
    char send_cmd[128];
    char receive_cmd[128];
    int imap;
    int omap;
    int emap;
    char *log_filename;
    char *initstring;
    int exit_after;
    int exit;
    int lower_rts;
    int lower_dtr;
    int raise_rts;
    int raise_dtr;
    int quiet;
} opts = {
    .port = NULL,
    .baud = 9600,
    .flow = FC_NONE,
    .parity = P_NONE,
    .databits = 8,
    .stopbits = 1,
    .lecho = 0,
    .noinit = 0,
    .noreset = 0,
    .hangup = 0,
#if defined (UUCP_LOCK_DIR) || defined (USE_FLOCK)
    .nolock = 0,
#endif
    .escape = CKEY('a'),
    .noescape = 0,
    .send_cmd = "sz -vv",
    .receive_cmd = "rz -vv -E",
    .imap = M_I_DFL,
    .omap = M_O_DFL,
    .emap = M_E_DFL,
    .log_filename = NULL,
    .initstring = NULL,
    .exit_after = -1,
    .exit = 0,
    .lower_rts = 0,
    .lower_dtr = 0,
    .raise_rts = 0,
    .raise_dtr = 0,
    .quiet = 0
};

int sig_exit = 0;

#define STI STDIN_FILENO
#define STO STDOUT_FILENO
#define STE STDERR_FILENO

int tty_fd = -1;
int log_fd = -1;

/* RTS and DTR are usually raised upon opening the serial port (at least
   as tested on Linux, OpenBSD and macOS, but FreeBSD behave different) */
int rts_up = 1;
int dtr_up = 1;

#define TTY_Q_SZ_MIN 256
#ifndef TTY_Q_SZ
#define TTY_Q_SZ 32768
#endif

struct tty_q {
    int sz;
    int len;
    unsigned char *buff;
} tty_q = {
    .sz = 0,
    .len = 0,
    .buff = NULL
};

#define STI_RD_SZ 16
#define TTY_RD_SZ 128

int tty_write_sz;

#define TTY_WRITE_SZ_DIV 10
#define TTY_WRITE_SZ_MIN 8

#define set_tty_write_sz(baud)                          \
    do {                                                \
        tty_write_sz = (baud) / TTY_WRITE_SZ_DIV;       \
        if ( tty_write_sz < TTY_WRITE_SZ_MIN )          \
            tty_write_sz = TTY_WRITE_SZ_MIN;            \
    } while (0)

/**********************************************************************/

#ifdef UUCP_LOCK_DIR

/* use HDB UUCP locks  .. see
 * <http://www.faqs.org/faqs/uucp-internals> for details
 */

char lockname[_POSIX_PATH_MAX] = "";

int
uucp_lockname(const char *dir, const char *file)
{
    char *p, *cp;
    struct stat sb;

    if ( ! dir || *dir == '\0' || stat(dir, &sb) != 0 )
        return -1;

    /* cut-off initial "/dev/" from file-name */
    p = strchr(file + 1, '/');
    p = p ? p + 1 : (char *)file;
    /* replace '/'s with '_'s in what remains (after making a copy) */
    p = cp = strdup(p);
    do { if ( *p == '/' ) *p = '_'; } while(*p++);
    /* build lockname */
    snprintf(lockname, sizeof(lockname), "%s/LCK..%s", dir, cp);
    /* destroy the copy */
    free(cp);

    return 0;
}

int
uucp_lock(void)
{
    int r, fd, pid;
    char buf[16];
    mode_t m;

    if ( lockname[0] == '\0' ) return 0;

    fd = open(lockname, O_RDONLY);
    if ( fd >= 0 ) {
        r = read(fd, buf, sizeof(buf));
        close(fd);
        /* if r == 4, lock file is binary (old-style) */
        pid = (r == 4) ? *(int *)buf : strtol(buf, NULL, 10);
        if ( pid > 0
             && kill((pid_t)pid, 0) < 0
             && errno == ESRCH ) {
            /* stale lock file */
            pinfo("\r\nRemoving stale lock: %s\r\n", lockname);
            sleep(1);
            unlink(lockname);
        } else {
            lockname[0] = '\0';
            errno = EEXIST;
            return -1;
        }
    }
    /* lock it */
    m = umask(022);
    fd = open(lockname, O_WRONLY|O_CREAT|O_EXCL, 0666);
    if ( fd < 0 ) { lockname[0] = '\0'; return -1; }
    umask(m);
    snprintf(buf, sizeof(buf), "%04d\n", getpid());
    write(fd, buf, strlen(buf));
    close(fd);

    return 0;
}

int
uucp_unlock(void)
{
    if ( lockname[0] ) unlink(lockname);
    return 0;
}

#endif /* of UUCP_LOCK_DIR */

/**********************************************************************/

#define HEXBUF_SZ 128
#define HEXDELIM " \r;:-_.,/"

#define hexisdelim(c) ( strchr(HEXDELIM, (c)) != NULL )




/**********************************************************************/



/**********************************************************************/

int
pinfo(const char *format, ...)
{
    va_list args;
    int len;

    if ( opts.quiet ) {
        return 0;
    }
    va_start(args, format);
    len = fd_vprintf(STO, format, args);
    va_end(args);

    return len;
}

void
cleanup (int drain, int noreset, int hup)
{
    if ( tty_fd >= 0 ) {
        /* Print msg if they fail? Can't do anything, anyway... */
        if ( drain )
            term_drain(tty_fd);
        term_flush(tty_fd);
        /* term_flush does not work with some drivers. If we try to
           drain or even close the port while there are still data in
           it's output buffers *and* flow-control is enabled we may
           block forever. So we "fake" a flush, by temporarily setting
           f/c to none, waiting for any data in the output buffer to
           drain, and then reseting f/c to it's original setting. If
           the real flush above does works, then the fake one should
           amount to instantaneously switching f/c to none and then
           back to its propper setting. */
        if ( opts.flow != FC_NONE ) term_fake_flush(tty_fd);
        term_set_hupcl(tty_fd, !noreset || hup);
        term_apply(tty_fd, 1);
        if ( noreset ) {
            pinfo("Skipping tty reset...\r\n");
            term_erase(tty_fd);
#ifdef USE_FLOCK
            /* Explicitly unlock tty_fd before exiting. See
               comments in term.c/term_exitfunc() for more. */
            flock(tty_fd, LOCK_UN);
#endif
            close(tty_fd);
            tty_fd = -1;
        }
    }

//#ifdef LINENOISE
//    cleanup_history();
//#endif
#ifdef UUCP_LOCK_DIR
    uucp_unlock();
#endif
    if ( opts.initstring ) {
        free(opts.initstring);
        opts.initstring = NULL;
    }
    if ( tty_q.buff ) {
        free(tty_q.buff);
        tty_q.buff = NULL;
    }
    if (opts.log_filename) {
        free(opts.log_filename);
        close(log_fd);
    }
}

void
fatal (const char *format, ...)
{
    va_list args;

    fd_printf(STE, "\r\nFATAL: ");
    va_start(args, format);
    fd_vprintf(STE, format, args);
    va_end(args);
    fd_printf(STE, "\r\n");

    cleanup(0 /* drain */, opts.noreset, opts.hangup);

    exit(EXIT_FAILURE);
}

/**********************************************************************/

/* maximum number of chars that can replace a single characted
   due to mapping */
#define M_MAXMAP 4


int
do_map (char *b, char c)
{
    int n = 1;
    b[0] = c;

    assert(n > 0 && n <= M_MAXMAP);

    return n;
}



/**********************************************************************/



/**********************************************************************/

/**********************************************************************/

/**********************************************************************/

#define RUNCMD_ARGS_MAX 32
#define RUNCMD_EXEC_FAIL 126




/**********************************************************************/

int tty_q_push(const char *s, int len) {
    int i, sz, n;
    unsigned char *b;

    for (i = 0; i < len; i++) {
        while (tty_q.len + M_MAXMAP > tty_q.sz) {
            sz = tty_q.sz * 2;
            if ( TTY_Q_SZ && sz > TTY_Q_SZ )
                return i;
            b = realloc(tty_q.buff, sz);
            if ( ! b )
                return i;
            tty_q.buff = b;
            tty_q.sz = sz;
        }
        n = do_map((char *)tty_q.buff + tty_q.len, s[i]);
        tty_q.len += n;
    }

    return i;
}



/**********************************************************************/
#define CMD_NUM 10

/* loop-exit reason */
enum le_reason {
    LE_CMD,
    LE_IDLE,
    LE_STDIN,
    LE_SIGNAL
};

enum le_reason
loop(void)
{
    fd_set rdset, wrset;
    int r, n;

	FD_ZERO(&rdset);
	FD_ZERO(&wrset);
	FD_SET(tty_fd, &rdset);
	FD_SET(tty_fd, &wrset);


    char* cmd[CMD_NUM]={"DCC 1=67:80\n",
    		"UCA 1,1,0\n",
    		"DCC 1=67:74;2=155:162\n",
    		"UCA 1,1,0;2,1,0\n",
    		"DCC 1=67:74;2=155:162;3=163:170\n",
    		"UCA 1,1,0;2,1,0;3,2,0\n",
    		"DCC 1=67:74;2=155:162;3=163:170;4=171:178\n",
    		"UCA 1,1,0;2,1,0;3,2,0;4,2,0\n",
    		"DCC 1=67:74;2=155:162;3=163:170;4=171:178;5=180:181;6=240:250\n",
			"UCA 1,1,0;2,1,0;3,2,0;4,2,0;5,3,0;6,3,0\n"};
    struct timeval starttime[CMD_NUM];
    struct timeval end[CMD_NUM];
    int round =0;

    for (round = 0; round <10; round++){
    	printf("Round = %d\n",round);
    	gettimeofday(&starttime[round%CMD_NUM],NULL);
		if ( tty_q_push(cmd[round%CMD_NUM], strlen(cmd[round%CMD_NUM])) != strlen(cmd[round%CMD_NUM]) )
			printf("Number do not match\n");
		if ( tty_q.len ) {
			FD_SET(tty_fd, &wrset);
		}

		if ( FD_ISSET(tty_fd, &wrset) ) {
			/* write to port */
			int sz;
			sz = (tty_q.len < tty_write_sz) ? tty_q.len : tty_write_sz;
			do {
				n = write(tty_fd, tty_q.buff, sz);
			} while ( n < 0 && errno == EINTR );
			printf("the buffer written is\n%.*s\n",n,tty_q.buff);
			if ( n <= 0 )
				fatal("write to port failed: %s", strerror(errno));
			memmove(tty_q.buff, tty_q.buff + n, tty_q.len - n);
			tty_q.len -= n;


			FD_ZERO(&rdset);
			FD_SET(tty_fd,&rdset);
			r = select(tty_fd + 1, &rdset, NULL, NULL, NULL);
			if(r > 0 && FD_ISSET(tty_fd,&rdset)){
				printf("Something has returned from serial port \n");
				char tbuff_rd[TTY_RD_SZ];
				do {
					n = read(tty_fd, &tbuff_rd, sizeof(tbuff_rd));
				} while (n < 0 && errno == EINTR);
				printf("Read after select is \n%.*s\n",n,tbuff_rd);
			}
			FD_ZERO(&rdset);
			FD_SET(tty_fd,&rdset);
			r = select(tty_fd + 1, &rdset, NULL, NULL, NULL);
			if(r > 0 && FD_ISSET(tty_fd,&rdset)){
				printf("Something has returned from serial port \n");
				char tbuff_rd[TTY_RD_SZ];
				do {
					n = read(tty_fd, &tbuff_rd, sizeof(tbuff_rd));
				} while (n < 0 && errno == EINTR);
				printf("Read after select is \n%.*s\n",n,tbuff_rd);
			}
		}

		gettimeofday(&end[round%CMD_NUM], NULL );
		long timeuse = 1000000*( end[round%CMD_NUM].tv_sec - starttime[round%CMD_NUM].tv_sec ) + end[round%CMD_NUM].tv_usec - starttime[round%CMD_NUM].tv_usec;
		double runtime = timeuse/1000.0;
		printf("running time is %6.2f \n\n\n",runtime);
	        sleep(10000);

    }
    return LE_SIGNAL;
}

/**********************************************************************/





/**********************************************************************/



/**********************************************************************/


/**********************************************************************/


int
main (int argc, char *argv[])
{
    int xcode = EXIT_SUCCESS;
    int ler;
    int r;

    opts.baud = 115200;
    opts.port = "/dev/ttyUSB0";

    r = term_lib_init();
    if ( r < 0 )
        fatal("term_lib_init failed: %s", term_strerror(term_errno, errno));//need

#ifdef UUCP_LOCK_DIR
    if ( ! opts.nolock ) uucp_lockname(UUCP_LOCK_DIR, opts.port);
    if ( uucp_lock() < 0 )
        fatal("cannot lock %s: %s", opts.port, strerror(errno));
#endif

    tty_fd = open(opts.port, O_RDWR | O_NONBLOCK | O_NOCTTY);
    if (tty_fd < 0)
        fatal("cannot open %s: %s", opts.port, strerror(errno));

#ifdef USE_FLOCK
    if ( ! opts.nolock ) {
        r = flock(tty_fd, LOCK_EX | LOCK_NB);
        if ( r < 0 )
            fatal("cannot lock %s: %s", opts.port, strerror(errno));
    }
#endif


	r = term_set(tty_fd,
				 1,              /* raw mode. */
				 opts.baud,      /* baud rate. */
				 opts.parity,    /* parity. */
				 opts.databits,  /* data bits. */
				 opts.stopbits,  /* stop bits. */
				 opts.flow,      /* flow control. */
				 1,              /* local or modem */
				 !opts.noreset); /* hup-on-close. */
    if ( r < 0 )
        fatal("failed to add port: %s", term_strerror(term_errno, errno));

    r = term_apply(tty_fd, 0);
    if ( r < 0 )
        fatal("failed to config port: %s",
              term_strerror(term_errno, errno));


    set_tty_write_sz(term_get_baudrate(tty_fd, NULL));

    /* Allocate output buffer with initial size */
    tty_q.buff = calloc(TTY_Q_SZ_MIN, sizeof(*tty_q.buff));
    if ( ! tty_q.buff )
        fatal("out of memory");
    tty_q.sz = TTY_Q_SZ_MIN;
    tty_q.len = 0;

    pinfo("Terminal ready\r\n");

    /* Enter main processing loop */
    ler = loop();

    /* Terminating picocom */
    pinfo("\r\n");
    pinfo("Terminating...\r\n");

    if ( ler == LE_CMD || ler == LE_SIGNAL )
        cleanup(0 /* drain */, opts.noreset, opts.hangup);
    else
        cleanup(1 /* drain */, opts.noreset, opts.hangup);

    if ( ler == LE_SIGNAL ) {
        pinfo("Serial process was killed\r\n");
        xcode = EXIT_FAILURE;
    }

    return xcode;
}

/**********************************************************************/

/*
 * Local Variables:
 * mode:c
 * tab-width: 4
 * c-basic-offset: 4
 * End:
 */
