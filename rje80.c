//  A program to emulate an IBM 2780/3780 RJE station over a simulated
//  bisync line using the BSC protocol as implemented by the Hercules/370 2703
//  device.  This program will not work with "real" bisync hardware.

#include <stdio.h>
#if defined (_WIN32)	// Windows
#include <winsock2.h>
#include <conio.h>
#include <io.h>
#include <signal.h>
#include <windows.h>
#else					// Linux
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <termios.h>
#include <netdb.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#endif
#include <errno.h>

// Prototypes 

int do_char(unsigned char c);
int do_input(int count);
int execute();
int nexttoken();
int gettoken(int upper);
int InitSockets();
int CloseSockets();
int connecthost();
int sendfile(char *command);
int get_buffer();
unsigned char read_byte();
int read_poll(int sec, int usec);
int read_data(int sec, int usec, int mode);
int clear_input_buffer();
int clear_input_record();
int write_record();
int printer_function(unsigned char func);
int write_buffer();
int send_ack(char ack);
int ttyinit();
int ttyclose();
int ttygets(char *str);
int ttyread(unsigned char *buf);
int ttychar(char c);
int ttystr(char *msg);
int rjesleep(int t);
char *translate_to_ebcdic (unsigned char *str);
char *translate_to_ascii (unsigned char *str);

// Global control items

// Status values for overall status flag

#define NOLINK -1		/*  not connected, not yet trying */
#define INITIAL_WAIT 0		/*  Connected waiting for signon command */
#define IDLE 1			/*  Connected, signed on, idle */
#define SENDING 5		/*  Data being sent to host */
#define RECEIVING 6		/*  data being received from host */
#define SHUTDOWN 9		/*  emulator shutdown */

int status = NOLINK;		/* Emulator overall status */

int transparent = 0;		/* 1 when in transparent mode */

int debugit = 0;		/* debug flag, set by -d on command */
		
char inethost[128];		/* Internet host (from command line) */
int inetport;			/* Internet port (ditto) */				
char macro[8192];		/* The macro buffer */
int macro_size = 0;		/* Size of macro in biffer */
int macro_ctr = 0;		/* chars in macro buffer */
char command[128];		/* A command to the Emulator */
char token[64];			/* A token from the command */
int comctr;			/* used by the parser */
int comlen = 0;			/* Length of command */
int prompt = 0;			/* Prompt flag */
int pollflag = 2;		/* Polling control */
int pollctr = 0;
unsigned char lastack = 0;	/* To flip between ACK0 and ACK1 */
char reader[80];		/* Reader filename */
int reader_recl = 80;		/* record length */
int reader_fmt = 0;		/* 0=ascii 1=ebcdic */
char print[80];			/* Print filename */
int print_recl = 132;		/* max printer width */
char htabs[256];		/* Horizontal tabs storage */
char punch[80];			/* Punch filename */
int punch_recl = 80;		/* Punch recl (used for ebcdic only) */
int punch_fmt = 0;		/* 0=ascii 1=ebcdic */
int punchform = 0;		/* Punch format 0=ascii 1=ebcdic */
int readform = 0;		/* Reader format 0=ascii 1=ebcdic */
int device_select = 0;		/* 0 = printer 1 = punch */

int print_open = 0;		/* need to open = 1 */
int punch_open = 0;

FILE *printfd;			/* FDs for files */
FILE *punchfd;
FILE *readerfd;
FILE *tracefd;
FILE *rcfd;

char tracefile[80];

char opt_user[32];		/* Username */
int opt_poll = 0;		/* Poll when idle */
int opt_trn = 0;		/* Transparency on all writes */
int opt_pause = 0;		/* -1 = every FF, 0 = none, > 0 = pause */
int opt_copy = 1;		/* 1 = display printer output, 0 = no disp */
int opt_os = 0;			/* 0 = generic */
				/* 1 = VM/370 RSCS */
				/* 2 = JES2 */
				/* 3 = JES3 */
				/* 4 = DOS/VS */
				/* 5 = RES (VS1) */
				/* 6 = OS/360 */
// TTY-related data

unsigned char ttybuf[1];
#if defined (_WIN32)
#else
struct termios cmdtty, runtty;
#endif

// Socket-related stuff

int sockfd;			/* The socket itself */
char hname[80];			/* given host name */
int host_ip;
int inport;			/* our input port */
int outport;			/* the hosts port */

// Communications buffers

unsigned char phybuffer[8192];	/* Raw socket data */
int phy_ctr = 0;		/* Size in physical buffer */
unsigned char line_in[1024];	/* Input data from line */
int line_in_ctr = 0;		/* How many are stacked up */
unsigned char record_in[1024];	/* Record data */
int record_ctr = 0;		/* How many bytes in record */
unsigned char line_out[1024];	/* Output data to be sent */
int line_out_size = 0;		/* How many to actually send */
char signon[80];		/* we build this */

// BSC control characters

unsigned char SOH = 0x01;
unsigned char STX = 0x02;
unsigned char ETX = 0x03;
unsigned char DLE = 0x10;
unsigned char DC1 = 0x11;
unsigned char DC2 = 0x12;
unsigned char NL  = 0x15;
unsigned char EM  = 0x19;
unsigned char IGS = 0x1d;
unsigned char IRS = 0x1e;
unsigned char ITB = 0x1f;
unsigned char ETB = 0x26;
unsigned char ESC = 0x27;
unsigned char ENQ = 0x2d;
unsigned char SYN = 0x32;
unsigned char EOT = 0x37;
unsigned char NAK = 0x3d;
unsigned char DC3 = 0x5d;
unsigned char ACK1 = 0x61;
unsigned char WABT = 0x6b;
unsigned char ACK0 = 0x70;
unsigned char EPAD = 0xff;
unsigned char SPAD = 0xaa;
unsigned char RVI = 0x7c;

// Translation tables (from Hercules)

static unsigned char
ascii_to_ebcdic[] = {
    "\x00\x01\x02\x03\x37\x2D\x2E\x2F\x16\x05\x25\x0B\x0C\x0D\x0E\x0F"
    "\x10\x11\x12\x13\x3C\x3D\x32\x26\x18\x19\x1A\x27\x22\x1D\x35\x1F"
    "\x40\x5A\x7F\x7B\x5B\x6C\x50\x7D\x4D\x5D\x5C\x4E\x6B\x60\x4B\x61"
    "\xF0\xF1\xF2\xF3\xF4\xF5\xF6\xF7\xF8\xF9\x7A\x5E\x4C\x7E\x6E\x6F"
    "\x7C\xC1\xC2\xC3\xC4\xC5\xC6\xC7\xC8\xC9\xD1\xD2\xD3\xD4\xD5\xD6"
    "\xD7\xD8\xD9\xE2\xE3\xE4\xE5\xE6\xE7\xE8\xE9\xAD\xE0\xBD\x5F\x6D"
    "\x79\x81\x82\x83\x84\x85\x86\x87\x88\x89\x91\x92\x93\x94\x95\x96"
    "\x97\x98\x99\xA2\xA3\xA4\xA5\xA6\xA7\xA8\xA9\xC0\x6A\xD0\xA1\x07"
    "\x68\xDC\x51\x42\x43\x44\x47\x48\x52\x53\x54\x57\x56\x58\x63\x67"
    "\x71\x9C\x9E\xCB\xCC\xCD\xDB\xDD\xDF\xEC\xFC\xB0\xB1\xB2\xB3\xB4"
    "\x45\x55\xCE\xDE\x49\x69\x04\x06\xAB\x08\xBA\xB8\xB7\xAA\x8A\x8B"
    "\x09\x0A\x14\xBB\x15\xB5\xB6\x17\x1B\xB9\x1C\x1E\xBC\x20\xBE\xBF"
    "\x21\x23\x24\x28\x29\x2A\x2B\x2C\x30\x31\xCA\x33\x34\x36\x38\xCF"
    "\x39\x3A\x3B\x3E\x41\x46\x4A\x4F\x59\x62\xDA\x64\x65\x66\x70\x72"
    "\x73\xE1\x74\x75\x76\x77\x78\x80\x8C\x8D\x8E\xEB\x8F\xED\xEE\xEF"
    "\x90\x9A\x9B\x9D\x9F\xA0\xAC\xAE\xAF\xFD\xFE\xFB\x3F\xEA\xFA\xFF"
};

static unsigned char
ebcdic_to_ascii[] = {
    "\x00\x01\x02\x03\xA6\x09\xA7\x7F\xA9\xB0\xB1\x0B\x0C\x0D\x0E\x0F"
    "\x10\x11\x12\x13\xB2\x0A\x08\xB7\x18\x19\x1A\xB8\xBA\x1D\xBB\x1F"
    "\xBD\xC0\x1C\xC1\xC2\x0A\x17\x1B\xC3\xC4\xC5\xC6\xC7\x05\x06\x07"
    "\xC8\xC9\x16\xCB\xCC\x1E\xCD\x04\xCE\xD0\xD1\xD2\x14\x15\xD3\xFC"
    "\x20\xD4\x83\x84\x85\xA0\xD5\x86\x87\xA4\xD6\x2E\x3C\x28\x2B\xD7"
    "\x26\x82\x88\x89\x8A\xA1\x8C\x8B\x8D\xD8\x21\x24\x2A\x29\x3B\x5E"
    "\x2D\x2F\xD9\x8E\xDB\xDC\xDD\x8F\x80\xA5\x7C\x2C\x25\x5F\x3E\x3F"
    "\xDE\x90\xDF\xE0\xE2\xE3\xE4\xE5\xE6\x60\x3A\x23\x40\x27\x3D\x22"
    "\xE7\x61\x62\x63\x64\x65\x66\x67\x68\x69\xAE\xAF\xE8\xE9\xEA\xEC"
    "\xF0\x6A\x6B\x6C\x6D\x6E\x6F\x70\x71\x72\xF1\xF2\x91\xF3\x92\xF4"
    "\xF5\x7E\x73\x74\x75\x76\x77\x78\x79\x7A\xAD\xA8\xF6\x5B\xF7\xF8"
    "\x9B\x9C\x9D\x9E\x9F\xB5\xB6\xAC\xAB\xB9\xAA\xB3\xBC\x5D\xBE\xBF"
    "\x7B\x41\x42\x43\x44\x45\x46\x47\x48\x49\xCA\x93\x94\x95\xA2\xCF"
    "\x7D\x4A\x4B\x4C\x4D\x4E\x4F\x50\x51\x52\xDA\x96\x81\x97\xA3\x98"
    "\x5C\xE1\x53\x54\x55\x56\x57\x58\x59\x5A\xFD\xEB\x99\xED\xEE\xEF"
    "\x30\x31\x32\x33\x34\x35\x36\x37\x38\x39\xFE\xFB\x9A\xF9\xFA\xFF"
};

// Mainline code - - start here.

main(int argc, char *argv[])
{
	int stat, i, rstat;
	int tries = 0;
	int startar = 1;
	int gotdle = 0;
	int gotstx = 0;
	unsigned char buf[1], ch, sho[16];

	strcpy(inethost, "");
	strcpy(opt_user, "");
	strcpy(tracefile, "");

	if (argc > 1) {
		if (strcmp(argv[1], "-d") == 0) {
			printf("We are in debug mode.\n");
			debugit = 1;
			startar = 2;
		}
		if (argc >= startar) {
			strcpy(inethost, argv[startar]);
			startar++;
		}
		if (argc >= startar) {	
			inetport=atoi(argv[startar]);
			outport = inetport;
		}	
	}		

	ttyinit();
	ttystr("\r\nRJE80 IBM 3780 Emulator Version 0.29");
	strcpy(print, "");	/* default output files to display */
	strcpy(punch, "punch.txt");

	if (InitSockets() == -1) {
		status = SHUTDOWN;
	}

	if (strlen(inethost) > 0 &&
		status != SHUTDOWN) {	/* host given in command line ? */
		connecthost();		/* YEAH */
	}	

	// See if there's an rje80.rc file, if so, read it and stuff the
	// characters in it into the macro buffer

	if ((rcfd = fopen("rje80.rc", "r")) != NULL) {
		macro_size = 0;
		while (1) {
			if (feof(rcfd))
				break;
			macro[macro_size] = fgetc(rcfd);
			macro_size++;
		}
		macro_ctr = 0;
		fclose(rcfd);
	}

	// This is the main loop.  It cycles continuously, looking for 
	// events to process.  Events such as a character from the local
	// keyboard, data arriving on the communications socket, or 
	// a timer expiring.
	
	while (status != SHUTDOWN) {
		if (!prompt) {			/* Need a new prompt? */
			for (i=0; i < 128; i++) command[i] = 0;
			ttychar('\n');
			ttychar('\r');
			ttychar(')');
			ttychar(' ');
			prompt = 1;
			comlen = 0;
		}	
		stat = ttyread(buf);
		if (stat) {
			do_char(buf[0]);
		}
		if (status == IDLE) {
			if (read_poll(0, 10000) == 1) { /* Have data? */
				read_data(3, 0, 0);		/* Yes -- go get it */
				while (ch = read_byte()) {	/* Process data */
					if (ch == ENQ) {	/* he wants to send us something */
						prompt = 1;		
						pollflag = 0;
						status = RECEIVING;
						clear_input_record();
						lastack = ACK0;
						send_ack(ACK0);
						continue;
					}
					if (ch == EPAD || ch == SYN) {	/* fillers - ignore */
						continue;
					}	
					if (ch == DLE) {	/* Probably poll response */
						pollflag = 1;
						continue;
					}
					if (ch == ACK0 || ch == ACK1) {	/* it is a poll response */
						pollflag = 2;	/* set up to poll again */
						continue;
					}
					// WE don't know what it is ...
					// Ignore it for now ...
					if (debugit == 1) {
						ttystr("\r\nOdd data found: ");
						sprintf(sho, "%2x", ch);
						ttystr(sho);
						ttystr("\r\n");
					}
				}
			} else {	// no data .. so, we consider polling
				if (pollflag == 2 && pollctr > 50) {
					if (opt_poll) {
						line_out[0] = line_out[1] = line_out[2] = DLE;
						line_out[3] = ACK0;
						line_out_size = 4;
						write_buffer();
					}
					pollctr = 0;
				} else {
					pollctr++;
				}
			}
		}
		if (status == RECEIVING) {
			if (line_in_ctr < 1)
				rstat = read_data(3, 0, 1);
			if (rstat == -1) {
				ttystr("\r\nRJE127S The line has disconnected.\r\n");
				status = IDLE;
				continue;
			}
			ch = read_byte();
			if (ch != 0) {
				if (transparent) {	// Handle chars in transparent mode */
					if (gotdle == 1 && ch == EOT) {	/* End of file */
						if (device_select == 0 && strlen(print) != 0) {
							ttystr("EOT\r\n");
							prompt = 0;
						}	
						if (device_select == 1 && strlen(punch) != 0) {
							ttystr("EOT\r\n");
							prompt = 0;
						}	
						send_ack(0);
						status = IDLE;
						pollflag = 2;
						pollctr = 0;
						prompt = 0;
						gotdle = gotstx = 0;
						continue;
					}
					if (gotdle == 1 && ch == STX) {	/* Start of text */
						if (gotdle)
							transparent = 1;
						gotstx = 1;
						gotdle = 0;
						continue;
					}
					if (gotdle == 0 && ch == DLE) {	/* DLE what follows is special */
						gotdle = 1;
						gotstx = 0;
						continue;
					}
					if (gotdle == 1 && ch == DC1) {	/* DC1: Select printer if after an STX */
						if (gotstx == 1) {
							device_select = 0;
							if (strlen(print) != 0) {
								ttystr("\r\nRJE001I Receiving print data...");
							} else {
								ttystr("\r\n");
							}
						}
						gotdle = gotstx = 0;
						continue;
					}	
					if (gotdle == 1 && ch == DC2) {	/* DC2: Select punch if after an STX */
						if (gotstx == 1) {
							device_select = 1;
							ttystr("\r\nRJE001I Receiving punch data...");
						}
						continue;
					}
					if (gotdle == 1 && ch == ETB && record_ctr > 0) {	// DOS bug, some recs have only ETB
						write_record();
					}
					if (gotdle == 1 && ch == ETX) {	/* End of block -- */
						send_ack(0);		/* Acknowledge */
						gotdle = gotstx = 0;
						transparent = 0;	/* transparent off */
						continue;
					}	
					if (gotdle == 1 && ch == ETB) {	/* End of block -- */
						send_ack(0);		/* Acknowledge */
						gotdle = gotstx = 0;
						continue;
					}	
					if (gotdle == 1 && (ch == IRS ||
					    ch == NL)) {			/* End of record - write */
						write_record();		
						gotdle = gotstx = 0;
						continue;	
					}
					if (gotdle == 1 && ch == ENQ) {
						send_ack(0);		/* Acknowledge */
						gotdle = gotstx = 0;
						continue;		
					}
					
					record_in[record_ctr] = ch;	/* data - save it */
					record_ctr++;
					gotdle = gotstx = 0;

				} else {
					if (ch == EOT) {	/* End of file */
						if (device_select == 0 && strlen(print) != 0) {
							ttystr("EOT\r\n");
							prompt = 0;
						}	
						if (device_select == 1 && strlen(punch) != 0) {
							ttystr("EOT\r\n");
							prompt = 0;
						}	
						send_ack(0);
						status = IDLE;
						pollflag = 2;
						pollctr = 0;
						prompt = 0;
						gotdle = gotstx = 0;
						continue;
					}
					if (ch == STX) {	/* Start of text */
						if (gotdle)
							transparent = 1;
						gotstx = 1;
						gotdle = 0;
						continue;
					}
					if (ch == DLE) {	/* DLE what follows is special */
						gotdle = 1;
						gotstx = 0;
						continue;
					}
					if (ch == DC1) {	/* DC1: Select printer if after an STX */
						if (gotstx == 1) {
							device_select = 0;
							if (strlen(print) != 0) {
								ttystr("\r\nRJE001I Receiving print data...");
							} else {
								ttystr("\r\n");
							}
						}
						gotdle = gotstx = 0;
						continue;
					}	
					if (ch == DC2) {	/* DC2: Select punch if after an STX */
						if (gotstx == 1) {
							device_select = 1;
							ttystr("\r\nRJE001I Receiving punch data...");
						}
						continue;
					}
					if (ch == ETB && record_ctr > 0) {	// DOS bug, some recs have only ETB
						write_record();
					}
					if (ch == ETX ||		/* End of block -- */
					    ch == ETB) {
						send_ack(0);		/* Acknowledge */
						gotdle = gotstx = 0;
						continue;
					}	
					if (ch == IRS ||
					    ch == NL) {			/* End of record - write */
						write_record();		
						gotdle = gotstx = 0;
						continue;	
					}
					if (ch == ENQ) {
						send_ack(0);		/* Acknowledge */
						gotdle = gotstx = 0;
						continue;		
					}
					
					// WHAT DO TO WITH ALL THE OTHER CONTROL CHARS?
					// store'em, thats what ... until we know better ...
					// NOTE: thats what we want to do with ESC sequences
					// ...we handle them when we output the record

					record_in[record_ctr] = ch;	/* data - save it */
					record_ctr++;
					gotdle = gotstx = 0;
				}
			}
		}	
		rjesleep(10);			/* Allow local CPU some cycles */
	}

	if (strlen(tracefile) > 0)

		fclose(tracefd);
	CloseSockets();
	ttyclose();
	printf("Goodbye...\n");
}

// A character typed - store it, or execute the command

int do_char(unsigned char c)
{
	
	if (c == 8) {					/* Backspace */
		if (comlen > 0) {
			comlen--;
			command[comlen] = ' ';
			ttystr("\b \b");
		}	
		return (0);
	}	
	if (c == '\n' || c == '\r') {			/* Line terminator */
		execute();
		return (0);
	}
	if (comlen > 128) {
		ttystr("\n\rRJE002W Command too long, sorry, only enter, del, or esc allowed\r\n");
		return (0);
	}	
	command[comlen] = c;				/* Just a char - store it */
	comlen++;	
	ttychar(c);
	return (0);
}

// This function executes commands from the user

int execute() 
{
	char passw[32];
	char reclen[32];
	char shell[128];
	char cmd[128];
	char cmdline[128];
	int i, rc, retry, savep;
	
	
	prompt = 0;
	if (comlen < 1)
		return (0);
	comctr = 0;
	while ((command[comctr] == ' ' || command[comctr] < 0)
		&& comctr < comlen) {
		comctr++;
	}
	if (comctr >= comlen)
		return (0);
	gettoken(1);
	if (strcmp(token, "OPEN") == 0 || strcmp(token, "START") == 0 ||
		strcmp(token, "OPE") == 0 ||
		strcmp(token, "OP") == 0 ||
		strcmp(token, "O") == 0) {
		if (nexttoken() == 1) {
			ttystr("\r\nRJE121A The hostname is missing, try again\r\n");
			return (0);
		}
		if (status > INITIAL_WAIT) {
			ttystr("\n\rRJE122A Connection is already open. CLOSE will close it.\r\n");
			return (0);
		}	
		gettoken(0);
		strcpy(inethost, token);
		if (nexttoken() == 1) {
			ttystr("\r\nRJE123A Remote host port is missing, try again.\r\n");
			return (0);
		}
		gettoken(0);
		inetport=atoi(token);
		if (nexttoken() == 0) {
			ttystr("\r\nRJE124W Extra data after the port number is ignored\r\n");
		}
		connecthost();
		return (0);
	}	
	if (strcmp(token, "SHELL") == 0 || strcmp(token, "SH") == 0 ||
		strcmp(token, "!") == 0) {
		ttystr("\r\n\r\n");
		fflush(stdout);						/* flush stdout */
#if defined (_WIN32)
		system ("command.com");
#else
		tcsetattr (0, TCSAFLUSH, &cmdtty);
		system ("bash");
		tcsetattr (0, TCSAFLUSH, &runtty);
#endif
		return (0);
	}	
	if (strcmp(token, "SIGNON") == 0 ||
		strcmp(token, "SIGNO") == 0 ||
		strcmp(token, "SIGN") == 0 ||
		strcmp(token, "SIG") == 0 ||
		strcmp(token, "SI") == 0) {
		if (status < INITIAL_WAIT) {
			ttystr("\r\nRJE125A You need a connection first.  Use OPEN.\r\n");
			return (0);
		}
		if (nexttoken() == 1) {
			ttystr("\r\nRJE126A The user id is missing, try again\r\n");
			return (0);
		}
		gettoken(0);
		if (strcmp(token, "*") == 0) {
			ttystr("\r\nRJE300I Signon bypassed.");
			status = IDLE;
			pollflag = 2;
			pollctr = 0;
			return (0);
		}	
		ttystr("\r\n");


		// Flush out anything that might be in the pipe

		while (1) {
			rc = read_poll(0, 5000);
			if (rc == 0) break;
			rc = read_data(3, 0, 0);
		}

		// Send the initial ENQ and see if the host gives us an ACK0
		
		retry = 0;
		while (retry < 3) {
			line_out[0] = line_out[1] = line_out[2] = SYN;
			line_out[3] = ENQ;
			line_out_size = 4;
			write_buffer();
			rc = read_data(3, 0, 0);
			if (rc != 0)
				break;
			retry++;	
		}	
		if (rc == -1) {
			ttystr("\r\nRJE127S The line has disconnected.\r\n");
			return (0);
		}
		if (rc == 0) {
			ttystr("\r\nRJE128S The host did not respond to our initial greeting.\r\n");
			return (0);
		}
		if (line_in[0] == NAK || (line_in[0] == DLE && line_in[1] == NAK)) {
			ttystr("\r\nRJE129S The host says (with a NAK) it's not ready.\r\n");
			return (0);
		}
		if (line_in[0] != DLE || line_in[1] != ACK0) {
			return (0);
		}
		
		// Ok, great, it's talking to us.
		// Build the SIGNON card 
		
		line_out[0] = SYN;
		line_out[1] = SYN;
		line_out[2] = STX;
		line_out[3] = 0;
		line_out_size = 3;
		switch (opt_os) {
			case 1:		// VM/370
				strcpy(signon, "SIGNON ");
				break;
			case 4:		// DOS/VS
				strcpy(signon, "* .. SIGNON ");
				break;
			case 6:		// OS/360
				strcpy(signon, ".. RJSTART ");
				break;
			default:	// All others
				strcpy(signon, "SIGNON ");
				break;
		}
		strcat(signon, token);
		if (nexttoken() != 1) {
			gettoken(0);
			strcpy(passw, token);
		} else {
			strcpy(passw, "");
		}
		switch (opt_os) {
			case 1:		// VM/370
				strcat(signon, " ");
				strcat(signon, "3780 B512 P120 TRSY PCHY");
				if (strlen(passw) > 0) {
					strcat(signon, " PWD=");
					strcat(signon, passw);
				}
				break;
			case 4:		// DOS/VS
				strcat(signon, " ");
				strcat(signon, passw);
				break;
			case 6:		// OS/360
				strcat(signon, ",BRDCST=YES");
				break;
			default:	// All others
				strcat(signon, " ");
				strcat(signon, passw);
				break;
		}
		translate_to_ebcdic(signon);
		strcat(line_out, signon);
		line_out_size += strlen(signon);
		line_out[line_out_size] = ETX;
		line_out_size++;
		write_buffer();
		rc = read_data(10, 0, 0);
		if (rc == 0) {
			ttystr("\r\nRJE131S The host did not respond to the signon record.\r\n");
			return (0);
		}
		if (line_in[0] == NAK || (line_in[0] == DLE && line_in[1] == NAK)) {
			ttystr("\r\nRJE132S The host says (with a NAK) it didn't like the signon.\r\n");
			return (0);
		}
		if (line_in[0] != DLE || line_in[1] != ACK1) {
			ttystr("\r\nRJE133S The host said something odd.  Use TRACE.\r\n");
			return (0);
		}
		
		// Cool. The host said it got our signon.
		// Let's send it an EOT so it'll know we're done
		
		line_out[0] = line_out[1] = SYN;
		line_out[2] = EOT;
		line_out_size = 3;
		write_buffer();
		// Note:  EOT does not expect a reply
		status = IDLE;
		pollflag = 2;
		pollctr = 0;
		return (0);	
		
	}
	if (strcmp(token, "STATUS") == 0 ||
		strcmp(token, "STATU") == 0 ||
		strcmp(token, "STAT") == 0 ||
		strcmp(token, "STA") == 0 ||
		strcmp(token, "ST") == 0) {
		if (status == NOLINK) {
			ttystr("\r\nRJE134I You have not connected to the host.  Use OPEN.");
		}	
		if (status == INITIAL_WAIT) {
			ttystr("\r\nRJE135I Connected but not signed on.  Use SIGNON.");
		}	
		if (status == IDLE) {
			ttystr("\r\nRJE136I Link is signed on but currently idle.");
		}
		if (status == SENDING) {
			ttystr("\r\nRJE137I Sending a file to the host.");
		}
		if (status == RECEIVING) {
			ttystr("\r\nRJE138I Receiving data from the host.");
		}
		ttystr("\r\nRJE139I Received print data will be ");
		if (strlen(print) > 0) {
			ttystr("stored in ");
			ttystr(print);
		} else {
			if (opt_copy == 1) {
				ttystr("displayed onscreen only");
			} else {
				ttystr("discarded.");
			}
		}
		ttystr("\r\nRJE140I Received punch data will be ");
		if (strlen(punch) > 0) {
			ttystr("stored in ");
			ttystr(punch);
			ttystr(" in ");
			if (punch_fmt) {
				ttystr("EBCDIC, recl=");
				sprintf(reclen,"%d",punch_recl);
				ttystr(reclen);
			} else {
				ttystr("ASCII");
			}
		} else {
			ttystr("discarded");
		}
		return (0);
	}	
	if (strcmp(token, "TRACE") == 0 ||
		strcmp(token, "TRAC") == 0 ||
		strcmp(token, "TRA") == 0 ||
		strcmp(token, "TR") == 0) {
		if (nexttoken() == 1) {
			strcpy(token, "");
		} else {
			gettoken(0);
		}
		if (strlen(token) > 0) {
			ttystr("\r\nRJE035I Data trace will be recorded in ");
			ttystr(token);
		} else {
			ttystr("\r\nRJE036I Data trace will no longer be recorded");
			if (strlen(tracefile) > 0)
				fclose(tracefd);
		}
		strcpy(tracefile, token);
		if (strlen(tracefile) > 0)
			tracefd = fopen(tracefile, "w");
		return (0);
	}
	if (strcmp(token, "PRINT") == 0 ||
		strcmp(token, "PRIN") == 0 ||
		strcmp(token, "PRI") == 0 ||
		strcmp(token, "PR") == 0) {
		if (nexttoken() == 1) {
			strcpy(token, "");
		} else {
			gettoken(0);
		}
		strcpy(print, token);
		ttystr("\r\nRJE141I Received print data will be ");
		if (strlen(print) > 0) {
			ttystr("stored in ");
			ttystr(print);

			if (print_open == 1)

				fclose(printfd);

			print_open = 0;
		} else {
			if (opt_copy == 1) {
				ttystr("displayed onscreen only");
			} else {
				ttystr("discarded.");
			}
		}
		return (0);
	}	
	if (strcmp(token, "PUNCH") == 0 ||
		strcmp(token, "PUNC") == 0 ||
		strcmp(token, "PUN") == 0 ||
		strcmp(token, "PU") == 0) {
		if (nexttoken() == 1) {
			strcpy(token, "");
		} else {
			gettoken(0);
		}
		strcpy(punch, token);
		if (nexttoken() != 1) {
			gettoken(1);
			savep = punch_fmt;
			punch_fmt = -1;
			if (strcmp(token, "ASCII") == 0) {
				punch_fmt = 0;
			}
			if (strcmp(token, "EBCDIC") == 0) {
				punch_fmt = 1;
			}
			if (punch_fmt == -1) {
				ttystr("\r\nRJE142A Invalid option following punch filename");
				punch_fmt = savep;
				return (0);
			}
			if (nexttoken() != 1) {
				gettoken(0);
				savep = punch_recl;
				punch_recl=atoi(token);
				if (punch_recl < 80 || punch_recl > 512) {
					ttystr("\r\nRJE143A Invalid punch record length");
					punch_recl = savep;
					return (0);
				}
			}
		}
		ttystr("\r\nRJE144I Received punch data will be ");
		if (strlen(punch) > 0) {
			ttystr("stored in ");
			ttystr(punch);
			if (punch_open == 1)
				fclose(punchfd);
			punch_open = 0;
			ttystr(" in ");
			if (punch_fmt) {
				ttystr("EBCDIC, recl=");
				sprintf(reclen,"%d",punch_recl);
				ttystr(reclen);
			} else {
				ttystr("ASCII");
			}
		} else {
			ttystr("discarded");
		}
		return (0);
	}	
	if (strcmp(token, "SET") == 0) {
		if (nexttoken() == 1) {
			ttystr("\r\nRJE145I Host OS is: ");
			switch (opt_os) {
			case 0: 
				ttystr("Generic RJE");
				break;
			case 1: 
				ttystr("VM/370 R6 RSCS V1.");
				break;
			case 2: 
				ttystr("MVS JES2");
				break;
			case 3: 
				ttystr("MVS JES3");
				break;
			case 4: 
				ttystr("DOS[/VS] POWER[/VS].");
				break;
			case 5: 
				ttystr("OS/VS1 RES.");
				break;
			case 6: 
				ttystr("OS/360 RJE");
				break;
			}
			ttystr("\r\nRJE146I Route to this user id: ");
			if (strlen(opt_user) > 0) {
				ttystr(opt_user);
			} else {
				ttystr("(none)");
			}
			ttystr("\r\nRJE147I Display printer output: ");
			if (opt_copy == 1) {
				ttystr("ON");
			} else {
				ttystr("OFF");
			}
			ttystr("\r\nRJE148I Poll when idle: ");
			if (opt_poll == 1) {
				ttystr("ON");
			} else {
				ttystr("OFF");
			}
			ttystr("\r\nRJE301I Transparent send: ");
			if (opt_trn == 1) {
				ttystr("ON");
			} else {
				ttystr("OFF");
			}
//			ttystr("\r\nRJE148I Pause printer display: ");
//			switch (opt_pause) {
//			case -1:
//				ttystr("on form feed.");
//				break;
//			case 0:
//				ttystr("never.");
//				break;
//			default:
//				sprintf(wstr, "every %d lines.", opt_pause);
//				ttystr(wstr);
//				break;
//			}
		} else {
			gettoken(1);
			if (strcmp(token, "NOCOPY") == 0) {
				opt_copy = 0;
				return (0);
			} 
			if (strcmp(token, "COPY") == 0) {
				opt_copy = 1;
				return (0);
			} 
			if (strcmp(token, "NOPOLL") == 0) {
				opt_poll = 0;
				return (0);
			} 
			if (strcmp(token, "POLL") == 0) {
				opt_poll = 1;
				return (0);
			} 
			if (strcmp(token, "NOTRN") == 0) {
				opt_trn = 0;
				return (0);
			} 
			if (strcmp(token, "TRN") == 0) {
				opt_trn = 1;
				return (0);
			}
			if (strcmp(token, "NOOS") == 0) {
				opt_os = 0;
				return (0);
			} 
			if (strcmp(token, "OS") == 0) {
				opt_trn = 1;
				opt_os = 6;
				return (0);
			} 
			if (strcmp(token, "VM") == 0) {
				opt_os = 1;
				return (0);
			} 
			if (strcmp(token, "JES2") == 0) {
				opt_os = 2;
				return (0);
			} 
			if (strcmp(token, "JES3") == 0) {
				opt_os = 3;
				return (0);
			} 
			if (strcmp(token, "DOS") == 0) {
				opt_os = 4;
				return (0);
			} 
			if (strcmp(token, "RES") == 0) {
				opt_os = 5;
				return (0);
			} 
			if (strcmp(token, "USER") == 0) {
				if (nexttoken() == 1) {
					ttystr("\r\nRJE160A Missing userid");
				} else {
					gettoken(1);
					strcpy(opt_user, token);
					opt_os = 1;
				}
				return (0);
			}
			if (strcmp(token, "PAUSE") == 0) {
				if (nexttoken() == 1) {
					ttystr("\r\nRJE161A Missing number or FF after SET PAUSE");
				} else {
					gettoken(1);
					if (strcmp(token, "FF") == 0 ||
						strcmp(token, "ff") ==0) {
							opt_pause = -1;
							return (0);
					}
					if (strcmp(token, "NO") == 0 ||
						strcmp(token, "no") ==0) {
							opt_pause = 0;
							return (0);
					}
					opt_pause = atoi(token);
				}
				return (0);
			}
			ttystr("\r\nRJE162A Invalid SET option");
		}
		return (0);
	}	
	if (strcmp(token, "CMD") == 0 ||
		strcmp(token, "C") == 0) {
		if (nexttoken() == 1) {
			ttystr("\r\nRJE163A Enter a command after CMD to send.\r\n");
			return (0);
		}
		ttystr("\r\n");
		for (i = 0; i < 128; i++) {cmd[i] = 0;}
		i = 0;
		while (command[comctr] != '\n' && command[comctr] != 0) {
			cmd[i] = command[comctr];
			i++;
			comctr++;
		}
		switch (opt_os) {
		case 4:
			strcpy(cmdline, "* .. ");
			strcat(cmdline, cmd);
			break;
		case 6:
			strcpy(cmdline, ".. ");
			strcat(cmdline, cmd);
			break;
		default:
			strcpy(cmdline, cmd);
			break;
		}
		rc = sendfile(cmdline);
		if (rc != -1) {
			status = IDLE;
			pollflag = 2;
			pollctr = 0;
		}
		return (0);
	}	

	if (strcmp(token, "SEND") == 0 ||
		strcmp(token, "SEN") == 0 ||
		strcmp(token, "S") == 0) {
		if (status < IDLE) {
			ttystr("\r\nRJE064A You are not signed on, use SIGNON.\r\n");
			return (0);
		}
		if (nexttoken() == 0) {
			gettoken(0);
			strcpy(reader, token);
			if (nexttoken() != 1) {
				gettoken(1);
				savep = reader_fmt;
				reader_fmt = -1;
				if (strcmp(token, "ASCII") == 0) {
					reader_fmt = 0;
				}
				if (strcmp(token, "EBCDIC") == 0) {
					reader_fmt = 1;
				}
				if (reader_fmt == -1) {
					ttystr("\r\nRJE164A Invalid option following filename");
					reader_fmt = savep;
					return (0);
				}
				if (nexttoken() != 1) {
					gettoken(0);
					savep = reader_recl;
					reader_recl=atoi(token);
					if (reader_recl < 80 || reader_recl > 512) {
						ttystr("\r\nRJE165A Invalid send record length");
						reader_recl = savep;
						return (0);
					}
				}
			}
		}
		status = SENDING;
		sendfile("");
		status = IDLE;
		pollflag = 2;
		pollctr = 0;
		return (0);
	}	
	if (strcmp(token, "CLOSE") == 0 ||
		strcmp(token, "CL") == 0) {
		if (status > NOLINK) {
			close (sockfd);
			ttystr("\r\nRJE166I Connection closed.\n\r");
			status = NOLINK;
		} else {
			ttystr("\r\nRJE167W You are not presently connected.\n\r");
		}	
		return (0);
	}	
	if (strcmp(token, "QUIT") == 0 ||
		strcmp(token, "QUI") == 0 ||
		strcmp(token, "Q") == 0 ||
		strcmp(token, "QU") == 0 ||
		strcmp(token, "BYE") == 0 ||
		strcmp(token, "BY") == 0 ||
		strcmp(token, "EXIT") == 0 ||
		strcmp(token, "EX") == 0 ||
		strcmp(token, "END") == 0) {
		ttystr("\r\nRJE168I Shutting down RJE80...\n\r");
		if (status > NOLINK) {
			ttystr("RJE169I Connection closed\r\n");
			close(sockfd);
		}	
		status = SHUTDOWN;
		return (0);
	}	
	if (strcmp(token, "HELP") == 0 || strcmp(token, "?") == 0) {
		if (nexttoken() == 1) {
			ttystr("\r\nHelp topics for RJE80:\n\r\n\r");
			ttystr("   Intro    General help for RJE80 program.\r\n");
			ttystr("   Open     Open TCP/IP connection to the host.\r\n");
			ttystr("   SIgnon   Signon to the host.\r\n");
			ttystr("   PRint    Set the print destination filename.\r\n");
			ttystr("   PUnch    Set the punch destination filename.\r\n");
			ttystr("   Cmd      Send a command to the remote host OS.\r\n");
			ttystr("   Send     Send a file to the host.\r\n");
			ttystr("   STatus   Show the status of the connection.\r\n");
			ttystr("   SEt      Control various options and parameters.\r\n");
			ttystr("   TRace    Record communcations data in a log file.\r\n");
			ttystr("   CLose    Close the open TCP/IP connection.\r\n");
			ttystr("   !        Spawn a shell underneath RJE80.\r\n");
			ttystr("   Quit     Exit the program.\r\n");
			ttystr("\r\n");
			ttystr("   For more information, use HELP <topic>\r\n");
			ttystr("\r\n");
			return (0);
		}
		gettoken(1);
		if (strcmp(token, "OPEN") == 0 ||
			strcmp(token, "O") == 0) {
			ttystr("\r\n\r\n");
			ttystr("   Syntax: OPEN <hostname> <port>\r\n");
			ttystr("\r\n");
			ttystr("   Before you can start communicating with the host, you\r\n");
			ttystr("   must establish the TCP/IP connection to the port\r\n");
			ttystr("   configured as the LPORT in the Hercules configuration.\r\n");
			ttystr("   <hostname> is the name (or IP address) of the machine \r\n");
			ttystr("   acting as the host, and <port> is the LPORT of the BSC\r\n");
			ttystr("   line you want to connect to.  The host OS does not have\r\n");
			ttystr("   to be running when you make this connection, only when\r\n");
			ttystr("   when you SIGNON, which is the next step of the process.\r\n");
			ttystr("   \r\n");
			ttystr("   Example: OPEN mvs.dinosrus.com 3780\r\n");
			ttystr("   \r\n");
			ttystr("   Note:  You can specify a host and port to open on the\r\n");
			ttystr("   rje80 command line, for example: rje80 mvs.dino.com 3780\r\n");
			ttystr("   \r\n");
			ttystr("   Important:  It may seem counterintuitive, but this command\r\n");
			ttystr("   doesn't verify that the other end is actually ready for the \r\n");
			ttystr("   connection.  This is so that you can start rje80 first, then\r\n");
			ttystr("   the host.  It's the SIGNON command that verifies that the \r\n");
			ttystr("   connection can be used.\r\n");
			return(0);
		}
		if (strcmp(token, "CMD") == 0 ||
			strcmp(token, "C") == 0) {
			ttystr("\r\n\r\n");
			ttystr("   Syntax: CMD <command string>\r\n");
			ttystr("\r\n");
			ttystr("   Most remote operating systems allow the RJE operator to send\r\n");
			ttystr("   commands to control their station from the host side.  Refer to\r\n");
			ttystr("   the manual for each OS for details on the possible commands. To\r\n");
			ttystr("   send a command, put it after the CMD, spaces and all.  The string\r\n");
			ttystr("   will be sent to the remote host, as you typed it.  For certain OSes\r\n");
			ttystr("   RJE80 will prepend a command identifier, if the os is SET.  For\r\n");
			ttystr("   example, DOS/VS POWER/VS requires the 4-character string '* ..' to\r\n");
			ttystr("   precede each command.  If you SET DOS, RJE80 will insert this in \r\n");
			ttystr("   front of the commands you type, saving you the trouble.\r\n");
			ttystr("   \r\n");
			ttystr("   Example: CMD PDISPLAY LST sends a PDISPLAY LST command to DOS/VS.\r\n");
			ttystr("            C SIGNOFF signs you off the DOS/VS RJE line.\r\n");
			return(0);
		}
		if (strcmp(token, "SIGNON") == 0 ||
			strcmp(token, "SI") == 0) {
			ttystr("\r\n\r\n");
			ttystr("   Syntax: SIGNON <userid> [password]\r\n");
			ttystr("\r\n");
			ttystr("   This step establishes communication with the operating\r\n");
			ttystr("   system running in the remote host.  Once signon completes\r\n");
			ttystr("   normally, you are ready to send and receive files.  The \r\n");
			ttystr("   <userid> is defined on the host system, on a VM/370 RSCS \r\n");
			ttystr("   machine it is the linkname you are connecting to.  The\r\n");
			ttystr("   [password] is usually necessary but could be optional,\r\n");
			ttystr("   depending on the host's particular configuration.\r\n");
			ttystr("   \r\n");
			ttystr("   Example:  SIGNON RJELINK1 GUESTPWD\r\n");
			ttystr("   \r\n");
			ttystr("   Note: There are many possible reasons for failure.  Among\r\n");
			ttystr("   them are:  * Wrong hostname or port given on OPEN command,\r\n");
			ttystr("              * Hercules configuration doesn't match OPEN\r\n");
			ttystr("              * Host OS isn't running on hercules machine\r\n");
			ttystr("              * BSC line isn't enabled from the host OS\r\n");
			ttystr("              * A firewall is blocking the IP traffic\r\n");
			ttystr("   \r\n");
			ttystr("   If your signon is rejected, it may be that the host OS doesn't\r\n");
			ttystr("   understand its format.  Be sure to SET your host OS so RJE80 can\r\n");
			ttystr("   properly format the SIGNON command.\r\n");
			return(0);
		}
		if (strcmp(token, "CLOSE") == 0 ||
			strcmp(token, "CL") == 0) {
			ttystr("\r\n\r\n");
			ttystr("   Syntax: CLOSE\r\n");
			ttystr("   \r\n");
			ttystr("   This closes the current connection.  If you do this, the\r\n");
			ttystr("   TCP/IP socket is disconnected, and the host OS will likely\r\n");
			ttystr("   see this as a drop in the connect signal from the modem, and\r\n");
			ttystr("   may or may not disable the line as a result.\r\n");
			return(0);
		}
		if (strcmp(token, "TRACE") == 0 ||
			strcmp(token, "TR") == 0) {
			ttystr("\r\n\r\n");
			ttystr("   Syntax: TRACE <filename>\r\n");
			ttystr("   \r\n");
			ttystr("   Use this command with a filename to record all data sent and\r\n");
			ttystr("   received over the bisync line in that file.  An ascii translation\r\n");
			ttystr("   as well as the EBCDIC hex is saved in a human readable format.  Use\r\n");
			ttystr("   TRACE without a filename to turn off tracing.\r\n");
			return(0);
		}
		if (strcmp(token, "INTRO") == 0 ||
			strcmp(token, "I") == 0) {
			ttystr("\r\n\r\n");
			ttystr("   In the early years of the System/360, when batch mainframes \r\n");
			ttystr("   were kings and interactive terminals almost unknown, IBM designed\r\n");
			ttystr("   a device which allowed users to remotely submit jobs to the 360\r\n");
			ttystr("   mainframe over a communications link.  That device was the 2780\r\n");
			ttystr("   RJE terminal, consisting of a card reader, printer, and optionally\r\n");
			ttystr("   a card punch.  Later devices such as the 3780 were improvements on\r\n");
			ttystr("   the original 2780, but the principle remained the same.  Over the years,\r\n");
			ttystr("   the 2780/3780 RJE protocol became a de facto standard for mainframe job\r\n");
			ttystr("   entry and file transfer.  Every operating system from IBM that runs on\r\n");
			ttystr("   the System/370 class mainframes supports this form of RJE.\r\n");
			ttystr("   \r\n");
			ttystr("   The Hercules emulator, which runs on PC-class hardware under Windows and\r\n");
			ttystr("   Linux, emulates a System/370 (or later) class mainframe.  Part of this\r\n");
			ttystr("   emulation is a 2703 device, which is the generic term for the hardware\r\n");
			ttystr("   device that implements the bisync protocol, on which RJE is based.  This\r\n");
			ttystr("   emulation doesn't work on real bisync hardware such as modems, instead it\r\n");
			ttystr("   is simulated by using a TCP/IP connection over the internet.\r\n");  
			ttystr("   \r\n");
			ttystr("   This program (RJE80) emulates a 3780, connected over a simulated bisync\r\n");
			ttystr("   line to a mainframe operating system (such as MVS) running on a Hercules\r\n");
			ttystr("   emulator.\r\n");
			ttystr("   \r\n");
			ttystr("   The operation of RJE80 is simple, because the 3780 was a very simple device.\r\n");
			ttystr("   You need to know four things:  the name (or internet address) of the host\r\n");
			ttystr("   want to connect to, the port number on that host that's listening for the\r\n");
			ttystr("   line to expect to connect to, and a user ID and password that's assigned \r\n");
			ttystr("   by the mainframe operator to that line.  Having all that you first use the\r\n");
			ttystr("   OPEN command to connect to the host and port, then the SIGNON command to \r\n");
			ttystr("   connect to the mainframe OS.  Once that's done, use the SEND command to\r\n");
			ttystr("   send a job.  There is no RECEIVE command, print and punch output are \r\n");
			ttystr("   received automatically, but there are PRINT and PUNCH commands to specify\r\n");
			ttystr("   what is to be done with the data received.  For other commands, see more \r\n");
			ttystr("   of the online help.  Enjoy RJE80!\r\n");
			return(0);
		}
		if (strcmp(token, "PRINT") == 0 ||
			strcmp(token, "PR") == 0) {
			ttystr("\r\n\r\n");
			ttystr("   Syntax: PRINT [filename]\r\n");
			ttystr("   \r\n");
			ttystr("   When data is received from the host directed to the local\r\n");
			ttystr("   printer, this option determines whether it will be saved in a\r\n");
			ttystr("   file or not.  If you want printer output saved, give a file\r\n");
			ttystr("   name.  To just display it on the console, use PRINT without a \r\n");
			ttystr("   filename (which is the default when RJE80 starts).\r\n");
			ttystr("   \r\n");
			ttystr("   Example: PRINT myprintout.txt\r\n");
			ttystr("   \r\n");
			ttystr("   Note: Printer output is always translated to ascii.  It's also\r\n");
			ttystr("   always displayed on your screen as it's received, unless you\r\n");
			ttystr("   use a SET NOCOPY command to stop it.\r\n");
			return(0);
		}
		if (strcmp(token, "PUNCH") == 0 ||
			strcmp(token, "PU") == 0) {
			ttystr("\r\n\r\n");
			ttystr("   Syntax: PUNCH [filename] [ascii | ebcdic] [recl]\r\n");
			ttystr("   \r\n");
			ttystr("   When data is received from the host directed to the local\r\n");
			ttystr("   punch device, it will be stored in the filename you give\r\n");
			ttystr("   here.  PUNCH without a file name means punch data will go\r\n");
			ttystr("   into the bit bucket.  You have a choice as to whether you \r\n");
			ttystr("   want the data translated to ascii or not, or saved in its\r\n");
			ttystr("   original EBCDIC form, bit for bit.  If you choose EBCDIC, you \r\n");
			ttystr("   have the option of giving a record length, which if not given\r\n");
			ttystr("   defaults to 80 (of course).\r\n");
			ttystr("   \r\n");
			ttystr("   Example:  PUNCH objectprog.cd ebcdic 80\r\n");
			ttystr("   \r\n");
			ttystr("   Note: the default punch output file is 'punch.txt' and is\r\n");
			ttystr("   translated to ascii.\r\n");
			return(0);
		}
		if (strcmp(token, "SEND") == 0 ||
			strcmp(token, "S") == 0) {
			ttystr("\r\n\r\n");
			ttystr("   Syntax: SEND <filename> [ascii | ebcdic] [recl]\r\n");
			ttystr("   \r\n");
			ttystr("   This is the command to send a file to the host.  In most cases\r\n");
			ttystr("   this means you're submitting JCL to an input queue on the host.\r\n");
			ttystr("   Unless you specify otherwise, the file is considered to be\r\n");
			ttystr("   an Ascii text file, which will be translated to EBCDIC before\r\n");
			ttystr("   being sent.  You can override this by specifying ebcdic as the\r\n");
			ttystr("   second option.  Normally, records are 80 bytes long.  You can \r\n");
			ttystr("   change this too, with the recl option.  If you want to specify \r\n");
			ttystr("   a record length, you'll need to include the second option before\r\n");
			ttystr("   it (ascii or ebcdic).  If you specify * for the filename, input\r\n");
			ttystr("   will come from the keyboard.  Press CTRL-D and ENTER to end input.\r\n");
			ttystr("   \r\n");
			ttystr("   Example: SEND myfile.txt \r\n");
			ttystr("            SEND objectdeck.cd ebcdic\r\n");
			ttystr("            SEND *\r\n");
			ttystr("   \r\n");
			ttystr("   Note:  You can SEND a file at any time.  If printer or punch data\r\n");
			ttystr("   is being received when you SEND, it will be suspended until the\r\n");
			ttystr("   transmission is complete.  If you are sending a file to a VM/370\r\n");
			ttystr("   user, use the SET USER command to specify the userid, or be sure \r\n");
			ttystr("   that the cards are preceded by a valid ID card.\r\n");
			return(0);
		}
		if (strcmp(token, "STATUS") == 0 ||
			strcmp(token, "ST") == 0) {
			ttystr("\r\n\r\n");
			ttystr("   Syntax: STATUS\r\n");
			ttystr("   \r\n");
			ttystr("   This command prints out several lines giving you the status of\r\n");
			ttystr("   your connection.\r\n");
			ttystr("   \r\n");
			return(0);
		}
		if (strcmp(token, "SET") == 0 ||
			strcmp(token, "SE") == 0) {
			ttystr("\r\n\r\n");
			ttystr("   Syntax: SET <option>\r\n");
			ttystr("   \r\n");
			ttystr("   This command sets (or resets) various options:\r\n");
			ttystr("   \r\n");
			ttystr("   SET [NO]COPY      Whether or not printer data is displayed\r\n");
			ttystr("   SET [NO]POLL      Whether or not to poll the line when idle\r\n");
			ttystr("   SET USER <userid> A VM userid to send the file to\r\n");
			//ttystr("   SET PAUSE nnn     Pause display after every nnn lines\r\n");
			//ttystr("   SET PAUSE NO      Do not pause display (default) \r\n");
			//ttystr("   SET PAUSE FF      Pause display on every form feed\r\n");
			//ttystr("   SET WIDTH nnn     Set the width of the print line\r\n");
			ttystr("   SET OS            The remote host is OS/360 (or generic)\r\n");
			ttystr("   SET VM            The remote host is VM/370 RSCS\r\n");
			ttystr("   SET DOS           The remote host is DOS[/VS] \r\n");
			ttystr("   SET JES2          The remote host is MVS/JES2 \r\n");
			ttystr("   SET JES3          The remote host is MVS/JES3 \r\n");
			ttystr("   SET RES           The remote host is OS/VS1 \r\n");
			ttystr("   SET               Display current options\r\n");
			ttystr("   \r\n");
			ttystr("   Note: SET without parameters displays the current settings.\r\n");
			return(0);
		}
		if (strcmp(token, "QUIT") == 0 ||
			strcmp(token, "Q") == 0) {
			ttystr("\r\n\r\n");
			ttystr("   Syntax: QUIT\r\n");
			ttystr("   \r\n");
			ttystr("   This exits the program, closing the connection if \r\n");
			ttystr("   necessary.\r\n");
			return(0);
		}
		ttystr("\r\nRJE170A That (");
		ttystr(token);
		ttystr(")");
		ttystr(" is not a valid help topic.\n\r");
	}	
	ttystr("\r\nRJE171A That (");
	ttystr(token);
	ttystr(")");
	ttystr(" is not a valid command.  Try HELP.\n\r");
	return(0);		
}

int nexttoken()
{
	while (comctr < comlen) {
		if (command[comctr] == ' ' || command[comctr] == ',') {
			comctr++;
			continue;
		}
		return(0);	
	}
	return(1);
}

int gettoken(int upper)
{
	int i, j = 0;
	for (i = 0; i < 64; i++) token[i] = 0;
	while (command[comctr] != ' ' && command[comctr] != ',' && j < 64) {
		if (!upper) {
			token[j] = command[comctr];
		} else {
			token[j] = toupper(command[comctr]);
		}
		j++;
		comctr++;
	}
	return (0);
}

// Function to connect to the host

int connecthost() 
{
	struct hostent *he;
	struct sockaddr_in sin;
	struct in_addr intmp;
	char thing[80];
	int rc, wsaerrno;
	long int non_block = 1;

	he = gethostbyname(inethost);
	strcpy(hname, inethost);
	if (he == NULL) {
		ttystr("\r\nRJE201A Can't locate hostname: ");
		ttystr(inethost);
		ttystr("\r\n");
		return(-1);
	}
	memcpy(&host_ip, he->h_addr_list[0], 4);
	sin.sin_family = AF_INET;
	sin.sin_addr.s_addr = host_ip;
	sin.sin_port = htons(inetport);
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
#if defined (_WIN32)
	ioctlsocket (sockfd, FIONBIO, &non_block);
	rc = connect(sockfd, (struct sockaddr *)&sin, sizeof(sin));
	if ((rc == SOCKET_ERROR) &&
		(wsaerrno = WSAGetLastError() != WSAEWOULDBLOCK)) {
		ttystr("\r\nRJE202S Failed to connect\r\n");
		ttystr("\r\nRJE202I The winsock error number is ");
		sprintf(thing, "%d", wsaerrno);
		ttystr(thing);
		close(sockfd);
		status = NOLINK;
		return (-1);
	}
#else
	rc = fcntl(sockfd, F_GETFL);
	rc |= O_NONBLOCK;
	fcntl(sockfd, F_SETFL, rc);
	rc = connect(sockfd, (struct sockaddr *)&sin, sizeof(sin));
	if (rc < 0 && errno!= EINPROGRESS) {
		ttystr("\r\nRJE202S Failed to connect\r\n");
		perror("RJE202I The reason was: ");
		close(sockfd);
		status = NOLINK;
		return (-1);
	}
#endif
	ttystr("\r\nRJE203I Link established to ");	
	ttystr(hname);
	ttystr(" (");
	intmp.s_addr = host_ip;
	strcpy(thing, (char *)inet_ntoa(intmp));
	ttystr(thing);
	ttystr(") ");
	ttystr(" port ");
	sprintf(thing, "%d", inetport);
	ttystr(thing);
	ttystr(".");
	status = INITIAL_WAIT;
	return (1);
}


// This is the mighty SEND function
// It's one parameter is a possible command to be sent in place of the file

int sendfile(char *command)
{
	int rc, i, count = 0, show = 0, retry = 0;
	char wstr[32];
	unsigned char cardbuf[512];
	unsigned char savebuf[512];
	unsigned char output_data[512];

	strcpy(savebuf, "");
	if (strlen(command) == 0) {
		if (strcmp(reader, "*") != 0) {
			readerfd = fopen(reader, "r");
			if (readerfd == NULL) {
				ttystr("\r\nRJE172S Can't open that file, it doesn't exist?\r\n");
				return (-1);
			}
			ttystr("\r\nRJE173I Sending file '");
			ttystr(reader);
			ttystr("' to the host.\r\n");
		} else {
			ttystr("\r\nRJE174A Enter lines to send, CTRL-D for EOF\r\n");
		}
	} else {
		strcpy(savebuf, command);
	}

	// Send the initial ENQ and see if the host gives us an ACK0
	
	while (retry < 10) {
		line_out[0] = line_out[1] = SYN;
		line_out[2] = ENQ;
		line_out_size = 3;
		write_buffer();
		rc = read_data(10, 0, 0);
		if (rc != 0)
			break;
		retry++;	
	}	
	if (rc == -1) {
		ttystr("\r\nRJE175S The line has disconnected.\r\n");
		ttystr("\r\nRJE176W The connection is now closed.\r\n");
		status = NOLINK;
		close(sockfd);
		return (-1);
	}
	if (rc == 0) {
		ttystr("\r\nRJE177S The host did not respond to our initial greeting.\r\n");
		return (-1);
	}
	if (line_in[0] == NAK || (line_in[0] == DLE && line_in[1] == NAK)) {
		ttystr("\r\nRJE178S The host says (with a NAK) it's not ready.\r\n");
		return (-1);
	}
	if (line_in[0] != DLE || line_in[1] != ACK0) {
		return (-1);
	}

	// HOST appears to want our file

	while (1) {
		if (show > 9) {
			show = 0;
			ttystr("RJE180I ");
			sprintf(wstr, "%d Records sent.\r", count);
			ttystr(wstr);
		}
		if (strlen(savebuf) == 0) {
			for (i = 0; i < 512; i++) {cardbuf[i] = 0;}
			if (strcmp(reader, "*") != 0) {
				if (reader_fmt == 0) {
					fgets(cardbuf, 512, readerfd);
				} else {
					fread(cardbuf, reader_recl, 1, readerfd);
				}
				if (feof(readerfd)) {
					ttystr("\r\nRJE181I ");
					sprintf(wstr, "%d Total records sent.", count);
					ttystr(wstr);
					fclose(readerfd);
					line_out[0] = line_out[1] = SYN;
					line_out[2] = EOT;
					line_out_size = 3;
					write_buffer();
					return (0);
				}
			} else {
				ttygets(cardbuf);
				if (cardbuf[0] == '\004') {
					line_out[0] = line_out[1] = SYN;
					line_out[2] = EOT;
					line_out_size = 3;
					write_buffer();
					return (0);
				}
			}
		} else {
			strcpy(cardbuf, savebuf);
			strcpy(savebuf, "");
		}

		// Handle special VM ID card

		if (opt_os == 1 && reader_fmt == 0 && strlen(command) == 0) {
			if (count == 0) {
				for (i = 0; i < 9; i++) {wstr[i] = cardbuf[i];}
				wstr[10] = 0;
				if (strcmp(wstr, "ID       ") != 0) {
					if (strlen(opt_user) > 0) {
						strcpy(wstr, "ID       ");
						strcat(wstr, opt_user);
						strcpy(savebuf, cardbuf);
						strcpy(cardbuf, wstr);
					}
				}
			}
		}

		// We have a line of data ... massage it, then send it

		if (reader_fmt == 0 || strlen(command) > 0) {
			for (i = 0; i < 512; i++) {output_data[i] = 0x20;}
			for (i = 0; i < strlen(cardbuf); i++) {
				if (cardbuf[i] != '\n' &&
				    cardbuf[i] != '\r')
					output_data[i] = cardbuf[i];
			}
			output_data[reader_recl + 1] = 0;
			translate_to_ebcdic(output_data);
		} else {
			for (i = 0; i < reader_recl; i++) {
				output_data[i] = cardbuf[i];
			}
		}
		line_out[0] = STX;	/* Build the record to transmit */
		line_out_size = 1;
		for (i = 0; i < reader_recl; i++) {
			line_out[line_out_size] = output_data[i];
			line_out_size++;
		}
		line_out[line_out_size] = ETX;
		line_out_size++;
		retry = 0;
		while (1) {
			write_buffer();			/* Send the data */
			rc = read_data(10, 0, 0);	/* Get the reply */
			if (rc == -1) {
				ttystr("\r\nRJE182S The line has disconnected during the send.\r\n");
				ttystr("\r\nRJE183W The connection is now closed.\r\n");
				status = NOLINK;
				close(sockfd);
				return (-1);
			}
			if (rc == 0) {
				ttystr("\r\nRJE184S The send timed out, host probably down.\r\n");
				return (-1);
			}
			if (line_in[0] == DLE && line_in[1] == ACK0)
				break;
			if (line_in[0] == DLE && line_in[1] == ACK1)
				break;
			if (line_in[0] == EOT)
				break;
			if (line_in[0] != NAK) {
				return (-1);
			}
			// it was a NAK -- return to try transmit again
			retry++;
			if (retry > 10) {
				ttystr("\r\nRJE186S 10 consecutive NAKs, giving up on send.\r\n");
				return (-1);
			}
		}
		if (strlen(command) > 0) {
			line_out[0] = line_out[1] = SYN;
			line_out[2] = EOT;
			line_out_size = 3;
			write_buffer();
			return (0);
		}
		count++;
		show++;
	}
}


// ---------------------------------------------------------------------------------
// This is code supporting the I/O to and from the line
// ---------------------------------------------------------------------------------

// Socket initialization (Windows only)

int InitSockets()
{
#if defined (_WIN32)
	int err;
	WORD wVersionRequested;
	WSADATA wsaData;

	wVersionRequested = MAKEWORD (1, 1); 
	err = WSAStartup(wVersionRequested, &wsaData);
	if (err != 0) {
		printf("\r\nRJE000T Winsock: Startup error number %d\n", err);
		return (-1);
	}
#endif
	return (0);
}

// Shutdown Sockets (Windows Only)

int CloseSockets()
{
	return (0);
}

// This function returns the next byte in the bisync input buffer.  If 
// there are no bytes there, it returns 0.  Once the byte is read,
// it's cleared from the buffer

unsigned char read_byte()
{
	unsigned char r;
	int i;
	
	if (line_in_ctr < 1)		/* Nothing? */
		return (0);		/* yep -- give up */
	r = line_in[0];			/* return first byte */
	line_in_ctr--;			/* and shift down the rest */
	if (line_in_ctr > 0) {
		for (i = 0; i < line_in_ctr; i++) {
			line_in[i] = line_in[i+1];
		}
	}
	return(r);	
}

// This function polls for data, and if there is some waiting,
// it returns 1, otherwise it returns 0.
// the parameters are the timeout values

int read_poll(int sec, int usec)
{
	struct timeval tv;
	fd_set readfdset;
	tv.tv_sec = sec;			/* Set timeout value */
	tv.tv_usec = usec;
	FD_ZERO(&readfdset);
	FD_SET(sockfd, &readfdset);
	select(sockfd+1, &readfdset, NULL, NULL, &tv);
	if (FD_ISSET(sockfd, &readfdset)) 	/* Data ready? */
		return (1);
	return (0);
}


// This is the read data function.  It listens on the line for data,
// and returns when it's got a bisync unpend character.  Those are:  
// ENQ, EOT, ETB, ETX.
// If we are in eib mode, NAK, ACK0, and ACK1
// are also unpend characters.
//
// The first two parameters are the timeout values, in seconds 
// and microseconds.  The third parameter determines if we are
// in eib (0), text (1), or transparent (2) mode.  The difference
// is when to end the read:  ACK0, ACK1, and NAK unpend in eib
// mode but not in the other two.  (eib mode is the mode when
// text is not being sent but controls are).
// Returns: -1 if the line is disconnected
//           0 if the read timed out
//           number of bytes read if otherwise

int read_data(int sec, int usec, int mode)
{
	int rc, i, j, k, unpend;
	char wstr[64];
	char traceline[1024];
	char transline[1024];
	struct timeval tv;
	fd_set readfdset;


	strcpy(traceline, "");
	clear_input_buffer();
	while (1) {
		tv.tv_sec = sec;			/* Set timeout value */
		tv.tv_usec = usec;
		FD_ZERO(&readfdset);
		FD_SET(sockfd, &readfdset);
		select(sockfd+1, &readfdset, NULL, NULL, &tv);
		if (FD_ISSET(sockfd, &readfdset)) {	/* Data ready? */
			rc = get_buffer();		/* We have data */
			if (rc < 0)			/* No we dont - disconnect */
				return (-1);
		} else {
			return (0);			/* We timed out */
		}

		// Copy from the physical buffer to the logical,
		// stop when you hit an unpend character.  If the
		// buffer empties without one, keep looking, other
		// wise return to the caller.

		unpend = 0;
		for (i = 0; i < phy_ctr; i++) {
			line_in[line_in_ctr] = phybuffer[i];
			line_in_ctr++;
			if (phybuffer[i] == ENQ ||
			    phybuffer[i] == EOT ||
			    phybuffer[i] == ETX ||
			    phybuffer[i] == ETB) {
				unpend = 1;
				break;
			}
			if (mode == 0 && (phybuffer[i] == NAK ||
				phybuffer[i] == ACK0 ||
				phybuffer[i] == ACK1)) {
				unpend = 1;
				break;
			}
		}
		if (i < phy_ctr) {
			k = 0;	/* shift stuff still in buffer down */
			for(j = i; j < phy_ctr; j++) {
				phybuffer[k] = phybuffer[j];
				phybuffer[j] = 0;
			}
			phy_ctr = k;
		} else {
			phy_ctr = 0;
		}	
		if (unpend == 1)
			break;
	}
	if (strlen(tracefile) > 0) {
		for (i = 0; i < 1024; i++) {transline[i] = 0;}
		for (i = 0; i < 1024; i++) {traceline[i] = 0;}
		for (i = 0; i < line_in_ctr; i++) {
			transline[i] = line_in[i];
		}
		translate_to_ascii(transline);
		strcpy(traceline, "RECV: ");
		j = 6;
		for (i = 0; i < line_in_ctr; i++) {
			if (isprint(transline[i]) && transline[i] > 0x1f) {
				traceline[j] = transline[i];
				j++;
			} else {
				traceline[j] = ' ';
				j++;
			}
		}
		strcat(traceline, "\n");
		fputs(traceline, tracefd);
		strcpy(traceline, "      ");
		j = 6;
		for (i = 0; i < line_in_ctr; i++) {
			sprintf(wstr, "%02x", line_in[i]);
			traceline[j] = wstr[0];
			j++;
		}
		//strcat(traceline, "\n");
		fputs(traceline, tracefd);
		strcpy(traceline, "      ");
		j = 6;
		for (i = 0; i < line_in_ctr; i++) {
			sprintf(wstr, "%02x", line_in[i]);
			traceline[j] = wstr[1];
			j++;
		}
		strcat(traceline, "\n");
		fputs(traceline, tracefd);
	}
	return (line_in_ctr);
}

// This function tries to read something from the line.  Whatever
// it reads, it adds to the physical buffer (phybuffer).  It returns
// the number of characters added to the buffer, or -1 if an error.


int get_buffer()
{
	int i, rc, err, count;
	char wstr[64];
	unsigned char inbuffer[256];
	
	if (status == NOLINK || status == SHUTDOWN)
		return (-1);

	rc = recv(sockfd, inbuffer, 256, 0);
	if (rc == 0) return (-1);	/* disconnect */

#if defined (_WIN32)			// Windows

	if (rc == SOCKET_ERROR) {
		err = WSAGetLastError();
		if (err == WSAEWOULDBLOCK)
			return (0);			/* no data */
		printf("\nWindows socket error: %d\n", err);
		return (-1);
	}

#else					// Linux

	if (rc < 0 && errno == EAGAIN)
		rc = 0;			/* No data */
#endif

	count = 0;
	if (rc > 0) {
		if (debugit)
			ttystr("\r\nData received: ");
		for (i = 0; i < rc; i++) {	/* Consider each character */
			if (inbuffer[i] == SYN || inbuffer[i] == EPAD || inbuffer[i] == SPAD)
				continue;
			if (debugit) {	
				sprintf(wstr,"%2x",inbuffer[i]);
				ttystr(wstr);
			}		
			phybuffer[phy_ctr] = inbuffer[i];
			phy_ctr++;
			count++;
		}
	}

	if (rc < 0)
		return (-1);
	return (count);		
}



// Clear the input buffer

int clear_input_buffer()
{
	int i;
	
	for (i = 0; i < 1024; i++) {
		line_in[i] = 0;
	}
	line_in_ctr = 0;
	return (0);	
}

// Clear the input record

int clear_input_record()
{
	int i;
	
	for (i = 0; i < 1024; i++) {
		record_in[i] = 0;
	}
	record_ctr = 0;
	return (0);	
}

// Write the data record to the output file

int write_record()
{
	char output_data[512];
	char print_line[256];
	unsigned char printer_action;
	int i, j, h;
	
	for (i = 0; i < 512; i++) {output_data[i] = 0;}
	if (device_select == 0) {
		if (strlen(print) > 0 && print_open == 0) {
			print_open = 1;
			printfd = fopen(print, "a");
		}

		// Is this is horizontal tabs record?  If so store it.

		if (record_in[0] == 0x27 && record_in[1] == 0x05) {
			for (i = 0; i < 256; i++) {htabs[i] = 0;}
			j = 2;
			for (i = 0; i < 256; i++) {
				if (record_in[j] != 0x40 &&
				    record_in[j] != 0x05)
					break;
				htabs[i] = record_in[j];
				j++;
			}
			i = 0;
			while (j < record_ctr) {
				record_in[i] = record_in[j];
				i++;
				j++;
			}
			record_ctr = i;
			if (i < 1)
				return (0);	/* DO NOT print it */
		}


		// check for vertical forms controls and store if found

		j = 0;
		for (i = 0; i < record_ctr; i++) {
			switch (record_in[i]) {
			case 0x27: 
				i++;
				printer_action = record_in[i];
				break;
			case 0x05:
				while (1) {
					output_data[j] = 0x40;
					j++;
					if (htabs[j] != 0x40)
						break;
				}
				break;
			default:
				output_data[j] = record_in[i];
				j++;
				break;
			}
		}
		translate_to_ascii(output_data);
		j = strlen(output_data);
		while (output_data[j] == ' ' && j > 1) {
			output_data[j] = 0;
			j--;
		}
		switch(printer_action) {
			case 0x61:	/* single space */
				strcpy(print_line, "\r\n");
				break;
			case 0xe2:	/* double space */
				strcpy(print_line, "\r\n\r\n");
				break;
			case 0xe3:	/* triple space */
				strcpy(print_line, "\r\n\r\n\r\n");
				break;
			case 0xc1:	/* Top of page */
				strcpy(print_line, "\r\n\014");
				break;
			case 0xd4:	/* Suppress spacing */
				strcpy(print_line, "\r");
				break;
			default:
				strcpy(print_line, "\r\n");
				break;
		}
		strcat(print_line, output_data);
		if (strlen(print) == 0) {
			ttystr(print_line);
		} else {
			fputs(print_line, printfd);
		}
	} else {
		if (strlen(punch) > 0 && punch_open == 0) {
			punch_open = 1;
			punchfd = fopen(punch, "a");
		}

		for (i = 0; i < punch_recl; i++) {
			if (i <= record_ctr)
				output_data[i] = record_in[i];
			else
				output_data[i] = 0x40;
		}
		if (punch_fmt == 1) {
			fwrite(output_data, punch_recl, 1, punchfd);
		} else {
			translate_to_ascii(output_data);
			j = strlen(output_data);
			while (output_data[j] == ' ' && j > 1) {
				output_data[j] = 0;
				j--;
			}
			strcat(output_data, "\n");
			fputs(output_data, punchfd);
		}
	}
	clear_input_record();	
	return (0);
}


// Write the output buffer to the line 

int write_buffer()
{
	int i, j, rc;
	char diswrite[1024];
	char hexch[32];
	char wstr[32];
	char traceline[1024];
	char transline[1024];
	unsigned char trndata[1024];
	int xlate = 0;

	strcpy(traceline, "");
	if (opt_trn == 1) {	// transparent?
		j = 0;		// yes -- insert DLEs
		for (i = 0; i < line_out_size; i++) {
			if (line_out[i] == STX)
				xlate = 1;
			if (line_out[i] < 0x40 &&
				xlate == 1) {
				trndata[j] = DLE;
				j++;
			}
			if (line_out[i] == ETX)
				xlate = 0;
			trndata[j] = line_out[i];
			j++;
		}
		for (i = 0; i < j; i++) {
			line_out[i] = trndata[i];
		}
		line_out_size = j;
	}
	rc = send(sockfd, line_out, line_out_size, 0);
	if (debugit) {
		strcpy(diswrite, "");
		for (i = 0; i < rc; i++) {
			sprintf(hexch, "%02x", line_out[i]);
			strcat(diswrite, hexch);
		}
		ttystr("\r\nWrote (");
		sprintf(hexch, "%d bytes):", rc);
		ttystr(hexch);
		ttystr(diswrite);	
		if (rc != line_out_size) {
			ttystr(" INCOMPLETE WRITE, ");
			sprintf(hexch, "%d%", (line_out_size - rc));
			ttystr(hexch);
			ttystr(" short!");
		}
	}	
	if (strlen(tracefile) > 0) {
		for (i = 0; i < 1024; i++) {transline[i] = 0;}
		for (i = 0; i < 1024; i++) {traceline[i] = 0;}
		for (i = 0; i < line_out_size; i++) {
			transline[i] = line_out[i];
		}
		translate_to_ascii(transline);
		strcpy(traceline, "SEND: ");
		j = 6;
		for (i = 0; i < line_out_size; i++) {
			if (isprint(transline[i]) && transline[i] > 0x1f) {
				traceline[j] = transline[i];
				j++;
			} else {
				traceline[j] = ' ';
				j++;
			}
		}
		strcat(traceline, "\n");
		fputs(traceline, tracefd);
		strcpy(traceline, "      ");
		j = 6;
		for (i = 0; i < line_out_size; i++) {
			sprintf(wstr, "%02x", line_out[i]);
			traceline[j] = wstr[0];
			j++;
		}
		//strcat(traceline, "\n");
		fputs(traceline, tracefd);
		strcpy(traceline, "      ");
		j = 6;
		for (i = 0; i < line_out_size; i++) {
			sprintf(wstr, "%02x", line_out[i]);
			traceline[j] = wstr[1];
			j++;
		}
		strcat(traceline, "\n");
		fputs(traceline, tracefd);
	}
	return (0);
}

// Send an ACK after a record is received

int send_ack(char ack)
{
	line_out[0] = line_out[1] = SYN;	/* Tell remote we're OK */
	line_out[2] = DLE;
	if (ack == 0) {
		if (lastack == ACK0)	/* Be sure and send correct ACK */
			lastack = ACK1;
			else
			lastack = ACK0;
	} else {
		lastack = ack;
	}			
	line_out[3] = lastack;
	line_out_size = 4;
	write_buffer();
	return (0);
}

// ---------------------------------------------------------------------------------
// This is the TTY code -- mostly from SIMH -- that handles I/O to the local tty
// -----------------------------------------------------------------------------

#if defined (_WIN32)

int ttyinit()
{
	return (0);
}

int ttyclose()
{
	return (0);
}

int ttygets(char *str)
{
	int i = 0, rc;
	unsigned char buf[1];

	while (1) {
		rc = ttyread(buf);
		rjesleep(100);
		if (rc == 0)
			continue;
		if (buf[0] == '\b') {
			ttystr("\b \b");
			i--;
			continue;
		}
		str[i] = buf[0];
		i++;
		ttychar(buf[0]);
		if (buf[0] == '\n' ||
			buf[0] == '\r' ||
			buf[0] == '\004') {
			ttystr("\r\n");
			break;
		}
	}
	return (i);
}

int ttyread(unsigned char *buf)
{
	if (macro_ctr < macro_size) {
		buf[0] = macro[macro_ctr];
		macro_ctr++;
		return (1);
	}
	if (!kbhit ())
		return (0);
	buf[0] = _getch ();
	return (1);
}

int ttychar(char c)
{
	_putch(c);
	return (0);
}

int ttystr(char *msg)
{
	unsigned int i;
	for (i = 0; i < strlen(msg); i++) {
		ttychar(msg[i]);
	}
	return (0);
}

int rjesleep(int t)
{
	return (0);
}

#else

// 1. Init the local TTY

int ttyinit() {
	if (!isatty (fileno (stdin))) 
		return (0);			/* skip if !tty */
	if (tcgetattr (0, &cmdtty) < 0) {
		ttychar('!');
		return (1);			/* get old flags */
	}	
	runtty = cmdtty;
	runtty.c_lflag = runtty.c_lflag & ~(ECHO | ICANON);	/* no echo or edit */
	runtty.c_oflag = runtty.c_oflag & ~OPOST;		/* no output edit */
	runtty.c_iflag = runtty.c_iflag & ~ICRNL;		/* no cr conversion */
	runtty.c_cc[VINTR] = 3;					/* interrupt */
	runtty.c_cc[VQUIT] = 0;					/* no quit */
	runtty.c_cc[VERASE] = 0;
	runtty.c_cc[VKILL] = 0;
	runtty.c_cc[VEOF] = 0;
	runtty.c_cc[VEOL] = 0;
	runtty.c_cc[VSTART] = 0;				/* no host sync */
	runtty.c_cc[VSUSP] = 0;
	runtty.c_cc[VSTOP] = 0;
	runtty.c_cc[VMIN] = 0;					/* no waiting */
	runtty.c_cc[VTIME] = 0;
	if (tcsetattr (0, TCSAFLUSH, &runtty) < 0) return (1);
	return (0);
}	

// Reset the tty to normal, please

int ttyclose()
{
	if (tcsetattr (0, TCSAFLUSH, &cmdtty) < 0) return (1);
	return (0);
}

int ttygets(char *str)
{
	int i = 0, rc;
	unsigned char buf[1];

	while (1) {
		rc = ttyread(buf);
		rjesleep(100);
		if (rc == 0)
			continue;
		if (buf[0] == '\b') {
			ttystr("\b \b");
			i--;
			continue;
		}
		str[i] = buf[0];
		i++;
		ttychar(buf[0]);
		if (buf[0] == '\n' ||
			buf[0] == '\r' ||
			buf[0] == '\004') {
			ttystr("\r\n");
			break;
		}
	}
	return (i);
}

int ttyread(unsigned char buf[])
{
	int r;

	r = read(0, buf, 1);
	return (r);
}

// Write a char on the local TTY 

int ttychar(char c)
{
	int lc;
	
	lc = c;
	write(1, &c, 1);
	return(0); 
}

// Write a string on the local TTY

int ttystr(char *msg)
{
	int i;
	for (i = 0; i < strlen(msg); i++) {
		ttychar(msg[i]);
	}
}

int rjesleep(int t)
{
//	usleep(t);
	return (0);
}

#endif

// ------------------------------------------------------------------------------
// Code translation services
// ------------------------------------------------------------------------------

/*-------------------------------------------------------------------*/
/* SUBROUTINE TO TRANSLATE A NULL-TERMINATED STRING TO EBCDIC        */
/*-------------------------------------------------------------------*/
char *translate_to_ebcdic (unsigned char *str)
{
int     i;                              /* Array subscript           */
unsigned char c;                        /* Character work area       */

    for (i = 0; str[i] != '\0'; i++)
    {
        c = str[i];
        str[i] = ascii_to_ebcdic[c];
    }

    return str;
}

/*-------------------------------------------------------------------*/
/* SUBROUTINE TO TRANSLATE A NULL-TERMINATED STRING TO ASCII         */
/*-------------------------------------------------------------------*/
char *translate_to_ascii (unsigned char *str)
{
int     i;                              /* Array subscript           */
unsigned char c;                        /* Character work area       */

    for (i = 0; str[i] != '\0'; i++)
    {
        c = str[i];
        str[i] = ebcdic_to_ascii[c];
    }

    return str;
}
