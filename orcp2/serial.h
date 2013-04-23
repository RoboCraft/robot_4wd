//
// OpenRoboVision
//
// working with serial
//
//
// robocraft.ru
//

#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <stdio.h>
#include <stdlib.h>

#if defined(WIN32)
# undef UNICODE
# define WIN32_LEAN_AND_MEAN
# include <windows.h>
#elif defined(LINUX)
# include <unistd.h>
# include <fcntl.h>		// file control options 
# include <termios.h>
# include <string.h>

# include <sys/types.h>
# include <errno.h>
# include <sys/select.h>
# include <sys/stat.h>
# include <sys/ioctl.h>
# include <linux/serial.h>
#endif

// buffer size
#ifndef SERIAL_NAME_BUF_SIZE
# define SERIAL_NAME_BUF_SIZE 256
#endif

#if defined(WIN32)
# define snprintf _snprintf_s
#endif //#if defined(WIN32)

#include "stream.h" // (may be removed without after-effects)

class Serial : public Stream
{
public:
	Serial();
	Serial(int port, int rate);
	Serial(const char* name, int rate, int databits=8, char parity='N', int stopbits=1);
	~Serial();

	// open the port
	int open(void);
	int open(const char* name, int rate);
	int open(int port, int rate);
#if defined(WIN32)
	// overlapped
	int open(int port, int rate, bool is_overlapped_) { is_overlapped=is_overlapped_; return open(port, rate);}
	int open(const char* name, int rate, bool is_overlapped_) { is_overlapped=is_overlapped_; return open(name, rate);}
#endif

	// close the port
	int close();

	// is port connected
	int connected();
	// how much bytes available for reading
	int available();

	// read from the serial port.  Returns only the bytes that are
	// already received, up to count.
	int read(void *ptr, int count);
	// write to the serial port.
	int write(const void *ptr, int len);
	int write(int val){ char buf[1]; buf[0] = val&0xff; return write(buf, 1); };

	// wait for all transmitted data with Write to actually leave the serial port
	void outputFlush();

	// discard all received data that hasn't been read 
	void discardInput();
	void discardOutput();

	// set the baud rate
	int setBaud(int baud);

	// wait up to msec for data to become available for reading.
	int waitInput(int msec);

	// get LSR (Line Status Register) (Linux only!)
	int getLSR();

	// wait until data doesnt transmit (Linux only!)
	int waitTxEmpty();
	
	// set DTR and RTS,  0=low, 1=high, -1=unchanged
	int setControl(int dtr, int rts);

#if defined(WIN32)
	HANDLE getPortHandle() const { return port_handle; }
#elif defined(LINUX)
	int getPortHandle() const { return port_fd; }
	void setLowLatency(bool val=true) { is_low_latency=val; }
#endif

	void setDataBits(int val) { databits=val; }
	void setStopBits(int val) { stopbits=val; }
	void setParity(char val) { parity=val; }
	
private:

	void init();
	// защита от случайного копирования объекта
	Serial(const Serial&);
	Serial& operator=(const Serial&);

	int port_is_open;
	char port_name[SERIAL_NAME_BUF_SIZE];
	int baud_rate;

	int databits;
	int stopbits;
	char parity;

	// OS-depending data:
#if defined(WIN32)
	HANDLE port_handle;
	COMMCONFIG port_cfg_orig;
	COMMCONFIG port_cfg;
	// overlapped operation
	bool is_overlapped;

#elif defined(LINUX)
	int port_fd;
	struct termios settings_orig;
	struct termios settings;
	bool is_low_latency;
#endif

};

#endif //#ifndef _SERIAL_H_
