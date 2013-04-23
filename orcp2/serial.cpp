//
// OpenRoboVision
//
// working with serial
//
//
// robocraft.ru
//

#include "serial.h"

//
// WIN32
//
// Работа с COM-портом с помощью потоков
//   http://www.piclist.ru/S-COM-THREAD-RUS/S-COM-THREAD-RUS.html
// 
// Synchronization and Overlapped Input and Output
//   http://msdn.microsoft.com/en-us/library/ms686358%28VS.85%29.aspx
//
//
// LINUX
// 
// Serial Programming Guide for POSIX Operating Systems
//   http://www.opennet.ru/docs/RUS/serial_guide/
//

Serial::Serial()
{
	init();
}

Serial::Serial(int port, int rate)
{
	init();
	open(port, rate);
}

Serial::Serial(const char* name, int rate, int _databits, char _parity, int _stopbits)
{
	init();
	databits=_databits;
	parity=_parity;
	stopbits=_stopbits;
	open(name, rate);
}

Serial::~Serial()
{
	close();
}

void Serial::init()
{
	port_is_open = 0;
	databits = 8;
	parity = 'N';
	stopbits = 1;
#if defined(WIN32)
	port_handle = INVALID_HANDLE_VALUE;
	is_overlapped = false;
#elif defined(LINUX)
	port_fd = -1;
	is_low_latency = false;
#endif
}

// open a port, by name
int Serial::open(const char* name, int rate)
{
	if(!name)
		return -1;

	close();
	if(name!=port_name)
	{
		snprintf(port_name, SERIAL_NAME_BUF_SIZE, "%s", name);
	}
	baud_rate = rate;

#if defined(WIN32)

	COMMCONFIG cfg;
	COMMTIMEOUTS timeouts;
	int got_default_cfg=0, port_num;
	char buf[SERIAL_NAME_BUF_SIZE], name_createfile[SERIAL_NAME_BUF_SIZE], name_commconfig[SERIAL_NAME_BUF_SIZE], *p;
	DWORD len;

	snprintf(buf, SERIAL_NAME_BUF_SIZE, "%s", port_name);
	p = strstr(buf, "COM");
	if (p && sscanf_s(p + 3, "%d", &port_num) == 1)
	{
		snprintf(name_createfile, SERIAL_NAME_BUF_SIZE, "\\\\.\\COM%d", port_num);
		snprintf(name_commconfig, SERIAL_NAME_BUF_SIZE, "COM%d", port_num);
	}
	else
	{
		snprintf(name_createfile, SERIAL_NAME_BUF_SIZE, "%s", port_name);
		snprintf(name_commconfig, SERIAL_NAME_BUF_SIZE, "%s", port_name);
	}

	len = sizeof(COMMCONFIG);
	if (GetDefaultCommConfig(name_commconfig, &cfg, &len))
	{
		// this prevents unintentionally raising DTR when opening
		// might only work on COM1 to COM9
		got_default_cfg = 1;
		memcpy(&(port_cfg_orig), &cfg, sizeof(COMMCONFIG));
		cfg.dcb.fDtrControl = DTR_CONTROL_DISABLE;
		cfg.dcb.fRtsControl = RTS_CONTROL_DISABLE;
		SetDefaultCommConfig(name_commconfig, &cfg, sizeof(COMMCONFIG));
	}
	else
	{
		printf("[!][serial][open] Error: GetDefaultCommConfig\n");
	}

	port_handle = CreateFile(name_createfile, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, is_overlapped ? FILE_FLAG_OVERLAPPED : 0, NULL);
	if (port_handle == INVALID_HANDLE_VALUE)
	{
		printf("[!][serial][open] Error: Unable to open %s !\n", port_name);
		return -1;
	}
	len = sizeof(COMMCONFIG);
	if (!GetCommConfig(port_handle, &(port_cfg), &len))
	{
		CloseHandle(port_handle);
		port_handle = INVALID_HANDLE_VALUE;
		printf("[!][serial][open] Error: Unable to read communication config on %s !\n", port_name);
		return -1;
	}
	if (!got_default_cfg)
	{
		memcpy(&(port_cfg_orig), &(port_cfg), sizeof(COMMCONFIG));
	}
	// http://msdn2.microsoft.com/en-us/library/aa363188(VS.85).aspx
	port_cfg.dcb.BaudRate = baud_rate;
	port_cfg.dcb.fBinary = TRUE;
	port_cfg.dcb.fParity = FALSE;
	port_cfg.dcb.fOutxCtsFlow = FALSE;
	port_cfg.dcb.fOutxDsrFlow = FALSE;
	port_cfg.dcb.fDtrControl = DTR_CONTROL_DISABLE;
	port_cfg.dcb.fDsrSensitivity = FALSE;
	port_cfg.dcb.fTXContinueOnXoff = FALSE;
	port_cfg.dcb.fOutX = FALSE;
	port_cfg.dcb.fInX = FALSE;
	port_cfg.dcb.fErrorChar = FALSE;
	port_cfg.dcb.fNull = FALSE;
	port_cfg.dcb.fRtsControl = RTS_CONTROL_DISABLE;
	port_cfg.dcb.fAbortOnError = FALSE;
	// The number of bits in the bytes transmitted and received.
	port_cfg.dcb.ByteSize = databits;
	// The parity scheme to be used. 
	switch(parity)
	{
		case 'O':
			port_cfg.dcb.fParity = TRUE;
			port_cfg.dcb.Parity = ODDPARITY;
			break;
		case 'E':
			port_cfg.dcb.fParity = TRUE;
			port_cfg.dcb.Parity = EVENPARITY;
			break;
		case 'M':
			port_cfg.dcb.fParity = TRUE;
			port_cfg.dcb.Parity = MARKPARITY;
			break;
		case 'S':
			port_cfg.dcb.fParity = TRUE;
			port_cfg.dcb.Parity = SPACEPARITY;
			break;
		case 'N':
		default:
			port_cfg.dcb.Parity = NOPARITY;
			break;
	}
	// The number of stop bits to be used.
	switch(stopbits)
	{
		case 1:
			port_cfg.dcb.StopBits = ONESTOPBIT;
			break;
		case 15:
			port_cfg.dcb.StopBits = ONE5STOPBITS;
			break;
		case 2:
			port_cfg.dcb.StopBits = TWOSTOPBITS;
			break;
		default:
			port_cfg.dcb.StopBits = ONESTOPBIT;
			break;
	}

	//port_cfg.dcb.XonChar = 0;
	//port_cfg.dcb.XoffChar = (char)0xff;

	if (!SetCommConfig(port_handle, &(port_cfg), sizeof(COMMCONFIG)))
	{
		CloseHandle(port_handle);
		port_handle = INVALID_HANDLE_VALUE;
		printf("[!][serial][open] Error: Unable to write communication config to %s !\n", port_name);
		return -1;
	}
	if (!EscapeCommFunction(port_handle, CLRDTR | CLRRTS))
	{
		CloseHandle(port_handle);
		port_handle = INVALID_HANDLE_VALUE;
		printf("[!][serial][open] Error: Unable to control serial port signals on %s !\n", port_name);
		return -1;
	}
	// http://msdn2.microsoft.com/en-us/library/aa363190(VS.85).aspx
	// setting to all zeros means timeouts are not used
	//timeouts.ReadIntervalTimeout		= 0;
	timeouts.ReadIntervalTimeout		= MAXDWORD;
	timeouts.ReadTotalTimeoutMultiplier	= 0;
	timeouts.ReadTotalTimeoutConstant	= 0;
	timeouts.WriteTotalTimeoutMultiplier	= 0;
	timeouts.WriteTotalTimeoutConstant	= 0;
	if (!SetCommTimeouts(port_handle, &timeouts)) {
		CloseHandle(port_handle);
		printf("[!][serial][open] Error: Unable to write timeout settings to %s !\n", port_name);
		return -1;
	}

	// clear buffers for receive and transmit
	PurgeComm(port_handle, PURGE_RXCLEAR); 
	PurgeComm(port_handle, PURGE_TXCLEAR);

#elif defined(LINUX)

	/**
	* IOCTL	Запросы к последовательному порту 
	* Запрос		Описание													Функция POSIX
	*
	* TCGETS 		Получить текущие настройки последовательного порта. 		tcgetattr
	* TCSETS 		Установить настройки последовательного порта немедленно. 	tcsetattr(fd, TCSANOW, &options)
	* TCSETSF		Установить настройки последовательного порта после очистки 
	*				буферов ввода/вывода. 										tcsetattr(fd, TCSAFLUSH, &options)
	* TCSETSW		Установить настройки последовательного порта после того 
	*				как буфера ввода/вывода освободятся. 						tcsetattr(fd, TCSADRAIN, &options)
	* TCSBRK 		Послать на определенное время разрыв линии (break). 		tcsendbreak, tcdrain
	* TCXONC 		Программное управление потоком данных. 						tcflow
	* TCFLSH 		Очистка очередей ввода/вывода. 								tcflush
	* TIOCMGET 		Вернуть состояние "модемных" битов. 						None
	* TIOCMSET 		Установить состояние "модемных" битов. 						None
	* FIONREAD 		Вернуть количество байтов в буфере ввода (приемный буфер). 	None
	/**/

	struct serial_struct kernel_serial_settings;
	int bits;
	port_fd = ::open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (port_fd < 0)
	{

		printf("[!][serial][open] Error: ");

		if (errno == EACCES)
		{
			printf("Unable to access %s - insufficient permission!\n", port_name);

		}
		else if (errno == EISDIR)
		{
			printf("Unable to open %s - Object is a directory, not a serial port!\n", port_name);
		}
		else if (errno == ENODEV || errno == ENXIO)
		{
			printf("Unable to open %s - Serial port hardware not installed!\n", port_name);
		}
		else if (errno == ENOENT)
		{
			printf("Unable to open %s - Device name does not exist!\n", port_name);
		}
		else
		{
			printf("Unable to open %s - %s!\n", port_name, strerror(errno));
		}
		return -1;
	} 

#if 0
	// получения статусных битов
	if (ioctl(port_fd, TIOCMGET, &bits) < 0)
	{
		close(port_fd);
		printf("[!][serial][open] Error: Unable to query serial port signals!\n");
		return -1;
	}

	// TIOCM_LE	DSR (data set ready/line enable)
	// TIOCM_DTR	DTR (data terminal ready)
	// TIOCM_RTS	RTS (request to send)
	// TIOCM_ST		Secondary TXD (transmit)
	// TIOCM_SR		Secondary RXD (receive)
	// TIOCM_CTS	CTS (clear to send)
	// TIOCM_CAR	DCD (data carrier detect)
	// TIOCM_CD		Synonym for TIOCM_CAR
	// TIOCM_RNG	RNG (ring)
	// TIOCM_RI		Synonym for TIOCM_RNG
	// TIOCM_DSR	DSR (data set ready)

	// cброс сигнала DTR и RTS
	bits &= ~(TIOCM_DTR | TIOCM_RTS);
	if (ioctl(port_fd, TIOCMSET, &bits) < 0)
	{
		::close(port_fd);
		printf("[!][serial][open] Error: Unable to control serial port signals!\n");
		return -1;
	}
#endif

	// получение текущих опций для порта
	if (tcgetattr(port_fd, &(settings_orig)) != 0)
	{
		::close(port_fd);
		port_fd = -1;
		printf("[!][serial][open] Error: Unable to query serial port settings (perhaps not a serial port)!\n");
		return -1;
	}
	memset(&(settings), 0, sizeof(settings));

	//
	// настройки работы портом
	//

	//
	// for more info: man termios
	// 

	//
	// termios structure
	//
	// tcflag_t c_iflag;      // input modes
	// tcflag_t c_oflag;      // output modes
	// tcflag_t c_cflag;      // control modes
	// tcflag_t c_lflag;      // local modes
	// cc_t     c_cc[NCCS];   // control chars
	
	/**
	* c_cflag	Управляющие опции
	* c_lflag	Опции линии
	* c_iflag	Опции ввода
	* c_oflag	Опции вывода
	* c_cc		Управляющие символы
	* c_ispeed	Скорость ввода в бодах (новый интерфейс)
	* c_ospeed	Скорость вывода в бодах (новый интерфейс)
	*
	/**/

	//
	// установка опций вывода
	//
	settings.c_oflag &= ~OPOST; // необработанный (raw) вывод

	// установка опций обработки ввода из порта
	//settings.c_iflag |= (INPCK | ISTRIP);
	// IGNBRK Ignore BREAK condition on input
	// IGNPAR Ignore framing errors and parity errors.

	// отмена программно управляемого управления потоком передачи данных
	//settings.c_iflag &= ~(IXON | IXOFF | IXANY);
	// IXON	Enable software flow control (outgoing)
	// IXOFF	Enable software flow control (incoming)
	// IXANY	Allow any character to start flow again

	//settings.c_iflag|=BRKINT;
	// BRKINT	Send a SIGINT when a break condition is detected

	// управляющие символы

	//settings.c_cc[VMIN] = 0; 
	//settings.c_cc[VTIME] = 0;

	//
	// опции линии
	//
	settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // неканонический ввод

	//
	// установка управляющих опций
	//

	if(parity=='N')
		settings.c_cflag &= ~PARENB;	// no parity
	else
		settings.c_cflag |= PARENB;
	if(stopbits==1)
		settings.c_cflag &= ~CSTOPB; // 1 
	else
		settings.c_cflag |= CSTOPB; // 2 
	settings.c_cflag &= ~CSIZE;
	switch(databits)
	{
		case 5:
			settings.c_cflag |= CS5;
			break;
		case 6:
			settings.c_cflag |= CS6;
			break;
		case 7:
			settings.c_cflag |= CS7;
			break;
		case 8:
		default:
			settings.c_cflag |= CS8; // 8 bits
			break;
	}
	settings.c_cflag |= CREAD;
	settings.c_cflag |= CLOCAL;

	// PARENB Enable parity bit
	// CSTOPB 2 stop bits (1 otherwise)
	// CSIZE  Character size mask.  Values are CS5, CS6, CS7, or CS8.
	// CREAD  Enable receiver
	// HUPCL  Lower modem control lines after last process closes  the  device (hang up).
	// CLOCAL Ignore modem control lines.
	// CRTSCTS (not  in  POSIX)  Enable  RTS/CTS   (hardware)   flow   control.

	// установка скорости
	setBaud(rate);

#if 1
	if(is_low_latency)
	{
		// http://www.opennet.ru/man.shtml?topic=setserial&category=8&russian=0

		// Минимизировать время ожидания приёма данных последовательным портом 
		// ценой большего использования ресурсов CPU. 
		// (Обычно, для снижения накладных расходов, среднее время ожидания равно 5-10 мс, 
		// прежде чем символы начнут передаваться.)
		if (ioctl(port_fd, TIOCGSERIAL, &kernel_serial_settings) == 0)
		{
			kernel_serial_settings.flags |= ASYNC_LOW_LATENCY;
			ioctl(port_fd, TIOCSSERIAL, &kernel_serial_settings);
		}
	}
#endif

	// очистка очереди
	tcflush(port_fd, TCIOFLUSH);

#endif

	port_is_open=1;
	setBaud(rate);

	return 0;
}

int Serial::open(void)
{
	return this->open(port_name, baud_rate);
}

// open a port, by number
int Serial::open(int port, int rate)
{
#if defined(WIN32)
	snprintf(port_name, SERIAL_NAME_BUF_SIZE, "COM%d", port);
#elif defined(LINUX)
	snprintf(port_name, SERIAL_NAME_BUF_SIZE, "/dev/ttyS%d", port);
#endif

	baud_rate = rate;

	return this->open(port_name, rate);
}

// Close the port
int Serial::close()
{
	if (!port_is_open)
		return -1;

	port_is_open = 0;

#if defined(WIN32)

	if(port_handle == INVALID_HANDLE_VALUE)
		return -1;
	
	SetCommConfig(port_handle, &port_cfg_orig, sizeof(COMMCONFIG));
	CloseHandle(port_handle);
	return 0;

#elif defined(LINUX)

	if(port_fd <= 0)
		return -1;

	tcsetattr(port_fd, TCSANOW, &(settings_orig));
	if(!::close(port_fd))
	{
		port_fd = 0;
		return 0;
	}
	else
	{
		return -1;
	}

#endif
}

int Serial::connected()
{
	return port_is_open;
}

int Serial::available()
{
	if (!port_is_open)
		return 0;
	
#if defined(WIN32)
	// first, we'll find out how many bytes have been received
	// and are currently waiting for us in the receive buffer.
	//   http://msdn.microsoft.com/en-us/library/ms885167.aspx
	//   http://msdn.microsoft.com/en-us/library/ms885173.aspx
	//   http://source.winehq.org/WineAPI/ClearCommError.html

	int numBytes = 0;

	COMSTAT stat;
	DWORD err;
	if(port_handle != INVALID_HANDLE_VALUE)
	{
		if(!ClearCommError(port_handle, &err, &stat))
		{
			numBytes = 0;
		}
		else
		{
			numBytes = stat.cbInQue;
		}
	}
	else
	{
		numBytes = 0;
	}

	return numBytes; 
#elif defined(LINUX)
	if(port_fd <= 0)
		return 0;

	int bytes = 0;
	ioctl(port_fd, FIONREAD, &bytes);
	return bytes;
#endif
	return 0;
}

// set the baud rate
int Serial::setBaud(int baud)
{
	if (baud <= 0 || !port_is_open)
		return -1;

	printf("[i][serial] set baud: %d\n", baud);
	baud_rate = baud;
	
#if defined(WIN32)

	if(port_handle == INVALID_HANDLE_VALUE)
		return -1;

	DWORD len = sizeof(COMMCONFIG);
	port_cfg.dcb.BaudRate = baud;
	SetCommConfig(port_handle, &port_cfg, len);
#elif defined(LINUX)

	if(port_fd <= 0)
		return -1;

	speed_t spd;
	switch (baud)
	{
		case 230400:
			spd = B230400;
			break;
		case 115200:
			spd = B115200;
			break;
		case 57600:
			spd = B57600;
			break;
		case 38400:	
			spd = B38400;
			break;
		case 19200:	
			spd = B19200;
			break;
		case 9600:	
			spd = B9600;
			break;
		case 4800:	
			spd = B4800;
			break;
		case 2400:	
			spd = B2400;
			break;
		case 1800:	
			spd = B1800;
			break;
		case 1200:
			spd = B1200;
			break;
		case 600:
			spd = B600;
			break;
		case 300:
			spd = B300;
			break;
		case 200:
			spd = B200;
			break;
		case 150:
			spd = B150;
			break;
		case 134:
			spd = B134;
			break;
		case 110:
			spd = B110;
			break;
		case 75:
			spd = B75;
			break;
		case 50:
			spd = B50;
			break;
#ifdef B460800
		case 460800:	spd = B460800;	break;
#endif
#ifdef B500000
		case 500000:	spd = B500000;	break;
#endif
#ifdef B576000
		case 576000:	spd = B576000;	break;
#endif
#ifdef B921600
		case 921600:	spd = B921600;	break;
#endif
#ifdef B1000000
		case 1000000:	spd = B1000000;	break;
#endif
#ifdef B1152000
		case 1152000:	spd = B1152000;	break;
#endif
#ifdef B1500000
		case 1500000:	spd = B1500000;	break;
#endif
#ifdef B2000000
		case 2000000:	spd = B2000000;	break;
#endif
#ifdef B2500000
		case 2500000:	spd = B2500000;	break;
#endif
#ifdef B3000000
		case 3000000:	spd = B3000000;	break;
#endif
#ifdef B3500000
		case 3500000:	spd = B3500000;	break;
#endif
#ifdef B4000000
		case 4000000:	spd = B4000000;	break;
#endif
#ifdef B7200
		case 7200:	spd = B7200;	break;
#endif
#ifdef B14400
		case 14400:	spd = B14400;	break;
#endif
#ifdef B28800
		case 28800:	spd = B28800;	break;
#endif
#ifdef B76800
		case 76800:	spd = B76800;	break;
#endif
		default:
			return -1;
			break;
	}
	// cfsetospeed() sets the output baud rate stored in the termios structure
	//  The  zero baud rate, B0, is used to terminate the connection
	cfsetospeed(&(settings), spd);

	//  cfsetispeed() sets the input baud rate stored in the termios  structure
	cfsetispeed(&(settings), spd);

	// tcsetattr()  sets  the  parameters associated with the terminal (unless
	//   support is required from the underlying hardware that is not available)
	//   from  the termios structure referred to by termios_p.  optional_actions
	//   specifies when the changes take effect:

	// TCSANOW the change occurs immediately.

	if (tcsetattr(port_fd, TCSANOW, &(settings)) < 0){
		return -1; 
	}
#endif
	return 0;
}

// Read from the serial port.  Returns only the bytes that are
// already received, up to count.  This always returns without delay,
// returning 0 if nothing has been received
int Serial::read(void *ptr, int count)
{
	if (!port_is_open)
		return -1;
	
	if (!ptr || count <= 0)
		return -1;
	
#if defined(WIN32)
	//   http://msdn.microsoft.com/en-us/library/ms885167.aspx
	//   http://msdn.microsoft.com/en-us/library/ms885173.aspx
	//   http://source.winehq.org/WineAPI/ClearCommError.html

	if(port_handle == INVALID_HANDLE_VALUE)
		return -1;
	
	DWORD errmask=0, num_read, num_request;
	int r;

	num_request =  (DWORD)count;

	if(!is_overlapped)
	{
		if (ReadFile(port_handle, ptr, num_request, &num_read, 0))
		{
			r = num_read;
		} 
		else
		{
			printf("[!][serial][read] Error: read!\n");
			r = -1;
		}
	}
	else
	{
		// overlapped mode
		DWORD signal;
		OVERLAPPED rov = {0};
#if 0
		COMSTAT st;
		if (!ClearCommError(port_handle, &errmask, &st)){
			return -1;
		}
		if (st.cbInQue <= 0){
			return 0;
		}
		// now do a ReadFile, now that we know how much we can read
		// a blocking (non-overlapped) read would be simple, but win32
		// is all-or-nothing on async I/O and we must have it enabled
		// because it's the only way to get a timeout for WaitCommEvent
		num_request = ((DWORD)count < st.cbInQue) ? (DWORD)count : st.cbInQue; 
#endif

		rov.hEvent = CreateEvent(NULL, true, false, NULL);

		// если hEvent не создался - выходим 
		if (rov.hEvent == 0)
			return -1;

		// читаем сколько надо байт
		if (ReadFile(port_handle, ptr, num_request, &num_read, &rov))
		{
			// this should usually be the result, since we asked for
			// data we knew was already buffered
			r = num_read;
		} 
		else
		{
			if (GetLastError() == ERROR_IO_PENDING)
			{
				signal = WaitForSingleObject(rov.hEvent, INFINITE);
				if (signal == WAIT_OBJECT_0)
				{
					if(GetOverlappedResult(port_handle, &rov, &num_read, true))
						r = num_read;
					else
						r = -1;
				}
				else
					r = -1;
			}
			else
			{
				r = -1;
			}
		} 
		// close hEvent
		CloseHandle(rov.hEvent);
	}
	return r;

#elif defined(LINUX)

	if(port_fd <= 0)
		return -1;

	int n, bits;
	n = ::read(port_fd, ptr, count);
	if (n < 0 && (errno == EAGAIN || errno == EINTR))
	{
		// EAGAIN 
		// Non-blocking I/O has been selected using O_NONBLOCK and no data was immediately available for reading. 
		// EINTR
		// The call was interrupted by a signal before any data was read; see signal(7). 
		return 0;
	}
	if (n == 0 && ioctl(port_fd, TIOCMGET, &bits) < 0)
	{
		return -99;
	}
	return n;

#endif
	return 0;
}

// write to the serial port. 
int Serial::write(const void *ptr, int len)
{
	//printf("Write %d\n", len);
	if (!port_is_open)
		return -1;

#if defined(WIN32)
	if(port_handle == INVALID_HANDLE_VALUE)
		return -1;

	DWORD num_written;
	int r;

	if(!is_overlapped)
	{
		if (WriteFile(port_handle, ptr, len, &num_written, 0))
		{
			r = num_written;
		}
		else
		{
			printf("[!][serial][write] Error: write!\n");
			r = -1;
		}
	}
	else
	{
		// overlapped mode
		DWORD signal;	// переменная-заглушка
		OVERLAPPED wov = {0};
		wov.hEvent = CreateEvent(NULL, true, true, NULL);  //создать событие
		if(wov.hEvent == 0)
			return -1;

		//записать байты в порт (перекрываемая операция!)
		if(!WriteFile(port_handle, ptr, len, &num_written, &wov))
		{
			if(GetLastError()== ERROR_IO_PENDING)
			{
				//приостановить поток, пока не завершится перекрываемая операция WriteFile
				signal = WaitForSingleObject(wov.hEvent, INFINITE);
				if (signal == WAIT_OBJECT_0){
					if( GetOverlappedResult(port_handle, &wov, &num_written, true) )
					{
						r = num_written;
					}
					else
					{
						r = -1;
					}
				}
				else
				{
					r = -1;
				}
			}
			else
			{
				r = -1;
			}
		}
		CloseHandle(wov.hEvent);
		r = num_written;
	}
	return r;
#elif defined(LINUX)

	if(port_fd <= 0)
		return -1;

	int n, written=0;
	fd_set wfds;
	struct timeval tv;
	while (written < len)
	{
		n = ::write(port_fd, (const char *)ptr + written, len - written);
		if (n < 0 && (errno == EAGAIN || errno == EINTR))
		{
			// EAGAIN
			// The file descriptor fd has been marked non-blocking (O_NONBLOCK) and the write would block. 
			// EINTR
			// The call was interrupted by a signal before any data was written; see signal(7). 
			n = 0;
		}
		if (n < 0)
		{
			return -1;
		}
		if (n > 0)
		{
			written += n;
		}
		else
		{
			tv.tv_sec = 10;
			tv.tv_usec = 0;
			FD_ZERO(&wfds);
			FD_SET(port_fd, &wfds);
			n = select(port_fd+1, NULL, &wfds, NULL, &tv);
			if (n < 0 && errno == EINTR)
			{
				n = 1;
			}
			if (n <= 0)
			{
				return -1;
			}
		}
	}

	return written; 
#endif
	return 0;
}

// wait up to msec for data to become available for reading.
// return 0 if timeout, or non-zero if one or more bytes are
// received and can be read.  -1 if an error occurs
int Serial::waitInput(int msec)
{
	if (!port_is_open)
		return -1;

#if defined(WIN32)
	// http://msdn2.microsoft.com/en-us/library/aa363479(VS.85).aspx
	// http://msdn2.microsoft.com/en-us/library/aa363424(VS.85).aspx
	// http://source.winehq.org/WineAPI/WaitCommEvent.html

	if(port_handle == INVALID_HANDLE_VALUE)
		return -1;

	COMSTAT st;
	DWORD errmask=0, eventmask=EV_RXCHAR, ret;
	OVERLAPPED ov = {0};
	int r;
	// first, request comm event when characters arrive
	if (!SetCommMask(port_handle, EV_RXCHAR))
		return -1;
	
	// look if there are characters in the buffer already
	if (!ClearCommError(port_handle, &errmask, &st))
		return -1;
	
	//printf("Input_wait, %lu buffered, timeout = %d ms\n", st.cbInQue, msec);
	if (st.cbInQue > 0)
		return 1;

	ov.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (ov.hEvent == NULL)
		return -1;

	if (WaitCommEvent(port_handle, &eventmask, &ov))
	{
		//printf("Input_wait, WaitCommEvent, immediate success\n");
		r = 1;
	} 
	else
	{
		if (GetLastError() == ERROR_IO_PENDING)
		{
			ret = WaitForSingleObject(ov.hEvent, msec);
			if (ret == WAIT_OBJECT_0)
			{
				//printf("Input_wait, WaitCommEvent, delayed success\n");
				r = 1;
			}
			else if (ret == WAIT_TIMEOUT)
			{
				//printf("Input_wait, WaitCommEvent, timeout\n");
				GetCommMask(port_handle, &eventmask);
				r = 0;
			}
			else
			{  // WAIT_FAILED or WAIT_ABANDONED
				printf("[!][serial] Error: Input_wait, WaitCommEvent, delayed error\n");
				r = -1;
			}
		}
		else
		{
			printf("[!][serial] Error: Input_wait, WaitCommEvent, immediate error\n");
			r = -1;
		}
	}
	SetCommMask(port_handle, 0);
	CloseHandle(ov.hEvent);
	return r;
#elif defined(LINUX)

	if(port_fd <= 0)
		return -1;
	
	fd_set rfds;
	struct timeval tv;
	tv.tv_sec = msec / 1000;
	tv.tv_usec = (msec % 1000) * 1000;

	if( tv.tv_usec > 1000000 )
	{
		tv.tv_sec++;
		tv.tv_usec -= 1000000;
	}

	FD_ZERO(&rfds);
	FD_SET(port_fd, &rfds);
	return select(port_fd+1, &rfds, NULL, NULL, &tv);

#endif
	return 0;
}

// get LSR (Line Status Register)
int Serial::getLSR()
{
	int res = -1;

	if (!port_is_open){
		return -1;
	}

#if defined(WIN32)
	// !TODO!
#elif defined(LINUX)

	if(port_fd <= 0)
		return -1;

	if (ioctl(port_fd, TIOCSERGETLSR, &res) < 0){
		return -1;
	}

#endif 
	return res;
}

// wait until data doesnt transmit
int Serial::waitTxEmpty()
{
	int res = -1;

	if (!port_is_open){
		return -1;
	}

#if defined(WIN32)
	// !TODO!
#elif defined(LINUX)

	if(port_fd <= 0)
		return -1;

	int lsr = 0;
 
	do
	{
		if ( ioctl(port_fd, TIOCSERGETLSR, &lsr) != 0 )
		{
			return -1;
		}
	}
	while ((lsr&TIOCSER_TEMT) == 0);
	
	return 0;

#endif 
	return res;
}

// wait for all transmitted data with Write to actually leave the serial port
void Serial::outputFlush(void)
{
	if (!port_is_open)
		return;

#if defined(WIN32)
	if(port_handle == INVALID_HANDLE_VALUE)
		return;

	FlushFileBuffers(port_handle);
#elif defined(LINUX)
	if(port_fd <= 0)
		return;

	// ждет, пока все данные вывода, записанные на объект, на который ссылается fd, не будут переданы. 
	tcdrain(port_fd);
#endif
}

// discard all received data that hasn't been read
void Serial::discardInput(void)
{
	if (!port_is_open)
		return;

#if defined(WIN32)
	if(port_handle == INVALID_HANDLE_VALUE)
		return;

	PurgeComm(port_handle, PURGE_RXCLEAR);
#elif defined(LINUX)
	// отказывается от данных, записанных, но не переданных на объект, на который ссылается fd, 
	// или принятых, но не считанных данных

	//	tcflush() discards data written to the object referred to by fd but not
	//      transmitted, or data received but not read, depending on the  value  of
	//      queue_selector:
	//
	//      TCIFLUSH
	//             flushes data received but not read.
	//
	//      TCOFLUSH
	//             flushes data written but not transmitted.
	//
	//      TCIOFLUSH
	//             flushes  both  data  received but not read, and data written but
	//             not transmitted.

	if(port_fd <= 0)
		return;

	tcflush(port_fd, TCIFLUSH); 
#endif
}

void Serial::discardOutput()
{
	if (!port_is_open)
		return;

#if defined(WIN32)
	if(port_handle == INVALID_HANDLE_VALUE)
		return;

	// Clears the output buffer (if the device driver has one).
	PurgeComm(port_handle, PURGE_TXCLEAR); 
#elif defined(LINUX)
	if(port_fd <= 0)
		return;

	tcflush(port_fd, TCOFLUSH); 
#endif
}

// set DTR and RTS,  0=low, 1=high, -1=unchanged
int Serial::setControl(int dtr, int rts)
{
	if (!port_is_open)
		return -1;

#if defined(WIN32)
	if(port_handle == INVALID_HANDLE_VALUE)
		return -1;

	// http://msdn.microsoft.com/en-us/library/aa363254(VS.85).aspx
	if (dtr == 1) 
	{
		if (!EscapeCommFunction(port_handle, SETDTR))
		{
			return -1;
		}
	}
	else if (dtr == 0)
	{
		if (!EscapeCommFunction(port_handle, CLRDTR))
		{
			return -1;
		}
	}
	if (rts == 1)
	{
		if (!EscapeCommFunction(port_handle, SETRTS))
		{
			return -1;
		}
	}
	else if (rts == 0)
	{
		if (!EscapeCommFunction(port_handle, CLRRTS))
		{
			return -1;
		}
	}
#elif defined(LINUX)

	// http://www.opennet.ru/docs/RUS/serial_guide/#advanced

	if(port_fd <= 0)
		return -1;

	int bits;
	if (ioctl(port_fd, TIOCMGET, &bits) < 0)
	{
		return -1;
	}
	if (dtr == 1)
	{
		bits |= TIOCM_DTR;
	}
	else if (dtr == 0)
	{
		bits &= ~TIOCM_DTR;
	}
	if (rts == 1)
	{
		bits |= TIOCM_RTS;
	}
	else if (rts == 0)
	{
		bits &= ~TIOCM_RTS;
	}
	if (ioctl(port_fd, TIOCMSET, &bits) < 0)
	{
		return -1;
	}

#endif
	return 0;
}
