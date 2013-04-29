//
// console functions
//
// robocraft.ru
//

#include "console.h"

#if defined(WIN32)

#elif defined(LINUX)

//
// Linux (POSIX) implementation of _kbhit().
// Morgan McGuire, morgan@cs.brown.edu
//
// http://www.flipcode.com/archives/_kbhit_for_Linux.shtml
//
int _kbhit() 
{
	static const int STDIN = 0;
	static bool initialized = false;

	if (! initialized)
	{
		// Use termios to turn off line buffering
		termios term;
		tcgetattr(STDIN, &term);
		term.c_lflag &= ~ICANON;
		tcsetattr(STDIN, TCSANOW, &term);
		setbuf(stdin, NULL);
		initialized = true;
	}

	int bytesWaiting;
	ioctl(STDIN, FIONREAD, &bytesWaiting);
	return bytesWaiting;
}

//
// Linux equivalent of getch() (from conio)
//
// http://cboard.cprogramming.com/faq-board/27714-faq-there-getch-conio-equivalent-linux-unix.html
//
int _getch()
{
	struct termios oldt, newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}

#endif //#if defined(WIN32)

// wait key
int console::waitKey(int msec)
{
	int res = 0;

#if defined(WIN32)

	// get handle to stdin used by console functions
	HANDLE hStdin =  GetStdHandle(STD_INPUT_HANDLE);

	// disable mouse events
	DWORD dwMode;
	GetConsoleMode(hStdin, &dwMode);
	SetConsoleMode(hStdin, dwMode & ~ENABLE_MOUSE_INPUT);
	FlushConsoleInputBuffer(hStdin);

	const int MaxEvents = 16;
	DWORD dwNumRead = 0;
	INPUT_RECORD buf[MaxEvents];

	// wait event
	if( WaitForSingleObject(hStdin, msec) == WAIT_OBJECT_0 ){
		// got event - reading code
		ReadConsoleInput(hStdin, buf, MaxEvents, &dwNumRead);
		// process each event
		for(DWORD i = 0; i < dwNumRead; i++){
			// if keyboard event
			if( buf[i].EventType == KEY_EVENT){
				KEY_EVENT_RECORD* pKey = (KEY_EVENT_RECORD*)&buf[i].Event.KeyEvent;
				// if key down event
				if( pKey->bKeyDown == TRUE){
					res = pKey->wVirtualKeyCode;
				}
			}
		}
	}

#elif defined(LINUX)
	fd_set rfds;
	struct timeval tv;
	int retval;

	int nfds = STDIN_FILENO+1;

	FD_ZERO(&rfds);
	FD_SET(STDIN_FILENO, &rfds);

	tv.tv_sec = msec / 1000;
	tv.tv_usec = (msec % 1000) * 1000;

	if( tv.tv_usec > 1000000 )
	{
		tv.tv_sec++;
		tv.tv_usec -= 1000000;
	}

	retval = select(nfds, &rfds, NULL, NULL, msec!=0 ? &tv : 0);
	if (retval>0)
	{
		if(FD_ISSET(STDIN_FILENO, &rfds))
		{
			retval = read(STDIN_FILENO, &res, 1);
			if(retval<1)
				res = 0;
		}
	}
#endif //#if defined(WIN32)
	return res;
}
