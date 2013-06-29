//
// OpenRoboVision
//
// time functions
//
// robocraft.ru
//

#include "times.h"

// get time
void orv::time::get_current(orv::time::Time* t)
{
	if(!t)
		return;

#if defined(WIN32)

	SYSTEMTIME SysTime;
	// get local time
	GetLocalTime(&SysTime);

	t->Year = SysTime.wYear;
	t->Month = SysTime.wMonth;
	t->DayOfWeek = SysTime.wDayOfWeek;
	t->Day = SysTime.wDay;
	t->Hour = SysTime.wHour;
	t->Minute = SysTime.wMinute;
	t->Second = SysTime.wSecond;
	t->Milliseconds = SysTime.wMilliseconds;

#elif defined(LINUX)

	struct tm *newtime;
	time_t long_time;

	struct timespec tm;
	clock_gettime(CLOCK_REALTIME, &tm);
	long_time = tm.tv_sec;

	newtime=localtime(&long_time);
	if(newtime)
	{
		t->Year = 1900 + newtime->tm_year;
		t->Month = newtime->tm_mon + 1;
		t->DayOfWeek = newtime->tm_wday;
		t->Day = newtime->tm_mday;
		t->Hour = newtime->tm_hour;
		t->Minute = newtime->tm_min;
		t->Second = newtime->tm_sec;
		t->Milliseconds = tm.tv_nsec/1000000;
	}

#endif //#if defined(WIN32)
}

void orv::time::sleep(DWORD dwMilliseconds)
{
#if defined(WIN32)
	Sleep(dwMilliseconds);
#elif defined(LINUX)
	usleep(dwMilliseconds*1000);
#endif //#if defined(WIN32)
}

DWORD orv::time::millis()
{
#if defined(WIN32)
	return GetTickCount();
#elif defined(LINUX)
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec*1000+tv.tv_usec/1000);
#endif //#if defined(WIN32)
}
