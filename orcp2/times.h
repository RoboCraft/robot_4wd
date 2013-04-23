//
// OpenRoboVision
//
// time functions
//
// robocraft.ru
//

#ifndef _ORV_TIMES_H_
#define _ORV_TIMES_H_

#include "types.h"

#include <time.h>

#if defined(WIN32)

#elif defined(LINUX)
# include <unistd.h>
# include <sys/time.h>
#endif

namespace orv
{
	namespace time
	{

	// for store time
	typedef struct Time
	{
		WORD Year;
		WORD Month;
		WORD DayOfWeek;
		WORD Day;
		WORD Hour;
		WORD Minute;
		WORD Second;
		WORD Milliseconds;
	} Time;

	// get time
	void get_current(orv::time::Time* time);

	void sleep(DWORD dwMilliseconds);

	DWORD millis();

	}; // namespace time
}; //namespace orv

#endif //#ifndef _ORV_TIMES_H_
