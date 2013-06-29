//
//
// types
//
// robocraft.ru
//

#ifndef _TYPES_H_
#define _TYPES_H_

#if defined(WIN32)
# define WIN32_LEAN_AND_MEAN
# include <windows.h>

typedef unsigned __int64 uint64_t;

// Linux types
typedef char __s8;
typedef unsigned char __u8;

typedef short __s16;
typedef unsigned short __u16;

typedef int __s32;
typedef unsigned int __u32;

typedef long __s64;
typedef unsigned long __u64;

typedef signed char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long s64;
typedef unsigned long u64;

// bsd
typedef unsigned char           u_char;
typedef unsigned short          u_short;
typedef unsigned int            u_int;
typedef unsigned long           u_long;
// sysv
typedef unsigned char           unchar;
typedef unsigned short          ushort;
typedef unsigned int            uint;
typedef unsigned long           ulong;

typedef			__s16 			int16_t;
typedef			__s32 			int32_t;

typedef         __u8            uint8_t;
typedef         __u16           uint16_t;
typedef         __u32           uint32_t;

typedef         __u8            u_int8_t;
typedef         __u16            u_int16_t;
typedef         __u32            u_int32_t;

typedef char *          __kernel_caddr_t;
#ifndef _CADDR_T
#define _CADDR_T
typedef __kernel_caddr_t        caddr_t;
#endif

typedef int             __kernel_pid_t;
typedef __kernel_pid_t          pid_t;

typedef int socklen_t;

#elif defined(LINUX)
# include <stdint.h>

typedef unsigned long       DWORD;
typedef int                 BOOL;
typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef float               FLOAT;
typedef int                 INT;
typedef unsigned int        UINT;

typedef signed char         INT8, *PINT8;
typedef signed short        INT16, *PINT16;
typedef signed int          INT32, *PINT32;
typedef unsigned char       UINT8, *PUINT8;
typedef unsigned short      UINT16, *PUINT16;
typedef unsigned int        UINT32, *PUINT32;

typedef void				*LPVOID;

typedef char TCHAR, *PTCHAR;
typedef unsigned char TBYTE , *PTBYTE ;

typedef int HANDLE;

typedef const char*			LPCTSTR;

#elif defined(ARDUINO)

# if ARDUINO >= 100
#  include "Arduino.h"
# else
#  include "WProgram.h"
# endif

# include <inttypes.h>
# include <stdio.h> // for size_t

#endif //#if defined(WIN32)

#endif //#ifndef _TYPES_H_
