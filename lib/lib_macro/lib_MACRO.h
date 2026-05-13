///-----------------------------------------------------------------------------
//                              AMFITECH APS
//                          ALL RIGHTS RESERVED
//-----------------------------------------------------------------------------

#ifndef LIB_MACRO_H_
#define LIB_MACRO_H_

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------
#ifdef USE_PROJECT_CONF
#include "project_conf.h"
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

#define sizeof_member(type, member) (sizeof(((type *)0)->member))

#if defined(__GNUC__)
#define container_of(ptr, type, member) (__extension__({     \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
        (type *)( (char *)__mptr - offsetof(type,member) ); }))
#else
#define container_of(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))
#endif // defined ( __GNUC__ )

// #define MASK32(m, n)  (((uint32_t)~0u >> (32 - n - 1)) & (~0u << (32 - m)))		// Create a bit mask ('1') from bit number m to n	(Doesn't work)

//-----------------------------------------------------------------------------
// Type declarations
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Variables and constants
//-----------------------------------------------------------------------------

#endif
