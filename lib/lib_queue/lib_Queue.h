//-----------------------------------------------------------------------------
//                              AMFITECH APS
//                          ALL RIGHTS RESERVED
//
// $URL: $
// $Rev: $
// $Date: $
// $Author: $
//
// Description
// TODO Write a description here
//
//-----------------------------------------------------------------------------

#ifndef LIB_QUEUE_H_
#define LIB_QUEUE_H_

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#ifdef USE_PROJECT_CONF
#include "project_conf.h"
#endif

/// \cond
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
/// \endcond

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Type declarations
//-----------------------------------------------------------------------------
typedef struct libQueue_Pointer libQueue_Pointer_t;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------
void libQueue_Init(libQueue_Pointer_t *pointer, size_t buffer_length);

bool libQueue_Full(libQueue_Pointer_t *pointer);
bool libQueue_Empty(libQueue_Pointer_t *pointer);

// Call read before calling remove
size_t libQueue_Read(libQueue_Pointer_t *pointer);
void libQueue_Remove(libQueue_Pointer_t *pointer);

// Call write before calling add
size_t libQueue_Write(libQueue_Pointer_t *pointer);
void libQueue_Add(libQueue_Pointer_t *pointer);

void libQueue_Clear(libQueue_Pointer_t *pointer);

size_t libQueue_Remaining(libQueue_Pointer_t *pointer);
size_t libQueue_Size(libQueue_Pointer_t *pointer);
size_t libQueue_MaxSize(libQueue_Pointer_t *pointer);

//-----------------------------------------------------------------------------
// Variables and constants
//-----------------------------------------------------------------------------
struct libQueue_Pointer
{
	size_t head;
	size_t tail;
	size_t length;
};

#endif
