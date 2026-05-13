//-----------------------------------------------------------------------------
//                              AMFITECH APS
//                          ALL RIGHTS RESERVED
//
// $URL: $
// $Rev: $
// $Date: $
// $Author: $
//
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
/// \cond
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
/// \endcond

#include "lib_Queue.h"

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Type declarations
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Function prototypes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Variables and constants
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------
void libQueue_Init(libQueue_Pointer_t *pointer, size_t buffer_length)
{
	pointer->head = pointer->tail = 0;
	pointer->length = buffer_length;
}

bool libQueue_Full(libQueue_Pointer_t *pointer)
{
	assert(pointer->length > 0);

	bool retVal = false;

	if ((pointer->head + 1) % pointer->length == pointer->tail)
	{
		retVal = true;
	}

	return retVal;
}

bool libQueue_Empty(libQueue_Pointer_t *pointer)
{
	bool retVal = false;

	if (pointer->tail == pointer->head)
	{
		retVal = true;
	}

	return retVal;
}

size_t libQueue_Read(libQueue_Pointer_t *pointer)
{
	assert(!libQueue_Empty(pointer));

	return pointer->tail;
}

size_t libQueue_Write(libQueue_Pointer_t *pointer)
{
	assert(!libQueue_Full(pointer));

	return pointer->head;
}

void libQueue_Add(libQueue_Pointer_t *pointer)
{
	assert(pointer->length > 0);

	assert(!libQueue_Full(pointer));

	pointer->head = (pointer->head + 1) % pointer->length;
}

void libQueue_Remove(libQueue_Pointer_t *pointer)
{
	assert(pointer->length > 0);

	assert(!libQueue_Empty(pointer));

	pointer->tail = (pointer->tail + 1) % pointer->length;
}

void libQueue_Clear(libQueue_Pointer_t *pointer)
{
	pointer->tail = pointer->head;
}

size_t libQueue_Remaining(libQueue_Pointer_t *pointer)
{
	return libQueue_MaxSize(pointer) - libQueue_Size(pointer);
}

size_t libQueue_Size(libQueue_Pointer_t *pointer)
{
	return (pointer->head - pointer->tail + pointer->length) % pointer->length;
}

size_t libQueue_MaxSize(libQueue_Pointer_t *pointer)
{
	assert(pointer->length > 0);

	return pointer->length - 1;
}
