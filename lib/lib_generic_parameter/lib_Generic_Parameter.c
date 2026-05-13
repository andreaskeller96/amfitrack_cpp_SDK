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
#include <assert.h>
#include <string.h> // For memcpy
/// \endcond

#include "lib_Generic_Parameter.h"

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

bool lib_Generic_Parameter_ValueIsEqual(lib_Generic_Parameter_Value_t v1, lib_Generic_Parameter_Value_t v2)
{
	bool isEqual = false;

	if (v1.type == v2.type)
	{
		switch (v1.type)
		{
			case lib_Generic_Parameter_Type_bool:
				if (v1.b == v2.b)
				{
					isEqual = true;
				}
				break;
			case lib_Generic_Parameter_Type_char:
				if (v1.ch == v2.ch)
				{
					isEqual = true;
				}
				break;
			case lib_Generic_Parameter_Type_s8:
				if (v1.s8 == v2.s8)
				{
					isEqual = true;
				}
				break;
			case lib_Generic_Parameter_Type_u8:
				if (v1.u8 == v2.u8)
				{
					isEqual = true;
				}
				break;
			case lib_Generic_Parameter_Type_s16LE:
				if (v1.s16 == v2.s16)
				{
					isEqual = true;
				}
				break;
			case lib_Generic_Parameter_Type_u16LE:
				if (v1.u16 == v2.u16)
				{
					isEqual = true;
				}
				break;
			case lib_Generic_Parameter_Type_s32LE:
				if (v1.s32 == v2.s32)
				{
					isEqual = true;
				}
				break;
			case lib_Generic_Parameter_Type_u32LE:
				if (v1.u32 == v2.u32)
				{
					isEqual = true;
				}
				break;
			case lib_Generic_Parameter_Type_f32LE:
				if (v1.f32 == v2.f32)
				{
					isEqual = true;
				}
				break;
			case lib_Generic_Parameter_Type_f64LE:
				if (v1.f64 == v2.f64)
				{
					isEqual = true;
				}
				break;
			case lib_Generic_Parameter_Type_ProcedureCall:
				if (v1.b == v2.b)
				{
					isEqual = true;
				}
				break;
			case lib_Generic_Parameter_Type_void:
			{
				isEqual = true; // Void types are always equal
			}
			break;
			default:
				break;
		}
	}
	return isEqual;
}

uint8_t lib_Generic_Parameter_SizeWithType(lib_Generic_Parameter_Value_t value)
{
	uint8_t length = sizeof(value.type);

	switch (value.type)
	{
		case lib_Generic_Parameter_Type_void:
			length += sizeof(value.b); // For legacy compatibility
			// No storage allocated to void types
			break;
		case lib_Generic_Parameter_Type_bool:
			length += sizeof(value.b);
			break;
		case lib_Generic_Parameter_Type_char:
			length += sizeof(value.ch);
			break;
		case lib_Generic_Parameter_Type_s8:
			length += sizeof(value.s8);
			break;
		case lib_Generic_Parameter_Type_u8:
			length += sizeof(value.u8);
			break;
		case lib_Generic_Parameter_Type_s16LE:
			length += sizeof(value.s16);
			break;
		case lib_Generic_Parameter_Type_u16LE:
			length += sizeof(value.u16);
			break;
		case lib_Generic_Parameter_Type_s32LE:
			length += sizeof(value.s32);
			break;
		case lib_Generic_Parameter_Type_u32LE:
			length += sizeof(value.u32);
			break;
		case lib_Generic_Parameter_Type_f32LE:
			length += sizeof(value.f32);
			break;
		case lib_Generic_Parameter_Type_f64LE:
			length += sizeof(value.f64);
			break;
		case lib_Generic_Parameter_Type_ProcedureCall:
			length += sizeof(value.b); // Also uses bool as storage
			break;
		default:
			break;
	}

	return length;
}

uint8_t lib_Generic_Parameter_SerializeValueAndType(lib_Generic_Parameter_Value_t value, void *pDest, uint8_t maxLength)
{
	assert(pDest);

	char *dest = (char *)pDest;

	uint8_t length = lib_Generic_Parameter_SizeWithType(value);

	if (length <= maxLength)
	{
		memcpy(dest, &(value.type), sizeof(value.type));
		dest += sizeof(value.type);
		length -= sizeof(value.type);

		switch (value.type)
		{
			case lib_Generic_Parameter_Type_bool:
				memcpy(dest, &(value.b), length);
				break;
			case lib_Generic_Parameter_Type_char:
				memcpy(dest, &(value.ch), length);
				break;
			case lib_Generic_Parameter_Type_s8:
				memcpy(dest, &(value.s8), length);
				break;
			case lib_Generic_Parameter_Type_u8:
				memcpy(dest, &(value.u8), length);
				break;
			case lib_Generic_Parameter_Type_s16LE:
				memcpy(dest, &(value.s16), length);
				break;
			case lib_Generic_Parameter_Type_u16LE:
				memcpy(dest, &(value.u16), length);
				break;
			case lib_Generic_Parameter_Type_s32LE:
				memcpy(dest, &(value.s32), length);
				break;
			case lib_Generic_Parameter_Type_u32LE:
				memcpy(dest, &(value.u32), length);
				break;
			case lib_Generic_Parameter_Type_f32LE:
				memcpy(dest, &(value.f32), length);
				break;
			case lib_Generic_Parameter_Type_f64LE:
				memcpy(dest, &(value.f64), length);
				break;
			case lib_Generic_Parameter_Type_ProcedureCall:
				memcpy(dest, &(value.b), length);
				break;
			case lib_Generic_Parameter_Type_void:
				// Nothing to do for type Void
				break;
			default:
				break;
		}
	}
	else
	{
		length = 0;
	}

	return length;
}
