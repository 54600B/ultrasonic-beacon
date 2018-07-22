// coding: utf-8
/* Copyright (c) 2009, Roboterclub Aachen e.V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// ----------------------------------------------------------------------------

#ifndef	XPCC_TIMESTAMP_HPP
#define	XPCC_TIMESTAMP_HPP

#include <stdint.h>
#include <type_traits>

namespace xpcc
{

/**
 * Generic timestamp for 16bit and 32bit timestamps of variable timebase.
 *
 * @author	Fabian Greif
 * @author	Niklas Hauser
 * @ingroup	software_timer
 */
template< typename T >
class GenericTimestamp
{
public:
	typedef T Type;
	using SignedType = std::make_signed_t<T>;

public:
	/// @param time in ms
	GenericTimestamp(const Type time = 0) :
		time(time)
	{
	}

	GenericTimestamp(const GenericTimestamp<T> &other) :
		time(other.time)
	{
	}

	inline Type
	getTime() const
	{
		return time;
	}

	inline GenericTimestamp<T>
	operator + (const GenericTimestamp<T>& other) const
	{
		return GenericTimestamp<T>(time + other.time);
	}

	inline GenericTimestamp<T>
	operator - (const GenericTimestamp<T>& other) const
	{
		return GenericTimestamp<T>(time - other.time);
	}

	inline bool
	operator == (const GenericTimestamp<T>& other) const
	{
		return (time == other.time);
	}

	inline bool
	operator != (const GenericTimestamp<T>& other) const
	{
		return (time != other.time);
	}

	inline bool
	operator < (const GenericTimestamp<T>& other) const
	{
		return SignedType(time - other.time) < 0;
	}

	inline bool
	operator > (const GenericTimestamp<T>& other) const
	{
		return SignedType(time - other.time) > 0;
	}

	inline bool
	operator <= (const GenericTimestamp<T>& other) const
	{
		return SignedType(time - other.time) <= 0;
	}

	inline bool
	operator >= (const GenericTimestamp<T>& other) const
	{
		return SignedType(time - other.time) >= 0;
	}

private:
	Type time;
};

/// 16bit timestamp, which can hold up to 65 seconds at millisecond resolution.
/// @ingroup	software_timer
using ShortTimestamp = GenericTimestamp<uint16_t>;

/// 32bit timestamp, which can hold up to 49 days at millisecond resolution.
/// @ingroup	software_timer
using Timestamp      = GenericTimestamp<uint32_t>;

}	// namespace xpcc

#endif	// XPCC_TIMESTAMP_HPP
