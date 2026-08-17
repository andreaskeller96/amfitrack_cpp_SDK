//-----------------------------------------------------------------------------
//
//                              AMFITECH APS
//
//                          ALL RIGHTS RESERVED
//
//-----------------------------------------------------------------------------

#pragma once

#ifdef __cplusplus

//-----------------------------------------------------------------------------
// Section: Includes
//-----------------------------------------------------------------------------
#include <array>
#include <cstddef>

#ifdef USE_THREAD_BASED
#include <mutex>
#endif

//-----------------------------------------------------------------------------
// Section: Define
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Typedef
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Macro
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Class
//-----------------------------------------------------------------------------

// Locked under USE_THREAD_BASED: two threads queue frames while a third drains,
// and an unlocked count++/count-- loses updates, silently dropping frames.
template <typename T, std::size_t Capacity>
class lib_fifo
{
  public:
	static_assert(Capacity > 0, "FIFO capacity must be greater than 0");

	bool put(const T &item);
	bool pop(T &item);
	bool peek(T &item) const;
	T *peek();
	const T *peek() const;

	void clear();

	bool isFull() const;
	bool isEmpty() const;

	std::size_t size() const;
	constexpr std::size_t capacity() const;
	std::size_t remaining() const;

  private:
	std::size_t increment(std::size_t index) const;

	// Unlocked forms, for use where the lock is already held.
	bool full() const { return count >= Capacity; }
	bool empty() const { return count == 0; }

	std::array<T, Capacity> buffer{};

	std::size_t head{ 0 };
	std::size_t tail{ 0 };
	std::size_t count{ 0 };

#ifdef USE_THREAD_BASED
	mutable std::mutex _mutex;
#endif
};

#ifdef USE_THREAD_BASED
#define LIB_FIFO_LOCK() const std::lock_guard<std::mutex> _lock(_mutex)
#else
#define LIB_FIFO_LOCK() ((void)0)
#endif

//-----------------------------------------------------------------------------
// Section: Template implementation
//-----------------------------------------------------------------------------

template <typename T, std::size_t Capacity>
bool lib_fifo<T, Capacity>::put(const T &item)
{
	LIB_FIFO_LOCK();

	if (full())
	{
		return false;
	}

	buffer[head] = item;
	head = increment(head);
	count++;

	return true;
}

template <typename T, std::size_t Capacity>
bool lib_fifo<T, Capacity>::pop(T &item)
{
	LIB_FIFO_LOCK();

	if (empty())
	{
		return false;
	}

	item = buffer[tail];
	tail = increment(tail);
	count--;

	return true;
}

template <typename T, std::size_t Capacity>
bool lib_fifo<T, Capacity>::peek(T &item) const
{
	LIB_FIFO_LOCK();

	if (empty())
	{
		return false;
	}

	item = buffer[tail];

	return true;
}

// Pointer outlives the lock; safe only for the popping thread, and not across its own pop().
template <typename T, std::size_t Capacity>
T *lib_fifo<T, Capacity>::peek()
{
	LIB_FIFO_LOCK();

	if (empty())
	{
		return nullptr;
	}

	return &buffer[tail];
}

template <typename T, std::size_t Capacity>
const T *lib_fifo<T, Capacity>::peek() const
{
	LIB_FIFO_LOCK();

	if (empty())
	{
		return nullptr;
	}

	return &buffer[tail];
}

template <typename T, std::size_t Capacity>
void lib_fifo<T, Capacity>::clear()
{
	LIB_FIFO_LOCK();

	head = 0;
	tail = 0;
	count = 0;
}

template <typename T, std::size_t Capacity>
bool lib_fifo<T, Capacity>::isFull() const
{
	LIB_FIFO_LOCK();

	return full();
}

template <typename T, std::size_t Capacity>
bool lib_fifo<T, Capacity>::isEmpty() const
{
	LIB_FIFO_LOCK();

	return empty();
}

template <typename T, std::size_t Capacity>
std::size_t lib_fifo<T, Capacity>::size() const
{
	LIB_FIFO_LOCK();

	return count;
}

template <typename T, std::size_t Capacity>
constexpr std::size_t lib_fifo<T, Capacity>::capacity() const
{
	return Capacity;
}

template <typename T, std::size_t Capacity>
std::size_t lib_fifo<T, Capacity>::remaining() const
{
	LIB_FIFO_LOCK();

	return Capacity - count;
}

template <typename T, std::size_t Capacity>
std::size_t lib_fifo<T, Capacity>::increment(std::size_t index) const
{
	index++;

	if (index >= Capacity)
	{
		index = 0;
	}

	return index;
}

#endif

/** @} */ // end of module