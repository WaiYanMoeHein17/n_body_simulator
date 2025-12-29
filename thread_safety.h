#ifndef THREAD_SAFETY_H
#define THREAD_SAFETY_H

#include <mutex>
#include <type_traits>

struct no_lock {
	struct guard {guard(no_lock&) {} };
};

struct mutex_lock {
	std::mutex m_lock;
	struct guard {
		std::unique_lock<std::mutex> lock; 
		guard(mutex_lock& m_lock) : lock(m_lock.m_lock) {}
	};
};

template<bool thread_safe> 
using lock_policy = std::conditional_t<thread_safe, mutex_lock, no_lock>; 

template<bool thread_safe> 
class thread_decision : private lock_policy<thread_safe> {

private: 
	
	using base = lock_policy<thread_safe>; 
	long value = 0;

public: 

	void inc() {
		typename base::guard g(*this); 
	}

	long get_value() const {
		return value; 
	}
};

#endif 