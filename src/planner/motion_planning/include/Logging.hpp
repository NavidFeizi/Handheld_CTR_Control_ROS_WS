#pragma once

#include <iostream>
#include <mutex>

namespace logging
{
inline bool verbose = false;
inline std::mutex &streamMutex()
{
	static std::mutex mtx;
	return mtx;
}

template <typename... Args>
inline void debug(Args &&...args)
{
	if (!verbose)
		return;
	std::lock_guard<std::mutex> lock(streamMutex());
	std::cout << "[DEBUG] ";
	((std::cout << std::forward<Args>(args)), ...);
	std::cout << std::endl;
}
}
