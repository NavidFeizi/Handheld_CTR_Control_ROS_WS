#ifndef SHAREDSTATE_HPP
#define SHAREDSTATE_HPP

#include <future>
#include <atomic>
#include <mutex>
#include <iostream>
#include <lely/ev/future.hpp>

class SharedState
{
public:
    // std::promise<void> bootSuccessPromise;
    // std::future<void> bootSuccessFuture;

    lely::ev::Promise<void> bootPromise;
    lely::ev::Future<void> bootFuture = bootPromise.get_future();

    // Atomic flag for thread-safe operations
    std::atomic<bool> m_flag_operation_enabled{false};
    std::atomic<bool> m_flag_operation_enabled_2{false};
    std::atomic<bool> m_boot_success{false};
    std::atomic<bool> m_flag_robot_switched_on{false};
    std::atomic<bool> m_encoders_set{false};

    // Constructor to initialize the futures
    SharedState()
       : bootFuture(bootPromise.get_future()) {}

    // Method to signal that boot is successful
    void signalBootSuccess()
    {
        bootPromise.set(std::error_code{});
    }

private:
};

#endif // SHAREDSTATE_HPP