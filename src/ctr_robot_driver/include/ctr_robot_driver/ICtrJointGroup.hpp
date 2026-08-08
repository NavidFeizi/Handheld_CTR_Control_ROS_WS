#pragma once

// Hardware seam for the 4-joint CTR actuation group. RobotNode programs
// against this interface; CTRobot is the CANopen implementation. A test or
// simulation double implements the same surface without hardware.

#include <atomic>
#include <bitset>
#include <cstdint>

#include <blaze/Math.h>

#include "ctr_robot_driver/CanEssentials.hpp"

class ICtrJointGroup
{
public:
    virtual ~ICtrJointGroup() = default;

    // ============================== Lifecycle ==============================
    /// Start communication with the joint drives. Blocking; returns false if
    /// the hardware never comes up (after bounded retries).
    virtual bool connect(int sample_time_ms) = 0;
    /// Orderly teardown; safe to call more than once.
    virtual void shutdown() = 0;
    virtual bool isConnected() const = 0;

    // ============================== Command ==============================
    virtual void enableOperation(bool enable) = 0;
    virtual void setTargetPos(const blaze::StaticVector<double, 4UL> &target) = 0;
    virtual void setTargetVel(const blaze::StaticVector<double, 4UL> &target) = 0;

    // ============================== Configuration ==============================
    virtual void setMaxVel(const blaze::StaticVector<double, 4UL> &maxVel) = 0;
    virtual void setMaxAcc(const blaze::StaticVector<double, 4UL> &maxAcc) = 0;
    virtual void setMaxTorque(blaze::StaticVector<double, 4UL> negative,
                              blaze::StaticVector<double, 4UL> positive) = 0;
    virtual void setProfileParams(blaze::StaticVector<double, 4UL> max_vel,
                                  blaze::StaticVector<double, 4UL> max_acc,
                                  blaze::StaticVector<double, 4UL> max_dcc) = 0;
    virtual void setOperationMode(OpMode mode) = 0;
    virtual void setEncoders(blaze::StaticVector<double, 4> val) = 0;
    virtual void setPosLimit(const blaze::StaticVector<double, 4> &min,
                             const blaze::StaticVector<double, 4> &max) const = 0;

    // ============================== Feedback ==============================
    virtual blaze::StaticVector<double, 4> getCurrent() const = 0;
    virtual void getVel(blaze::StaticVector<double, 4UL> &val) const = 0;
    virtual void getPos(blaze::StaticVector<double, 4UL> &val) const = 0;
    virtual void getPosLimit(blaze::StaticVector<double, 4> &min,
                             blaze::StaticVector<double, 4> &max) const = 0;

    // ============================== Status ==============================
    virtual bool getSwitchStatus(blaze::StaticVector<bool, 4> &status) const = 0;
    virtual bool getSwitchStatus() const = 0;
    virtual blaze::StaticVector<bool, 4> getEnableStatus() const = 0;
    virtual blaze::StaticVector<bool, 4> getEncoderStatus() const = 0;
    virtual bool getDisabledStatus(blaze::StaticVector<bool, 4> &status) const = 0;
    virtual bool getDisabledStatus() const = 0;
    virtual blaze::StaticVector<bool, 4> getReachedStatus() const = 0;
    virtual void getTemperature(blaze::StaticVector<int32_t, 4> &cpu,
                                blaze::StaticVector<int32_t, 4> &driver) const = 0;
    virtual void getDigitalIn(blaze::StaticVector<std::bitset<32>, 4> &in) const = 0;
    virtual void getInterface() const = 0;

    // ============================== Waits ==============================
    virtual void waitUntilReach(const std::atomic<bool> &cancel_flag) const = 0;
    virtual void waitUntilReach() const = 0;
    virtual void waitUntilTransReach(const std::atomic<bool> &cancel_flag) const = 0;
    virtual void waitUntilTransReach() const = 0;
};
