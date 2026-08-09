#ifndef NODE_COMMAND_HPP
#define NODE_COMMAND_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

#include "ctr_robot_driver/CanEssentials.hpp" // OpMode

// Deliberately free of lely, blaze and ROS headers so the handshake can be
// unit-tested without hardware (see test/test_node_command.cpp).

/* Configuration commands handed from the caller threads to a node's lely
   fiber. The fiber may only touch the CANopen stack from inside its own
   coroutine, so every configuration call from a ROS thread has to cross this
   boundary. */
enum class NodeCommand : uint8_t
{
  None = 0,
  Enable,
  Disable,
  SetMaxTorque,
  SetProfileParams,
  SetEncoder,
  SetOperationMode,
};

/* Single-slot mailbox: caller threads produce, the node's lely fiber consumes.

   The payload MUST become visible to the consumer whenever the command tag
   does. Getting that wrong is what this class exists to prevent. The previous
   handshake was a plain `int m_isConfiguring` set to true *before* a plain
   `std::string m_commandMsg` was assigned, with no synchronisation at all:
   the fiber could observe the flag while the string was still stale or
   half-written, fall through every dispatch branch, and then clear both —
   silently swallowing the command for that one node. With four nodes each
   racing independently, the visible result was exactly one motor left
   un-enabled while the other three came up.

   Ordering contract:
     producer -- wait for the slot to drain, write payload, store(release) tag
     consumer -- load(acquire) tag, read payload, store(release) None

   The release/acquire pair on the tag publishes the payload writes to the
   consumer. The producer's wait for None (itself an acquire load, paired with
   the consumer's release store) guarantees the consumer has finished reading
   the previous payload before the producer overwrites it, so the payload
   itself is never concurrently accessed. */
class NodeCommandMailbox
{
public:
  struct Payload
  {
    bool enable = false;                        // Enable / Disable
    double maxTorqueNegative = 0.0;             // SetMaxTorque
    double maxTorquePositive = 0.0;             // SetMaxTorque
    int profileAcc = 0;                         // SetProfileParams, controller units
    int profileDcc = 0;                         // SetProfileParams, controller units
    int profileVel = 0;                         // SetProfileParams, controller units
    double encoderOffset = 0.0;                 // SetEncoder, SI
    OpMode operationMode = OpMode::Disabled;    // SetOperationMode
  };

  /* True while a command is waiting to be picked up by the fiber. */
  bool pending() const
  {
    return m_command.load(std::memory_order_acquire) != NodeCommand::None;
  }

  /* Publish `payload` under tag `command`.

     Waits up to `timeout` for a previously published command to drain, so a
     rapid second call cannot overwrite one the fiber has not consumed yet.
     Returns false (and publishes NOTHING) if the slot is still occupied when
     the timeout expires — publishing anyway would reintroduce the very data
     race this class removes, and a slot that has not drained in `timeout`
     means the fiber is wedged, where an overwrite would not have helped
     either. Callers must log a false return: a dropped command is acceptable,
     a silently dropped one is not.

     The default timeout is sized to clear the slowest handler, not the poll
     period: SetEncoder_ alone sits on ~2.5 s of internal AsyncWaits, so a
     caller that follows setEncoder() with enableOperation() legitimately has
     to wait. (That pair is exactly what findLinearHome does, and under the old
     handshake the trailing disable was silently wiped by the still-running
     handler.) */
  bool publish(NodeCommand command,
               const Payload &payload,
               std::chrono::milliseconds timeout = std::chrono::milliseconds(5000))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (m_command.load(std::memory_order_acquire) != NodeCommand::None)
    {
      if (std::chrono::steady_clock::now() >= deadline)
        return false;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    m_payload = payload;                                  // slot is ours: no reader
    m_command.store(command, std::memory_order_release);  // publishes the payload
    return true;
  }

  /* Take the pending command, or NodeCommand::None if the slot is empty. On a
     non-None return `payload` holds the values published alongside the tag.
     The payload is copied out before the slot is released, so the fiber may
     take as long as it likes to act on it. */
  NodeCommand consume(Payload &payload)
  {
    const NodeCommand command = m_command.load(std::memory_order_acquire);
    if (command == NodeCommand::None)
      return NodeCommand::None;

    payload = m_payload;                                          // published by the acquire above
    m_command.store(NodeCommand::None, std::memory_order_release); // releases the slot
    return command;
  }

private:
  std::atomic<NodeCommand> m_command{NodeCommand::None};
  Payload m_payload{};
};

#endif // NODE_COMMAND_HPP
