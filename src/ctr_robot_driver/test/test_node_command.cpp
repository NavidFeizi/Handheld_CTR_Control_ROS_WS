// Hardware-free tests for the caller-thread -> lely-fiber command handshake.
// The bug these guard against: the old handshake published a bare flag before
// the command payload, so the consumer could see "a command is pending" while
// the payload was still stale, consume it, discard it, and leave that one
// joint un-enabled while its three siblings came up.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "ctr_robot_driver/node_command.hpp"

TEST(NodeCommandMailbox, StartsEmpty)
{
    NodeCommandMailbox mailbox;
    NodeCommandMailbox::Payload payload;

    EXPECT_FALSE(mailbox.pending());
    EXPECT_EQ(mailbox.consume(payload), NodeCommand::None);
}

TEST(NodeCommandMailbox, PublishThenConsumeCarriesPayload)
{
    NodeCommandMailbox mailbox;

    NodeCommandMailbox::Payload out;
    out.profileAcc = 10000;
    out.profileDcc = 9000;
    out.profileVel = 1200;
    ASSERT_TRUE(mailbox.publish(NodeCommand::SetProfileParams, out));
    EXPECT_TRUE(mailbox.pending());

    NodeCommandMailbox::Payload in;
    EXPECT_EQ(mailbox.consume(in), NodeCommand::SetProfileParams);
    EXPECT_EQ(in.profileAcc, 10000);
    EXPECT_EQ(in.profileDcc, 9000);
    EXPECT_EQ(in.profileVel, 1200);
}

TEST(NodeCommandMailbox, ConsumeReleasesTheSlot)
{
    NodeCommandMailbox mailbox;
    NodeCommandMailbox::Payload payload;

    ASSERT_TRUE(mailbox.publish(NodeCommand::Enable, payload));
    ASSERT_EQ(mailbox.consume(payload), NodeCommand::Enable);

    EXPECT_FALSE(mailbox.pending());
    EXPECT_EQ(mailbox.consume(payload), NodeCommand::None);
}

TEST(NodeCommandMailbox, EnableAndDisableAreDistinguishable)
{
    NodeCommandMailbox mailbox;
    NodeCommandMailbox::Payload payload;

    payload.enable = true;
    ASSERT_TRUE(mailbox.publish(NodeCommand::Enable, payload));
    NodeCommandMailbox::Payload got;
    EXPECT_EQ(mailbox.consume(got), NodeCommand::Enable);
    EXPECT_TRUE(got.enable);

    payload.enable = false;
    ASSERT_TRUE(mailbox.publish(NodeCommand::Disable, payload));
    EXPECT_EQ(mailbox.consume(got), NodeCommand::Disable);
    EXPECT_FALSE(got.enable);
}

TEST(NodeCommandMailbox, PublishRefusesToOverwriteAnUnconsumedCommand)
{
    NodeCommandMailbox mailbox;
    NodeCommandMailbox::Payload first;
    first.encoderOffset = -0.167;
    ASSERT_TRUE(mailbox.publish(NodeCommand::SetEncoder, first));

    // Nothing consumed it, so the second publish must fail rather than
    // clobber the pending one — the caller logs and the operator finds out.
    NodeCommandMailbox::Payload second;
    second.encoderOffset = 99.0;
    // Short explicit timeout so the test does not sit on the 5 s default.
    EXPECT_FALSE(mailbox.publish(NodeCommand::SetEncoder, second, std::chrono::milliseconds(20)));

    NodeCommandMailbox::Payload got;
    EXPECT_EQ(mailbox.consume(got), NodeCommand::SetEncoder);
    EXPECT_DOUBLE_EQ(got.encoderOffset, -0.167);
}

TEST(NodeCommandMailbox, PublishSucceedsOnceTheConsumerDrainsTheSlot)
{
    NodeCommandMailbox mailbox;
    NodeCommandMailbox::Payload payload;
    ASSERT_TRUE(mailbox.publish(NodeCommand::Enable, payload));

    std::thread consumer([&mailbox]()
                         {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        NodeCommandMailbox::Payload got;
        mailbox.consume(got); });

    NodeCommandMailbox::Payload second;
    second.operationMode = OpMode::VelocityProfile;
    EXPECT_TRUE(mailbox.publish(NodeCommand::SetOperationMode, second, std::chrono::milliseconds(2000)));
    consumer.join();

    NodeCommandMailbox::Payload got;
    EXPECT_EQ(mailbox.consume(got), NodeCommand::SetOperationMode);
    EXPECT_EQ(got.operationMode, OpMode::VelocityProfile);
}

// The regression test that matters: hammer the handshake the way
// CTRobot::enableOperation does (producer thread) against a polling fiber
// (consumer thread). Every command that the producer successfully publishes
// must be delivered with the payload it was published with — never a stale
// one, never silently dropped.
TEST(NodeCommandMailbox, NoCommandIsDeliveredWithAStalePayload)
{
    NodeCommandMailbox mailbox;
    std::atomic<bool> stop{false};
    std::atomic<int> delivered{0};
    std::atomic<int> mismatched{0};

    std::thread consumer([&]()
                         {
        NodeCommandMailbox::Payload got;
        while (!stop.load(std::memory_order_acquire))
        {
            if (mailbox.consume(got) != NodeCommand::None)
            {
                // encoderOffset was published as the command's own sequence
                // number; anything else means we read a payload that did not
                // belong to the tag we just took.
                if (got.encoderOffset != static_cast<double>(got.profileAcc))
                    mismatched.fetch_add(1, std::memory_order_relaxed);
                delivered.fetch_add(1, std::memory_order_relaxed);
            }
            std::this_thread::yield();
        } });

    constexpr int kCommands = 2000;
    int published = 0;
    for (int i = 0; i < kCommands; i++)
    {
        NodeCommandMailbox::Payload payload;
        payload.profileAcc = i;
        payload.encoderOffset = static_cast<double>(i);
        if (mailbox.publish(NodeCommand::SetEncoder, payload, std::chrono::milliseconds(1000)))
            published++;
    }

    // Let the consumer drain whatever is still in flight before stopping it.
    while (mailbox.pending())
        std::this_thread::yield();
    stop.store(true, std::memory_order_release);
    consumer.join();

    EXPECT_EQ(published, kCommands) << "publish() gave up: the consumer never drained the slot";
    EXPECT_EQ(delivered.load(), published) << "a published command was lost";
    EXPECT_EQ(mismatched.load(), 0) << "a command was delivered with someone else's payload";
}
