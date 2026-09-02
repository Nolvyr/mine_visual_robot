#include "comm/comm_worker.hpp"

#include <atomic>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <vector>

namespace
{

comm::TxFrame MakeFrame(const std::uint32_t sequence)
{
    comm::TxFrame frame;
    frame.sequence = sequence;
    frame.active_map_id = 2;
    frame.bytes = {0xAA, 0x55, static_cast<std::uint8_t>(sequence & 0xFFU)};
    return frame;
}

}  // namespace

int main()
{
    std::mutex processed_mutex;
    std::vector<std::uint32_t> processed_sequences;

    comm::CommWorker worker(
        128,
        [&](const comm::TxFrame& frame) {
            std::lock_guard<std::mutex> lock(processed_mutex);
            processed_sequences.push_back(frame.sequence);
            return true;
        });

    worker.Start();
    worker.Start();

    bool all_enqueued = true;
    for (std::uint32_t sequence = 0; sequence < 100; ++sequence)
    {
        all_enqueued = worker.Enqueue(MakeFrame(sequence)) && all_enqueued;
    }

    worker.Stop();
    worker.Stop();

    const std::vector<comm::TxCompletion> completions =
        worker.DrainCompletions();

    bool fifo = processed_sequences.size() == 100;
    for (std::size_t i = 0; fifo && i < processed_sequences.size(); ++i)
    {
        fifo = processed_sequences[i] == i;
    }

    const comm::CommWorkerStats stats = worker.GetStats();
    const bool counts_ok =
        stats.enqueue_total == 100 &&
        stats.processed_total == 100 &&
        stats.enqueue_rejected == 0 &&
        stats.queue_size == 0 &&
        !stats.running;

    bool completion_fifo = completions.size() == 100;
    for (std::size_t i = 0; completion_fifo && i < completions.size(); ++i)
    {
        completion_fifo = completions[i].sequence == i && completions[i].success;
    }

    comm::CommWorker failure_worker(
        4,
        [](const comm::TxFrame&) { return false; });
    failure_worker.Start();
    const bool failure_enqueued = failure_worker.Enqueue(MakeFrame(500));
    failure_worker.Stop();
    const std::vector<comm::TxCompletion> failure_completions =
        failure_worker.DrainCompletions();
    const bool failure_completion =
        failure_completions.size() == 1 &&
        failure_completions.front().sequence == 500 &&
        !failure_completions.front().success;

    std::atomic<std::size_t> destructor_processed{0};
    {
        comm::CommWorker destructor_worker(
            16,
            [&](const comm::TxFrame&) {
                ++destructor_processed;
                return true;
            });
        destructor_worker.Start();
        all_enqueued =
            destructor_worker.Enqueue(MakeFrame(1000)) && all_enqueued;
    }

    const bool destructor_safe = destructor_processed.load() == 1;
    const bool passed = all_enqueued && fifo && counts_ok
        && completion_fifo && failure_enqueued && failure_completion
        && destructor_safe;

    std::cout
        << "enqueued=" << stats.enqueue_total
        << " processed=" << stats.processed_total
        << " fifo=" << fifo
        << " completion_fifo=" << completion_fifo
        << " failure_completion=" << failure_completion
        << " repeat_stop=1"
        << " destructor_safe=" << destructor_safe
        << " high_watermark=" << stats.queue_high_watermark
        << '\n';

    return passed ? 0 : 1;
}
