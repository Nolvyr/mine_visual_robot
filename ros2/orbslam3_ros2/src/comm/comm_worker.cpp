#include "comm/comm_worker.hpp"

#include <iostream>
#include <stdexcept>
#include <utility>

namespace comm
{

CommWorker::CommWorker(
    const std::size_t max_queue_size,
    FrameHandler frame_handler)
    : max_queue_size_(max_queue_size),
      frame_handler_(std::move(frame_handler))
{
    if (max_queue_size_ == 0)
    {
        throw std::invalid_argument("CommWorker queue size must be greater than zero");
    }
}

CommWorker::~CommWorker()
{
    Stop();
}

void CommWorker::Start()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_)
    {
        return;
    }

    stop_requested_ = false;
    running_ = true;
    worker_thread_ = std::thread(&CommWorker::WorkerLoop, this);
}

void CommWorker::Stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_)
        {
            return;
        }
        stop_requested_ = true;
    }

    condition_.notify_all();
    if (worker_thread_.joinable())
    {
        worker_thread_.join();
    }

    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
}

bool CommWorker::Enqueue(TxFrame frame)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ || stop_requested_ || queue_.size() >= max_queue_size_)
        {
            ++enqueue_rejected_;
            return false;
        }

        queue_.push_back(std::move(frame));
        ++enqueue_total_;
        if (queue_.size() > queue_high_watermark_)
        {
            queue_high_watermark_ = queue_.size();
        }
    }

    condition_.notify_one();
    return true;
}

std::vector<TxCompletion> CommWorker::DrainCompletions()
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<TxCompletion> completions;
    completions.reserve(completion_queue_.size());
    while (!completion_queue_.empty())
    {
        completions.push_back(completion_queue_.front());
        completion_queue_.pop_front();
    }
    return completions;
}

CommWorkerStats CommWorker::GetStats() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    CommWorkerStats stats;
    stats.queue_size = queue_.size();
    stats.enqueue_total = enqueue_total_;
    stats.processed_total = processed_total_;
    stats.queue_high_watermark = queue_high_watermark_;
    stats.enqueue_rejected = enqueue_rejected_;
    stats.running = running_;
    return stats;
}

void CommWorker::WorkerLoop()
{
    while (true)
    {
        TxFrame frame;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this]() {
                return stop_requested_ || !queue_.empty();
            });

            if (queue_.empty())
            {
                if (stop_requested_)
                {
                    break;
                }
                continue;
            }

            frame = std::move(queue_.front());
            queue_.pop_front();
        }

        const bool success = frame_handler_
            ? frame_handler_(frame)
            : SimulateSend(frame);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            completion_queue_.push_back(TxCompletion{frame.sequence, success});
            ++processed_total_;
        }
    }
}

bool CommWorker::SimulateSend(const TxFrame& frame) const
{
    std::cout
        << "[CommWorkerV6.0] simulated_tx seq=" << frame.sequence
        << " active_map=" << frame.active_map_id
        << " size=" << frame.bytes.size() << "B\n";
    return true;
}

}  // namespace comm
