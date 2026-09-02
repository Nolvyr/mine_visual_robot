#ifndef ORBSLAM3_ROS2_COMM_WORKER_HPP
#define ORBSLAM3_ROS2_COMM_WORKER_HPP

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "comm/comm_types.hpp"

namespace comm
{

class CommWorker
{
public:
    using FrameHandler = std::function<bool(const TxFrame&)>;

    explicit CommWorker(
        std::size_t max_queue_size = 16,
        FrameHandler frame_handler = FrameHandler{});
    ~CommWorker();

    CommWorker(const CommWorker&) = delete;
    CommWorker& operator=(const CommWorker&) = delete;

    void Start();
    void Stop();
    bool Enqueue(TxFrame frame);
    std::vector<TxCompletion> DrainCompletions();
    CommWorkerStats GetStats() const;

private:
    void WorkerLoop();
    bool SimulateSend(const TxFrame& frame) const;

    const std::size_t max_queue_size_;
    FrameHandler frame_handler_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<TxFrame> queue_;
    std::deque<TxCompletion> completion_queue_;
    std::thread worker_thread_;
    bool running_{false};
    bool stop_requested_{false};
    std::size_t enqueue_total_{0};
    std::size_t processed_total_{0};
    std::size_t queue_high_watermark_{0};
    std::size_t enqueue_rejected_{0};
};

}  // namespace comm

#endif
