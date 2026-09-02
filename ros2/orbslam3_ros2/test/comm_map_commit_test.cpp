#include "comm/comm_map.hpp"
#include "comm/comm_worker.hpp"

#include <cstdint>
#include <iostream>
#include <utility>
#include <vector>

namespace
{

comm::MapPointBatchData MakeOperation(
    const comm::MapPointOperation operation,
    const std::uint32_t id,
    const std::int16_t x_cm)
{
    comm::MapPointBatchData point;
    point.operation = operation;
    point.map_id = 2;
    point.id = id;
    point.x_cm = x_cm;
    point.y_cm = 20;
    point.z_cm = 30;
    return point;
}

}  // namespace

int main()
{
    comm::CommMap map;
    comm::SyncedPointState state;

    const std::vector<comm::MapPointBatchData> add_batch{
        MakeOperation(comm::MapPointOperation::ADD, 7, 10)};

    const bool add_not_committed =
        map.synced_size() == 0 && !map.GetSyncedPoint(7, state);
    const comm::CommCommitResult add_result = map.CommitBatch(add_batch);
    const bool add_committed =
        map.GetSyncedPoint(7, state) && state.x_cm == 10
        && add_result.synced_now == 1 && add_result.add_total == 1;

    const std::vector<comm::MapPointBatchData> update_batch{
        MakeOperation(comm::MapPointOperation::UPDATE, 7, 42)};
    const bool update_not_committed =
        map.GetSyncedPoint(7, state) && state.x_cm == 10;
    const comm::CommCommitResult update_result = map.CommitBatch(update_batch);
    const bool update_committed =
        map.GetSyncedPoint(7, state) && state.x_cm == 42
        && update_result.update_total == 1;

    const std::vector<comm::MapPointBatchData> delete_batch{
        MakeOperation(comm::MapPointOperation::DELETE, 7, 0)};
    const bool delete_not_committed = map.GetSyncedPoint(7, state);
    const comm::CommCommitResult delete_result = map.CommitBatch(delete_batch);
    const bool delete_committed =
        !map.GetSyncedPoint(7, state)
        && delete_result.synced_now == 0 && delete_result.delete_total == 1;

    const std::vector<comm::MapPointBatchData> send_batch{
        MakeOperation(comm::MapPointOperation::ADD, 9, 90)};
    comm::CommMap send_map;

    comm::CommWorker failure_worker(
        4,
        [](const comm::TxFrame&) { return false; });
    failure_worker.Start();
    comm::TxFrame failed_frame;
    failed_frame.sequence = 10;
    const bool failed_enqueued = failure_worker.Enqueue(std::move(failed_frame));
    failure_worker.Stop();
    const auto failed_completions = failure_worker.DrainCompletions();
    if (!failed_completions.empty() && failed_completions.front().success)
    {
        send_map.CommitBatch(send_batch);
    }
    const bool failure_not_committed =
        failed_enqueued && send_map.synced_size() == 0;

    comm::CommWorker success_worker(
        4,
        [](const comm::TxFrame&) { return true; });
    success_worker.Start();
    comm::TxFrame success_frame;
    success_frame.sequence = 11;
    const bool success_enqueued = success_worker.Enqueue(std::move(success_frame));
    success_worker.Stop();
    const auto success_completions = success_worker.DrainCompletions();
    const std::uint32_t wrong_pending_sequence = 12;
    if (!success_completions.empty()
        && success_completions.front().sequence == wrong_pending_sequence
        && success_completions.front().success)
    {
        send_map.CommitBatch(send_batch);
    }
    const bool mismatch_not_committed = send_map.synced_size() == 0;

    if (!success_completions.empty()
        && success_completions.front().sequence == 11
        && success_completions.front().success)
    {
        send_map.CommitBatch(send_batch);
    }
    const bool success_committed =
        success_enqueued && send_map.GetSyncedPoint(9, state);

    const bool passed = add_not_committed && add_committed
        && update_not_committed && update_committed
        && delete_not_committed && delete_committed
        && failure_not_committed && mismatch_not_committed
        && success_committed;

    std::cout
        << "add_deferred=" << add_not_committed
        << " add_commit=" << add_committed
        << " update_deferred=" << update_not_committed
        << " update_commit=" << update_committed
        << " delete_deferred=" << delete_not_committed
        << " delete_commit=" << delete_committed
        << " failure_no_commit=" << failure_not_committed
        << " mismatch_no_commit=" << mismatch_not_committed
        << " success_commit=" << success_committed
        << '\n';

    return passed ? 0 : 1;
}
