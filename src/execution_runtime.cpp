#include "execution_runtime.h"

#include <algorithm>
#include <stdexcept>
#include <thread>
#include <vector>

namespace voxel4d {

ExecutionRuntime::ExecutionRuntime(const ExecutionBackend requested,
                                   const std::size_t worker_count) {
    info_.requested = requested;
    switch (requested) {
        case ExecutionBackend::kCpuSerial:
            info_.active = ExecutionBackend::kCpuSerial;
            info_.worker_count = 1U;
            break;
        case ExecutionBackend::kCpuParallel: {
            const std::size_t detected_worker_count =
                std::max<std::size_t>(1U, std::thread::hardware_concurrency());
            info_.worker_count = worker_count == 0U ? detected_worker_count : worker_count;
            info_.worker_count = std::max<std::size_t>(1U, info_.worker_count);
            info_.active = info_.worker_count > 1U ? ExecutionBackend::kCpuParallel
                                                   : ExecutionBackend::kCpuSerial;
            break;
        }
        case ExecutionBackend::kGpu:
        case ExecutionBackend::kNpu:
        case ExecutionBackend::kApu:
            info_.active = ExecutionBackend::kCpuSerial;
            info_.used_fallback = true;
            info_.worker_count = 1U;
            break;
    }
}

const ExecutionBackendInfo& ExecutionRuntime::info() const {
    return info_;
}

void ExecutionRuntime::for_each_index(const std::size_t count,
                                      const std::function<void(std::size_t)>& callback) const {
    if (!callback) {
        throw std::invalid_argument("execution callback must not be empty");
    }
    if (count == 0U) {
        return;
    }

    if (info_.active != ExecutionBackend::kCpuParallel || info_.worker_count <= 1U) {
        for (std::size_t index = 0U; index < count; ++index) {
            callback(index);
        }
        return;
    }

    const std::size_t active_worker_count = std::min(info_.worker_count, count);
    const std::size_t base_block_size = count / active_worker_count;
    const std::size_t remainder = count % active_worker_count;

    std::vector<std::thread> workers;
    workers.reserve(active_worker_count);
    std::size_t block_begin = 0U;
    for (std::size_t worker_index = 0U; worker_index < active_worker_count; ++worker_index) {
        const std::size_t block_size = base_block_size + (worker_index < remainder ? 1U : 0U);
        const std::size_t block_end = block_begin + block_size;
        workers.emplace_back([block_begin, block_end, &callback] {
            for (std::size_t index = block_begin; index < block_end; ++index) {
                callback(index);
            }
        });
        block_begin = block_end;
    }

    for (std::thread& worker : workers) {
        worker.join();
    }
}

const char* execution_backend_name(const ExecutionBackend backend) {
    switch (backend) {
        case ExecutionBackend::kCpuSerial:
            return "CPU serial";
        case ExecutionBackend::kCpuParallel:
            return "CPU parallel";
        case ExecutionBackend::kGpu:
            return "GPU";
        case ExecutionBackend::kNpu:
            return "NPU";
        case ExecutionBackend::kApu:
            return "APU";
    }
    return "unknown";
}

}  // namespace voxel4d
