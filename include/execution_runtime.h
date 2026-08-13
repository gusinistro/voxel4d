#pragma once

#include <cstddef>
#include <functional>
#include <vector>

namespace voxel4d {

/** @brief Requested execution strategy for independent data-parallel work. */
enum class ExecutionBackend {
    kCpuSerial,
    kCpuParallel,
    kGpu,
    kNpu,
    kApu,
};

/** @brief Resolution result for a requested backend on the current build. */
struct ExecutionBackendInfo {
    ExecutionBackend requested{ExecutionBackend::kCpuSerial};
    ExecutionBackend active{ExecutionBackend::kCpuSerial};
    bool used_fallback{false};
    std::size_t worker_count{1U};
};

/** @brief Build-time availability statement for an execution backend. */
struct ExecutionBackendCapability {
    ExecutionBackend backend{ExecutionBackend::kCpuSerial};
    bool implemented{false};
    bool available_on_current_host{false};
};

/**
 * @brief Portable execution facade for independent index ranges.
 *
 * CPU serial and CPU parallel execution are implemented in this baseline.
 * GPU, NPU, and APU requests are accepted as architecture-level intents and
 * deterministically fall back to CPU serial until a dedicated backend is built.
 * Callbacks must be safe for concurrent invocation when CPU parallel is active.
 */
class ExecutionRuntime {
   public:
    /**
     * @param requested Requested backend intent.
     * @param worker_count CPU parallel worker count; zero selects hardware concurrency.
     */
    explicit ExecutionRuntime(ExecutionBackend requested = ExecutionBackend::kCpuSerial,
                              std::size_t worker_count = 0U);

    [[nodiscard]] const ExecutionBackendInfo& info() const;

    /** @brief Executes callback once for each index in [0, count). */
    void for_each_index(std::size_t count, const std::function<void(std::size_t)>& callback) const;

   private:
    ExecutionBackendInfo info_;
};

[[nodiscard]] const char* execution_backend_name(ExecutionBackend backend);

/** @return Explicitly reported backend capabilities for this build and host. */
[[nodiscard]] std::vector<ExecutionBackendCapability> execution_backend_capabilities();

}  // namespace voxel4d
