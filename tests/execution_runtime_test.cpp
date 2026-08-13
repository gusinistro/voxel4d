#include "execution_runtime.h"

#include <stdexcept>
#include <vector>

#include "test_support.h"

int main() {
    voxel4d::test::TestContext test;

    const voxel4d::ExecutionRuntime serial(voxel4d::ExecutionBackend::kCpuSerial);
    test.expect(serial.info().active == voxel4d::ExecutionBackend::kCpuSerial,
                "Serial request must retain CPU serial execution");
    test.expect(!serial.info().used_fallback, "Serial request must not report a fallback");

    std::vector<int> serial_values(5U, -1);
    serial.for_each_index(serial_values.size(), [&serial_values](const std::size_t index) {
        serial_values.at(index) = static_cast<int>(index);
    });
    for (std::size_t index = 0U; index < serial_values.size(); ++index) {
        test.expect(serial_values.at(index) == static_cast<int>(index),
                    "Serial runtime must execute every index exactly once");
    }

    const voxel4d::ExecutionRuntime parallel(voxel4d::ExecutionBackend::kCpuParallel, 3U);
    test.expect(parallel.info().active == voxel4d::ExecutionBackend::kCpuParallel,
                "Explicit multi-worker request must activate CPU parallel execution");
    test.expect(parallel.info().worker_count == 3U,
                "Parallel runtime must retain explicit worker count");
    std::vector<int> parallel_values(31U, -1);
    parallel.for_each_index(parallel_values.size(), [&parallel_values](const std::size_t index) {
        parallel_values.at(index) = static_cast<int>(index * index);
    });
    for (std::size_t index = 0U; index < parallel_values.size(); ++index) {
        test.expect(parallel_values.at(index) == static_cast<int>(index * index),
                    "Parallel runtime must execute every index exactly once");
    }

    const voxel4d::ExecutionRuntime gpu_request(voxel4d::ExecutionBackend::kGpu);
    test.expect(gpu_request.info().active == voxel4d::ExecutionBackend::kCpuSerial,
                "Unavailable GPU request must use deterministic CPU serial fallback");
    test.expect(gpu_request.info().used_fallback,
                "Unavailable accelerated backend must report a fallback");
    test.expect(
        std::string(voxel4d::execution_backend_name(voxel4d::ExecutionBackend::kNpu)) == "NPU",
        "Execution backend names must expose stable labels");
    const std::vector<voxel4d::ExecutionBackendCapability> capabilities =
        voxel4d::execution_backend_capabilities();
    test.expect(capabilities.size() == 5U && capabilities.at(0).implemented &&
                    !capabilities.at(2).implemented,
                "Capability report must distinguish implemented CPU from unavailable GPU backends");

    test.expect_throws<std::runtime_error>(
        [&] {
            parallel.for_each_index(8U, [](const std::size_t index) {
                if (index == 3U) {
                    throw std::runtime_error("worker failure");
                }
            });
        },
        "Parallel runtime must propagate worker callback failures after joining threads");
    test.expect_throws<std::invalid_argument>(
        [&] { serial.for_each_index(1U, std::function<void(std::size_t)>{}); },
        "Runtime must reject an empty execution callback");

    return test.failures() == 0 ? 0 : 1;
}
