# Validation Environment

## Reference environment

The current automated validation was executed in an isolated Linux environment with the following observable execution capability.

| Property | Observed value |
|---|---|
| Operating system | Ubuntu 24.04 Linux, x86_64 |
| CPU | Intel Xeon Processor at 2.50 GHz |
| Online logical CPUs | 6 |
| NUMA topology | One reported NUMA node |
| Discrete GPU device | Not exposed to the validation environment |
| NVIDIA driver device / `nvidia-smi` | Not available |
| Implemented execution paths validated here | CPU serial and CPU parallel |

## Local benchmark observation

The fixed 96 × 72 three-iteration renderer benchmark completed in approximately 1.16–1.17 seconds on the serial path, 0.64 seconds with two workers, and 0.36 seconds with six workers in this environment. The command used was `./build/voxel4d_render_benchmark --workers N` for `N = 2` and `N = 6`.

## Interpretation

The repository's portable CPU baseline and parameterized CPU rendering benchmark were exercised on this environment. This evidence does **not** validate GPU, NPU, APU, multi-node, or vendor-specific execution. Those backends remain unavailable at runtime and must continue to report explicit fallback behavior until a target-specific implementation is built and tested on actual hardware.

## Required validation record for a new target

A hardware-specific backend contribution should provide the device model, driver/runtime version, compiler version, operating system, memory limit, worker or stream count, workload dimensions, correctness comparison against the CPU reference, and benchmark methodology. Measurements without this record are local observations rather than portable performance claims.
