# Contributing to Voxel4D

Thank you for considering a contribution. Voxel4D is a research proof of concept, so contributions should improve correctness, reproducibility, documentation, tests, or the path toward real sensor support without overstating current capability.

## Before you start

Please search existing issues and pull requests before opening a new one. For material changes to architecture, data formats, public APIs, or physics assumptions, open an issue or discussion first so the design can be reviewed before substantial implementation work begins.

Security vulnerabilities must not be reported through public issues; follow [SECURITY.md](SECURITY.md) instead.

## Development setup

Use the same baseline build as continuous integration:

```bash
cmake -S . -B build -DVOXEL4D_WARNINGS_AS_ERRORS=ON -DBUILD_TESTING=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

For changes touching memory, ownership, input parsing, or numerical traversal, also run the sanitizer configuration:

```bash
cmake -S . -B build-sanitized \
  -DVOXEL4D_WARNINGS_AS_ERRORS=ON \
  -DVOXEL4D_ENABLE_SANITIZERS=ON \
  -DBUILD_TESTING=ON
cmake --build build-sanitized --parallel
ASAN_OPTIONS=detect_leaks=1 ctest --test-dir build-sanitized --output-on-failure
```

## Contribution expectations

| Area | Expectation |
|---|---|
| Code | Use C++17, meaningful names, explicit SI units, deterministic behavior, and no compiler warnings under the supported configurations. |
| Tests | Add or update a test whenever behavior changes or a bug is fixed. |
| Documentation | Keep README, architecture notes, API comments, and limitations aligned with implementation. |
| Claims | Do not describe planned research as an implemented feature or performance result. |
| Dependencies | Discuss new dependencies before adding them. State their license and rationale. |
| Data | Do not commit personal data, credentials, proprietary datasets, or large generated artifacts. |

## Pull request checklist

Before requesting review, please confirm that the pull request:

- explains the motivation, scope, and test evidence;
- keeps changes focused and avoids unrelated formatting churn;
- builds with `VOXEL4D_WARNINGS_AS_ERRORS=ON`;
- passes CTest and, when relevant, sanitizer checks;
- includes tests or a documented reason why a test is impractical;
- updates documentation and release notes where appropriate;
- preserves the MIT license and respects third-party notices; and
- follows the [Code of Conduct](CODE_OF_CONDUCT.md).

## Commit and review style

Use concise, imperative commit subjects such as `Fix DDA entry-cell handling`. Pull requests should explain observable behavior rather than only implementation details. Review comments should be specific, respectful, and focused on technical merit.

## Licensing contributions

By submitting a contribution, you agree that your contribution is licensed under the repository's [MIT License](LICENSE). You confirm that you have the right to submit the material and that it does not contain confidential information, credentials, or third-party material without permission.
