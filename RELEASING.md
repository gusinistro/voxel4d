# Release Process

This checklist is for maintainers preparing an open-source Voxel4D release. It is intentionally conservative because the project is pre-1.0 research software.

## Preparation

| Check | Required evidence |
|---|---|
| Scope | README and architecture note accurately state implemented capability and limitations. |
| Version | `CMakeLists.txt`, `CITATION.cff`, and `CHANGELOG.md` use the intended release version. |
| License | `LICENSE` and `NOTICE` are present and dependency attribution remains accurate. |
| Security | No secrets, private data, generated datasets, build outputs, or local configuration files are staged. |
| Community files | Contributing, conduct, security, governance, support, and issue templates are present. |

## Validation

Run the clean build and test suite:

```bash
cmake -S . -B build -DVOXEL4D_WARNINGS_AS_ERRORS=ON -DBUILD_TESTING=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

For C++ memory or parsing changes, also run:

```bash
cmake -S . -B build-sanitized \
  -DVOXEL4D_WARNINGS_AS_ERRORS=ON \
  -DVOXEL4D_ENABLE_SANITIZERS=ON \
  -DBUILD_TESTING=ON
cmake --build build-sanitized --parallel
ASAN_OPTIONS=detect_leaks=1 ctest --test-dir build-sanitized --output-on-failure
```

Review GitHub Actions results on the release commit before publishing a tag.

## Publication

Create an annotated version tag such as `v0.1.0`, publish a GitHub Release with the matching changelog section, and use GitHub-generated source archives instead of committing `.zip` or `.tar.gz` files. Record the commit SHA and CI run URL in the release notes.

If the repository will be transferred to an organization, update the placeholder repository URLs in `README.md`, `CHANGELOG.md`, and `CITATION.cff` before the first public tag.
