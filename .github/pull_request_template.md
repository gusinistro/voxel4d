## Summary

Describe the user-visible change and its motivation.

## Scope and design

Explain the data contracts, units, assumptions, or API changes. For research features, clearly separate what is implemented from what remains proposed.

## Validation

State the commands you ran and their results.

```text
cmake -S . -B build -DVOXEL4D_WARNINGS_AS_ERRORS=ON -DBUILD_TESTING=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Checklist

- [ ] I kept the pull request focused and documented any intentional API break.
- [ ] I added or updated tests, or explained why a test is not practical.
- [ ] I updated relevant documentation, limitations, and changelog material.
- [ ] I did not add secrets, personal data, proprietary assets, generated build files, or large binaries.
- [ ] I have read and followed `CONTRIBUTING.md` and `CODE_OF_CONDUCT.md`.
- [ ] I agree to license my contribution under the MIT License.
