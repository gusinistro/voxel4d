# Security Policy

## Supported versions

Voxel4D is currently pre-1.0 research software. Security fixes are applied to the latest development branch and the latest tagged release when feasible.

| Version | Supported |
|---|---|
| Latest development branch | Yes, best effort |
| Latest tagged pre-1.0 release | Yes, best effort |
| Earlier releases | No |

## Reporting a vulnerability

**Do not open a public GitHub issue for a suspected vulnerability.** Until a dedicated private security advisory channel is enabled on GitHub, contact the repository owner privately through the email address listed in the repository's GitHub profile. Include `Voxel4D security report` in the subject.

A useful report includes a concise description, affected files and versions, steps to reproduce, potential impact, suggested mitigations if known, and whether you have disclosed the issue elsewhere. Please avoid sending exploit payloads that could harm systems.

Maintainers will acknowledge a valid report as soon as practical, assess reproducibility and impact, work on remediation, and coordinate disclosure with the reporter. Exact timelines cannot be guaranteed for this volunteer research project.

## Security boundaries

The current PoC is designed for local synthetic data. It is not hardened for untrusted files, network exposure, multi-tenant deployment, physical-security decisions, surveillance, medical use, transportation, industrial control, or any other safety-critical setting. Treat any future real-sensor ingestion, model execution, or networked component as a new security-review scope.
