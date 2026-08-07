# Security Policy

## Supported code

Security fixes are focused on the current projects under `firmware/`. Content
under `legacy/` is provided for reference and may not receive security updates.

## Reporting

Do not publish credentials, control keys, or an exploit that can move physical
hardware in a public issue. Contact the repository owner privately through the
GitHub account that maintains this repository, and include the affected target,
firmware revision, reproduction steps, and potential physical impact.

## Deployment assumptions

The rover web interface and open access point are intended for controlled test
environments. Operators are responsible for network isolation, physical stop
controls, unique handheld-control keys, and replacing development defaults
before unattended use.
