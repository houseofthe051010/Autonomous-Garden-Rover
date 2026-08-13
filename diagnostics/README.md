# Diagnostics archive

This directory contains dated controller configurations, black-box captures,
and incident notes recorded while commissioning the rover. It is reference
material, not a source of truth for the currently flashed configuration.

When adding a capture:

- Use a dated, descriptive directory or filename.
- Include a short Markdown note with the board, firmware revision, and result.
- Remove Wi-Fi credentials, control keys, personal paths, and device-specific
  identifiers before committing.
- Keep reproducible build inputs and checksums beside release artifacts; avoid
  committing transient logs or full-flash backups.

Follow the target README under [`../firmware/`](../firmware/) for current build,
flash, and safety instructions.
