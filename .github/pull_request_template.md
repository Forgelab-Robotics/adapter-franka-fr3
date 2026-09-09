## Summary

-

## Validation

- [ ] `uv sync --frozen --all-groups` succeeds.
- [ ] `uv run pytest -q` passes.
- [ ] CLI help and version checks pass.
- [ ] Locked runtime dependencies pass `pip-audit`.
- [ ] No secrets, recordings, machine paths, or private repository URLs are included.
- [ ] License notices are updated when dependencies or bundled resources change.

## Robot safety

- [ ] The change does not weaken the `fake`-backend default or the `--execute` /
      `allow_real_motion` gates.
- [ ] Any physical robot validation used independent safety controls and an
      operator-accessible emergency stop.

Describe physical validation performed (or state that none was needed):