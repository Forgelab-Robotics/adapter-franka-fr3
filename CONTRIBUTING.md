# Contributing

Thank you for contributing to franka-fr3 (FR3v2 + Franka Hand adapter).

## Development

Use Python 3.12 and `uv`:

```bash
uv sync --frozen --all-groups
uv run pytest -q
uv run robots-franka-fr3 --help
uv run robots-franka-fr3 --version
```

Keep changes focused and add tests for behavior changes. Update the README and
`OPEN_SOURCE_AUDIT.md` when dependencies, bundled assets, or safety-relevant
defaults change.

## Physical robot safety

- The default backend is `fake`; real motion additionally requires
  `--backend franky` plus the explicit `--execute` flag (CLI) or
  `allow_real_motion: true` (Dora node config).
- Software stop is not a safety stop. Validate on hardware only with a
  Franka Desk / external emergency stop reachable by an operator.
- Do not run hardware tests unattended, and do not commit recordings,
  credentials, or machine-specific paths.

## Security and generated data

- Report vulnerabilities privately as described in `SECURITY.md`.
- Never commit credentials, private repository URLs, recordings, runtime state,
  machine-specific paths, or personal data.

By submitting a contribution, you agree that it is licensed under Apache
License 2.0.
