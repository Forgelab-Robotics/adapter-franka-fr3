# Security policy

## Reporting a vulnerability

Do not open a public issue for a suspected vulnerability. Use GitHub's private
vulnerability reporting for this repository. Include affected versions,
reproduction steps, impact, and any suggested mitigation.

## Physical robot boundary

This package controls a real Franka FR3 arm and Hand when pointed at hardware:

- The default backend is `fake`. Real motion requires `--backend franky` plus
  the explicit `--execute` flag (CLI) or `allow_real_motion: true` with a
  trusted robot IP (Dora node config).
- The driver's software stop and the fail-closed action validation are not
  safety functions. An operator-accessible Franka Desk / external emergency
  stop must be used for all hardware validation.
- Treat robot configurations as potentially sensitive; the config file must
  not contain credentials (see `config/robot.example.yaml`).

Security fixes are supported on the latest released version.
