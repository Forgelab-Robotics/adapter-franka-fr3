# Open-source release audit

Audit date: 2026-09-09

## Decision

The audited release is suitable for publication with its existing Git history,
branches, and tags. Project-owned code, configuration, tests, and documentation
use Apache-2.0. The bundled robot description assets retain their
`franka_description` Apache-2.0 (with BSD clause) provenance as documented in
`NOTICE` and `THIRD_PARTY_NOTICES.md`.

## Completed checks

- Public dependency resolution succeeds with `uv sync --frozen --all-groups`.
- `forge-common==1.0.1`, `forge-msgs==1.3.0`, and `forge-robot==1.0.1`
  resolve from PyPI.
- `franky-control==2.0.0+libfranka.0.17.0` resolves from the public franky
  wheel index (robot-server 9). This distribution is not published on PyPI,
  so the `pip-audit` run audits the PyPI-resolvable subset of the locked
  requirements and reports no known vulnerabilities. The franky wheel is
  pinned exactly and its bundled native libraries ship unmodified.
- All 87 tests (plus 8 subtests) pass on Python 3.12 without physical
  hardware.
- CLI reports `franka-fr3 0.1.0` and lists all 8 subcommands; the Dora node
  entry requires `--config` and exits 2 without it.
- The MuJoCo scene loads from `assets/mjcf/scene.xml` (nq=9, nu=8).
- `detect-secrets` findings on the tracked tree were verified one by one as
  false positives (sha256 checksums in asset provenance manifests, an
  upstream commit hash `7aeeddc449edf8d62b594f9e36a81da53e7796f9`, and
  high-entropy Base64 matches on upstream COLLADA mesh node attributes);
  CI excludes these files from the secret scan by regex.
- A high-confidence secret-pattern scan across all Git refs and historical
  blobs (7 revisions) reports zero findings.
- Private repository URLs and machine-specific paths are absent from the
  current publishable tree. The initial commit (ab25105) README still
  contains the former internal GitLab domain as historical provenance; it is
  not a current installation instruction.
- No file approaches GitHub's 100 MiB hard limit; the largest tracked file is
  9.1 MiB and the tree holds 125 tracked files.
- GitHub Actions (CI, Ubuntu 20.04 binary build), Dependabot, issue and
  pull-request templates, contribution, and security policy files are
  included.

## License findings

- Project-owned Python, configuration, tests, and documentation: Apache-2.0.
- Bundled `franka_description` assets: Apache-2.0 with the BSD clause noted
  in `NOTICE`.
- Runtime dependencies are not otherwise vendored and retain the licenses
  declared by their distributions.
- `franky-control`: MIT (wheel index distribution; native libraries bundled
  in the binary release).
- `dora-rs`: MIT.
- `forge-common`, `forge-msgs`, `forge-robot`: Apache-2.0.
- PyYAML: MIT. Typer: MIT.
- MuJoCo: dev-group only, Apache-2.0.
- PyInstaller is a build-only dependency under GPL-2.0-or-later with its
  special exception.

See `THIRD_PARTY_NOTICES.md` for provenance and binary-distribution cautions.

## Known limitations

- The franky-control wheel is distributed from a third-party public index,
  not PyPI; `pip-audit` therefore covers the PyPI-resolvable subset only.
- CI and this audit do not command physical hardware. The `fake` backend is
  the default; real motion additionally requires the `--execute` flag or
  `allow_real_motion: true`.
- Software stop and fail-closed action validation are not safety functions;
  an operator-accessible emergency stop is required for hardware validation.
- The default robot IP `172.16.0.2` and System Image `5.8.1` are pending
  on-site confirmation.
- PyInstaller binary releases require a separate artifact-level license and
  security review.
- This audit is an engineering review, not legal advice, penetration testing,
  or safety certification.

## Publication model

The audited current tree is published together with the repository's existing
commit graph, branches, and tags. Historical commits are retained for
traceability and may contain obsolete internal repository locations; they must
not be treated as current installation instructions.
