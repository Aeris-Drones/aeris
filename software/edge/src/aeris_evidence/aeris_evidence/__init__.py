"""Evidence bundling utilities for Aeris bench workflows.

Provides tools for collecting, hashing, and signing test artifacts
from simulation runs and field trials for audit and compliance workflows.

This package exports the evidence bundle CLI functionality through the
`evidence_bundle` module, which can create cryptographically signed
tarballs containing test artifacts with SHA-256 manifests.

Example:
    Create an evidence bundle from test outputs::

        from aeris_evidence.evidence_bundle import main
        import sys
        sys.argv = ["evidence_bundle", "--input-dir", "/tmp/test_outputs"]
        main()

Attributes:
    __version__ (str): Package version string.
"""

__version__ = "0.1.0"
