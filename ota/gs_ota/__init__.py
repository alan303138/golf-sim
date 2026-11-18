"""gs_ota: signed OTA updates for the golf simulator (docs/ARCHITECTURE.md section 8).

Public entry points for other components:
  gs_ota.client:AgentUpdater   Updater for gs_service (talks to the agent socket)
  gs_ota.updater:OtaUpdater    in-process Updater over an UpdateEngine
  gs_ota.agent:main            device agent daemon / admin CLI
  gs_ota.tools:main            release tooling
"""
from .manifest import Artifact, Manifest, ManifestError, SignatureError, verify_manifest
from .semver import Version

__version__ = "0.1.0"

__all__ = ["Artifact", "Manifest", "ManifestError", "SignatureError", "verify_manifest",
           "Version", "installed_version"]


def installed_version() -> str | None:
    """Version of the running sensor release (set by the bundle launcher)."""
    import os
    return os.environ.get("GS_SW_VERSION")
